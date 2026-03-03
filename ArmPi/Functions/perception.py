import time
import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple

import cv2
import numpy as np

from LABConfig import color_range
from ArmIK.Transform import getROI, getMaskROI, getCenter, convertCoordinate
from CameraCalibration.CalibrationConfig import square_length


@dataclass
class Detection:
    raw_color: str                  # color that won max-area THIS frame
    voted_color: Optional[str]      # smoothed color label (None until vote ready)
    world_x: float
    world_y: float
    rotation_deg: float
    area: float
    stable: bool


class ColorBlockPerception:
    """
    Perception pipeline that matches ColorSorting/ColorPalletizing style:
    - find max-area blob across candidate colors
    - optional ROI masking after first detection (until pick)
    - optional N-frame color voting
    - stability timer to decide when object is "still"
    """

    def __init__(
        self,
        target_colors: Iterable[str] = ("red", "green", "blue"),
        frame_size: Tuple[int, int] = (640, 480),
        min_blob_area: float = 2500.0,
        min_contour_area: float = 300.0,
        morph_kernel: int = 6,
        # stability
        stable_dist_thresh: float = 0.5,
        stable_time_sec: float = 1.0,
        # color vote (like ColorSorting: len(color_list)==3 then avg)
        vote_len: int = 3,
        # ROI masking mode: "never" | "after_detect" | "during_pick"
        roi_mask_mode: str = "after_detect",
    ):
        self.target_colors = tuple(target_colors)
        self.size = frame_size
        self.min_blob_area = float(min_blob_area)
        self.min_contour_area = float(min_contour_area)
        self.morph_kernel = int(morph_kernel)

        self.stable_dist_thresh = float(stable_dist_thresh)
        self.stable_time_sec = float(stable_time_sec)

        self.vote_len = int(vote_len)
        self.roi_mask_mode = roi_mask_mode

        # ROI state
        self.roi = None
        self.has_roi = False

        # stability state
        self.last_world = None            # (x,y)
        self.stable_start_t = None
        self.center_buf = []              # [(x,y), ...]

        # color vote state
        self.vote_buf = []                # [1,2,3,...] numeric
        self.last_voted_color = None

        # last rotation
        self.last_rotation = 0.0

    # pipeline steps

    def preprocess(self, frame_bgr: np.ndarray) -> np.ndarray:
        frame_resize = cv2.resize(frame_bgr, self.size, interpolation=cv2.INTER_NEAREST)
        frame_blur = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        return frame_blur

    def apply_roi_mask(self, frame_bgr: np.ndarray, picking_phase: bool) -> np.ndarray:
        if not self.has_roi or self.roi is None:
            return frame_bgr

        if self.roi_mask_mode == "never":
            return frame_bgr
        if self.roi_mask_mode == "after_detect" and picking_phase:
            # after_detect means: mask while detecting/tracking, but stop masking once pick starts
            return frame_bgr
        if self.roi_mask_mode == "during_pick" and not picking_phase:
            return frame_bgr

        return getMaskROI(frame_bgr, self.roi, self.size)

    def to_lab(self, frame_bgr: np.ndarray) -> np.ndarray:
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)

    def threshold_and_morph(self, frame_lab: np.ndarray, color: str) -> np.ndarray:
        lower, upper = color_range[color]
        mask = cv2.inRange(frame_lab, lower, upper)
        k = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, k)
        return closed

    def largest_contour(self, contours) -> Tuple[Optional[np.ndarray], float]:
        best = None
        best_area = 0.0
        for c in contours:
            a = abs(cv2.contourArea(c))
            if a > best_area:
                best_area = a
                if a > self.min_contour_area:
                    best = c
        return best, best_area

    def select_best_blob(self, frame_lab: np.ndarray) -> Tuple[Optional[str], Optional[np.ndarray], float]:
        """
        ColorSorting behavior:
        evaluate candidate colors and choose the blob with max area overall.
        """
        best_color = None
        best_contour = None
        best_area = 0.0

        for color in self.target_colors:
            mask = self.threshold_and_morph(frame_lab, color)
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            c, a = self.largest_contour(contours)
            if c is not None and a > best_area:
                best_area = a
                best_contour = c
                best_color = color

        return best_color, best_contour, best_area

    def reset_state_no_detection(self):
        self.has_roi = False
        self.roi = None
        self.last_world = None
        self.stable_start_t = None
        self.center_buf = []
        self.vote_buf = []
        self.last_voted_color = None

    def update_color_vote(self, raw_color: str) -> Optional[str]:
        """
        ColorSorting-style vote:
        map colors to ints, average over vote_len, round back to color.
        returns None until vote_len samples are collected.
        """
        mapping = {"red": 1, "green": 2, "blue": 3}
        inv = {1: "red", 2: "green", 3: "blue"}

        self.vote_buf.append(mapping.get(raw_color, 0))
        if len(self.vote_buf) < self.vote_len:
            return None

        voted_val = int(round(float(np.mean(np.array(self.vote_buf)))))
        self.vote_buf = []
        voted = inv.get(voted_val, None)
        self.last_voted_color = voted
        return voted

    def update_stability(self, xy: Tuple[float, float]) -> Tuple[bool, Tuple[float, float]]:
        """
        Returns (is_stable, (mean_x, mean_y_if_stable_else_current)).
        """
        if self.last_world is None:
            self.last_world = xy
            self.stable_start_t = time.time()
            self.center_buf = [xy]
            return False, xy

        dist = math.sqrt((xy[0] - self.last_world[0]) ** 2 + (xy[1] - self.last_world[1]) ** 2)
        self.last_world = xy

        if dist >= self.stable_dist_thresh:
            self.stable_start_t = time.time()
            self.center_buf = [xy]
            return False, xy

        self.center_buf.append(xy)
        if self.stable_start_t is None:
            self.stable_start_t = time.time()

        if (time.time() - self.stable_start_t) >= self.stable_time_sec:
            arr = np.array(self.center_buf)
            mean_xy = arr.mean(axis=0)
            return True, (float(mean_xy[0]), float(mean_xy[1]))

        return False, xy

    def process_frame(self, frame_bgr: np.ndarray, picking_phase: bool = False) -> Optional[Detection]:
        """
        Full perception step:
        - returns Detection (raw_color, voted_color, world pose, stable flag) or None
        """
        frame = self.preprocess(frame_bgr)
        frame = self.apply_roi_mask(frame, picking_phase)
        lab = self.to_lab(frame)

        raw_color, contour, area = self.select_best_blob(lab)
        if contour is None or raw_color is None or area < self.min_blob_area:
            self.reset_state_no_detection()
            return None

        # rectangle fit -> ROI update for next frames
        rect = cv2.minAreaRect(contour)
        self.last_rotation = float(rect[2])
        box = np.int0(cv2.boxPoints(rect))
        self.roi = getROI(box)
        self.has_roi = True

        # pixel center -> world
        img_centerx, img_centery = getCenter(rect, self.roi, self.size, square_length)
        world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)

        # color vote smoothing
        voted_color = self.update_color_vote(raw_color)

        # stability check
        stable, (mean_x, mean_y) = self.update_stability((float(world_x), float(world_y)))

        return Detection(
            raw_color=raw_color,
            voted_color=voted_color,
            world_x=mean_x if stable else float(world_x),
            world_y=mean_y if stable else float(world_y),
            rotation_deg=self.last_rotation,
            area=float(area),
            stable=stable,
        )

    def annotate(self, frame_bgr: np.ndarray, det: Optional[Detection]) -> np.ndarray:
        out = frame_bgr.copy()
        h, w = out.shape[:2]
        cv2.line(out, (0, h // 2), (w, h // 2), (0, 0, 200), 1)
        cv2.line(out, (w // 2, 0), (w // 2, h), (0, 0, 200), 1)

        if det is None:
            cv2.putText(out, "No detection", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
            return out

        vote_txt = det.voted_color if det.voted_color is not None else "voting..."
        msg = f"raw={det.raw_color} voted={vote_txt} xy=({det.world_x:.2f},{det.world_y:.2f}) rot={det.rotation_deg:.1f}"
        if det.stable and det.voted_color is not None:
            msg += "  READY"

        color = (0, 255, 0) if (det.stable and det.voted_color is not None) else (0, 0, 255)
        cv2.putText(out, msg, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        return out