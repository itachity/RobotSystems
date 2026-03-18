#!/usr/bin/python3
# coding=utf-8

import sys
import time
import math
import random
from dataclasses import dataclass
from collections import defaultdict

import cv2
import numpy as np

sys.path.append('/home/pi/ArmPi/')
import Camera
from LABConfig import color_range
from ArmIK.Transform import getROI, getCenter, convertCoordinate
from CameraCalibration.CalibrationConfig import square_length

from motion import ArmMotion, MotionConfig


# ----------------------------
# Data structures
# ----------------------------
@dataclass
class BlockDetection:
    color: str
    world_x: float
    world_y: float
    rotation_deg: float
    area: float
    box: np.ndarray


# ----------------------------
# Perception helpers
# ----------------------------
RANGE_RGB = {
    'red':   (0, 0, 255),
    'green': (0, 255, 0),
    'blue':  (255, 0, 0),
}

TARGET_COLORS = ("red", "green", "blue")
FRAME_SIZE = (640, 480)
MIN_CONTOUR_AREA = 300
MIN_BLOB_AREA = 2500


def get_area_max_contour(contours):
    best = None
    best_area = 0.0
    for c in contours:
        a = abs(cv2.contourArea(c))
        if a > best_area:
            best_area = a
            if a > MIN_CONTOUR_AREA:
                best = c
    return best, best_area


def detect_blocks(frame_bgr):
    """
    Detect up to one block per color (largest blob of each color).
    This is enough for the red/green/blue cube chaos project.
    Returns: resized_frame, [BlockDetection, ...]
    """
    frame_rs = cv2.resize(frame_bgr, FRAME_SIZE, interpolation=cv2.INTER_NEAREST)
    frame_blur = cv2.GaussianBlur(frame_rs, (11, 11), 11)
    frame_lab = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2LAB)

    detections = []

    for color in TARGET_COLORS:
        lower, upper = color_range[color]
        mask = cv2.inRange(frame_lab, lower, upper)

        k = np.ones((6, 6), np.uint8)
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, k)

        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        contour, area = get_area_max_contour(contours)

        if contour is None or area < MIN_BLOB_AREA:
            continue

        rect = cv2.minAreaRect(contour)
        box = np.int0(cv2.boxPoints(rect))
        roi = getROI(box)
        img_centerx, img_centery = getCenter(rect, roi, FRAME_SIZE, square_length)
        world_x, world_y = convertCoordinate(img_centerx, img_centery, FRAME_SIZE)

        detections.append(
            BlockDetection(
                color=color,
                world_x=float(world_x),
                world_y=float(world_y),
                rotation_deg=float(rect[2]),
                area=float(area),
                box=box,
            )
        )

    return frame_rs, detections


def average_detections(samples_by_color, min_samples=4):
    """
    Average detections collected over a short time window to reduce jitter.
    """
    stable = []

    for color, samples in samples_by_color.items():
        if len(samples) < min_samples:
            continue

        xs = [s.world_x for s in samples]
        ys = [s.world_y for s in samples]
        rs = [s.rotation_deg for s in samples]
        areas = [s.area for s in samples]

        # Use last box for drawing; average pose for action
        last_box = samples[-1].box

        stable.append(
            BlockDetection(
                color=color,
                world_x=float(np.mean(xs)),
                world_y=float(np.mean(ys)),
                rotation_deg=float(np.mean(rs)),
                area=float(np.mean(areas)),
                box=last_box,
            )
        )

    return stable


# ----------------------------
# Visualization helpers
# ----------------------------
def draw_scene(frame_rs, blocks, status_text="Scanning...", color=(0, 0, 255)):
    out = frame_rs.copy()
    h, w = out.shape[:2]

    cv2.line(out, (0, h // 2), (w, h // 2), (0, 0, 200), 1)
    cv2.line(out, (w // 2, 0), (w // 2, h), (0, 0, 200), 1)

    for b in blocks:
        cv2.drawContours(out, [b.box], -1, RANGE_RGB[b.color], 2)
        label = f"{b.color} ({b.world_x:.2f},{b.world_y:.2f})"
        cv2.putText(
            out,
            label,
            (min(b.box[0, 0], b.box[2, 0]), max(20, b.box[2, 1] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            RANGE_RGB[b.color],
            1,
        )

    cv2.putText(out, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    return out


def show_status(frame_rs, blocks, status_text, color=(0, 255, 255), hold_ms=250):
    vis = draw_scene(frame_rs, blocks, status_text, color=color)
    cv2.imshow("Color Chaos", vis)
    cv2.waitKey(max(1, hold_ms))


# ----------------------------
# Chaos motion helpers
# ----------------------------
YEET_ZONES = [
    (-4.0, 16.0),
    (4.0, 16.0),
    (9.0, 12.0),
    (-9.0, 12.0),
]


def dance_with_block(arm, cx, cy, z=12.0, radius=8, loops=1):
    """
    Dynamic dance with:
    - side swagger
    - forward/back bounce
    - diagonal cross-step
    - wrist flicks
    - center flourish
    """

    poses = [
        # x, y, z, move_time_ms, pause_s, wrist_pulse
        (cx,           cy,           z,     2*350, 0.10, 500),

        # swagger
        (cx + radius,  cy,           z+0.5, 2*420, 0.12, 380),
        (cx - radius,  cy,           z+0.5, 2*420, 0.12, 620),

        # forward/back bounce
        (cx,           cy + radius,  z-0.2, 2*420, 0.12, 420),
        (cx,           cy - radius,  z-0.2, 2*420, 0.12, 580),

        # diagonal cross-step
        (cx + radius,  cy + radius,  z+0.8, 2*450, 0.15, 360),
        (cx - radius,  cy - radius,  z+0.8, 2*450, 0.15, 640),
        (cx - radius,  cy + radius,  z,     2*450, 0.15, 400),
        (cx + radius,  cy - radius,  z,     2*450, 0.15, 600),

        # center flourish
        (cx,           cy,           z+1.2, 2*500, 0.18, 350),
        (cx,           cy,           z-0.3, 2*350, 0.12, 650),
        (cx,           cy,           z,     2*350, 0.15, 500),
    ]

    for _ in range(loops):
        for x, y, zpos, t_ms, pause, wrist in poses:
            arm.move_xyz(x, y, zpos, t_ms=t_ms)
            arm.set_wrist(wrist, ms=max(200, t_ms // 2))
            time.sleep(pause)

        # quick wrist shimmy at the end of each loop
        arm.set_wrist(380, 180)
        time.sleep(0.08)
        arm.set_wrist(620, 180)
        time.sleep(0.08)
        arm.set_wrist(500, 180)
        time.sleep(0.10)


def chaos_attack(arm, attacker, target):
    """
    Pick attacker, dance, then drop it onto the target block.
    """
    arm.set_led(attacker.color)
    arm.beep(0.1)

    ok = arm.pick(attacker.world_x, attacker.world_y, attacker.rotation_deg)
    if not ok:
        arm.home()
        return

    dance_with_block(arm, attacker.world_x, attacker.world_y)

    # Move above target and "drop attack"
    arm.move_xyz(target.world_x, target.world_y, 10.0)
    time.sleep(0.2)
    arm.move_xyz(target.world_x, target.world_y, 4.0, t_ms=300)
    time.sleep(0.1)
    arm.open_gripper()
    time.sleep(0.3)
    arm.move_xyz(target.world_x, target.world_y, arm.cfg.z_lift, t_ms=500)
    arm.home()


def chaos_yeet(arm, attacker):
    """
    Pick attacker, dance, then move to a random off-board zone and release from high up.
    """
    arm.set_led(attacker.color)
    arm.beep(0.1)

    ok = arm.pick(attacker.world_x, attacker.world_y, attacker.rotation_deg)
    if not ok:
        arm.home()
        return

    dance_with_block(arm, attacker.world_x, attacker.world_y)

    yeet_x, yeet_y = random.choice(YEET_ZONES)
    arm.move_xyz(yeet_x, yeet_y, 13.0)
    time.sleep(0.2)
    arm.open_gripper()
    time.sleep(0.3)
    arm.home()


# ----------------------------
# Main application
# ----------------------------
def main():
    # IMPORTANT:
    # Replace gripper_open / gripper_close with your calibrated values from Servo Test.
    cfg = MotionConfig(
        gripper_open=220,      # <-- replace with your actual open pulse
        gripper_close=540,     # <-- replace with your actual close pulse
        z_pick=0.3,            # lower if needed
        z_lift=12.0,
        y_offset=1.0,
    )
    arm = ArmMotion(cfg)
    arm.home()

    cam = Camera.Camera()
    cam.camera_open()
    time.sleep(0.2)

    acquire_sec = 0.9
    min_samples = 4
    samples_by_color = defaultdict(list)
    acquire_t0 = time.time()

    print("Color Chaos running. ESC to quit.")

    try:
        while True:
            frame = cam.frame
            if frame is None:
                continue

            frame_rs, blocks = detect_blocks(frame)

            # Collect short-term observations for averaging
            for b in blocks:
                samples_by_color[b.color].append(b)

            elapsed = time.time() - acquire_t0

            # Show scanning overlay
            vis = draw_scene(frame_rs, blocks, "Scanning for chaos...", color=(0, 255, 255))
            cv2.imshow("Color Chaos", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

            if elapsed < acquire_sec:
                continue

            stable_blocks = average_detections(samples_by_color, min_samples=min_samples)

            # Reset acquisition window for next cycle
            samples_by_color = defaultdict(list)
            acquire_t0 = time.time()

            if len(stable_blocks) == 0:
                continue

            if len(stable_blocks) == 1:
                attacker = stable_blocks[0]
                show_status(frame_rs, stable_blocks, f"Picking up {attacker.color} block", color=(0, 255, 0), hold_ms=350)
                show_status(frame_rs, stable_blocks, f"Yeeting {attacker.color} block", color=(0, 165, 255), hold_ms=350)
                chaos_yeet(arm, attacker)
                time.sleep(0.6)
                continue

            # 2 or more blocks: choose different attacker and target
            attacker, target = random.sample(stable_blocks, 2)

            show_status(frame_rs, stable_blocks, f"Picking up {attacker.color} block", color=(0, 255, 0), hold_ms=350)
            show_status(frame_rs, stable_blocks, f"Attacking {target.color} block", color=(0, 0, 255), hold_ms=350)
            chaos_attack(arm, attacker, target)
            time.sleep(0.6)

    finally:
        cam.camera_close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()