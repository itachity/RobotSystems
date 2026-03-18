#!/usr/bin/python3
# coding=utf-8

import time
from dataclasses import dataclass
from typing import Dict, Tuple, Optional

import sys
sys.path.append('/home/pi/ArmPi/')  # vendor libs

from ArmIK.ArmMoveIK import ArmIK
from ArmIK.Transform import getAngle
import HiwonderSDK.Board as Board


@dataclass
class MotionConfig:
    # --- Gripper calibration (set these from Servo Test!) ---
    gripper_open: int = 220     # example: replace with your measured open pulse
    gripper_close: int = 540    # example: replace with your measured close pulse

    # --- Wrist neutral ---
    wrist_neutral: int = 500

    # --- Z heights (cm, in ArmIK world frame) ---
    z_approach: float = 7.0
    z_pick: float = 0.8
    z_lift: float = 12.0

    # --- Small offsets for camera/gripper geometry ---
    x_offset: float = 0.0
    y_offset: float = 0.0  # vendor sometimes uses -2.0

    # --- Place bins (ColorSorting default) ---
    bins: Dict[str, Tuple[float, float, float]] = None

    # --- Palletizing/stacking ---
    stack_base: Tuple[float, float, float] = (-14.0, -7.5, 1.5)
    stack_dz: float = 2.5
    stack_max_layers: int = 3


class ArmMotion:
    def __init__(self, cfg: MotionConfig):
        self.cfg = cfg
        self.ak = ArmIK()

        if self.cfg.bins is None:
            self.cfg.bins = {
                "red":   (-14.5, 11.5, 1.5),
                "green": (-14.5,  5.5, 1.5),
                "blue":  (-14.5, -0.5, 1.5),
            }

        # stack state per color (more general than vendor’s z_r-only)
        self.stack_level = {"red": 0, "green": 0, "blue": 0}

    # ---------- low-level primitives ----------

    def beep(self, sec: float = 0.1):
        Board.setBuzzer(1)
        time.sleep(sec)
        Board.setBuzzer(0)

    def set_led(self, color: str):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()

    def open_gripper(self, ms: int = 500):
        Board.setBusServoPulse(1, self.cfg.gripper_open, ms)

    def close_gripper(self, ms: int = 500):
        Board.setBusServoPulse(1, self.cfg.gripper_close, ms)

    def wrist_neutral(self, ms: int = 500):
        Board.setBusServoPulse(2, self.cfg.wrist_neutral, ms)

    def rotate_wrist_for_block(self, x: float, y: float, rotation_deg: float, ms: int = 500):
        ang = getAngle(x, y, rotation_deg)
        Board.setBusServoPulse(2, ang, ms)

    def move_xyz(self, x: float, y: float, z: float, t_ms: Optional[int] = None) -> bool:
        """Move end-effector to world (x,y,z). Returns False if unreachable."""
        x += self.cfg.x_offset
        y += self.cfg.y_offset
        if t_ms is None:
            result = self.ak.setPitchRangeMoving((x, y, z), -90, -90, 0)
            if result is False:
                return False
            time.sleep(result[2] / 1000)
            return True
        else:
            result = self.ak.setPitchRangeMoving((x, y, z), -90, -90, 0, t_ms)
            return (result is not False)

    # ---------- high-level tasks ----------

    def home(self):
        # Start in a safe pose, gripper slightly open, wrist neutral
        self.open_gripper(ms=300)
        self.wrist_neutral(ms=500)
        self.ak.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)

    def pick(self, x: float, y: float, rotation_deg: float) -> bool:
        """Pick block at (x,y) with estimated rotation."""
        # Approach
        if not self.move_xyz(x, y, self.cfg.z_approach):
            return False

        # Prep end-effector
        self.open_gripper()
        self.rotate_wrist_for_block(x + self.cfg.x_offset, y + self.cfg.y_offset, rotation_deg)
        time.sleep(0.6)

        # Descend + grip
        if not self.move_xyz(x, y, self.cfg.z_pick, t_ms=1000):
            return False
        time.sleep(0.4)
        self.close_gripper()
        time.sleep(0.6)

        # Lift
        self.wrist_neutral()
        self.move_xyz(x, y, self.cfg.z_lift, t_ms=1000)
        time.sleep(0.4)
        return True

    def place_at(self, x: float, y: float, z: float) -> bool:
        """Place currently held block at (x,y,z)."""
        # Move above
        if not self.move_xyz(x, y, self.cfg.z_lift):
            return False

        # Rotate wrist for placement (vendor uses -90)
        self.rotate_wrist_for_block(x, y, -90)
        time.sleep(0.4)

        # Descend and release
        self.move_xyz(x, y, z + 3.0, t_ms=500)
        self.move_xyz(x, y, z, t_ms=1000)
        time.sleep(0.3)
        self.open_gripper()
        time.sleep(0.5)

        # Retract
        self.move_xyz(x, y, self.cfg.z_lift, t_ms=800)
        time.sleep(0.3)
        return True

    def sort_place(self, color: str) -> bool:
        """Place into the appropriate bin based on color."""
        x, y, z = self.cfg.bins[color]
        return self.place_at(x, y, z)

    def next_stack_z(self, color: str) -> float:
        """Compute next Z for stacking (palletizing)."""
        base_z = self.cfg.stack_base[2]
        level = self.stack_level.get(color, 0)
        z = base_z + level * self.cfg.stack_dz
        # cycle stack height like vendor (3 layers)
        level = (level + 1) % self.cfg.stack_max_layers
        self.stack_level[color] = level
        return z

    def palletize_place(self, color: str) -> bool:
        """Place all blocks into a single stacking area with incrementing Z."""
        x, y, _ = self.cfg.stack_base
        z = self.next_stack_z(color)
        return self.place_at(x, y, z)