#!/usr/bin/python3
# coding=utf-8

import sys
import cv2
import time

sys.path.append('/home/pi/ArmPi/')
import Camera

from perception import ColorBlockPerception
from motion import ArmMotion, MotionConfig


def main():
    # Perception (already working)
    perceiver = ColorBlockPerception(
        target_colors=("red", "green", "blue"),
        stable_dist_thresh=0.5,
        stable_time_sec=1.0,
        vote_len=3,
        roi_mask_mode="after_detect",
        ready_frames_required=5,
        ready_hold_frames=15,
    )

    # Motion (IMPORTANT: set your real gripper pulses from Servo Test)
    cfg = MotionConfig(
        gripper_open=220,    # <-- replace
        gripper_close=540,   # <-- replace
        y_offset=0.0,        # set to -2.0 if your camera geometry needs it
    )
    arm = ArmMotion(cfg)
    arm.home()

    cam = Camera.Camera()
    cam.camera_open()
    time.sleep(0.2)

    print("Pick-place demo running. ESC to quit.")
    try:
        while True:
            frame = cam.frame
            if frame is None:
                continue

            frame_rs = cv2.resize(frame, perceiver.size, interpolation=cv2.INTER_NEAREST)
            det = perceiver.process_frame(frame_rs, picking_phase=False)
            vis = perceiver.annotate(frame_rs, det)

            # When ready, perform a single pick and place into the RED bin (basic pick-place)
            if det is not None and det.ready and det.voted_color is not None:
                cv2.putText(vis, "EXECUTING PICK", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.imshow("PickPlace", vis)
                cv2.waitKey(1)

                arm.set_led(det.voted_color)
                arm.beep(0.1)

                ok = arm.pick(det.world_x, det.world_y, det.rotation_deg)
                if ok:
                    # basic: drop into matching bin (sorting behavior)
                    arm.sort_place(det.voted_color)
                arm.home()
                perceiver.reset_state_no_detection()  # avoid immediate retrigger on same scene

            cv2.imshow("PickPlace", vis)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

    finally:
        cam.camera_close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()