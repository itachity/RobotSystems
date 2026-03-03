#!/usr/bin/python3
# coding=utf-8

import sys
import cv2
import time

sys.path.append('/home/pi/ArmPi/')
import Camera

# Your refactored perception module (you said you named it perception.py)
# Assumes class name = ColorBlockPerception and methods = process_frame(), annotate()
from perception import ColorBlockPerception


def main():
    # Tune these to match ColorSorting defaults
    perceiver = ColorBlockPerception(
        target_colors=("red", "green", "blue"),
        stable_dist_thresh=0.5,
        stable_time_sec=1.0,
        vote_len=3,
        roi_mask_mode="after_detect",
    )

    cam = Camera.Camera()
    cam.camera_open()
    time.sleep(0.2)

    print("Demo running. ESC to quit.")
    try:
        while True:
            frame = cam.frame
            if frame is None:
                continue

            # Run perception (no motion)
            det = perceiver.process_frame(frame, picking_phase=False)

            # Define "ready" like your flowchart: stable + voted color available
            ready = (det is not None and det.stable and det.voted_color is not None)

            # Annotate and display
            vis = perceiver.annotate(frame, det)
            if ready:
                cv2.putText(
                    vis,
                    "READY TO PICK",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2,
                )
                # Print only when ready (optional)
                print(
                    f"READY: color={det.voted_color} "
                    f"x={det.world_x:.2f} y={det.world_y:.2f} rot={det.rotation_deg:.1f}"
                )

            cv2.imshow("Perception Demo", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break

    finally:
        cam.camera_close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()