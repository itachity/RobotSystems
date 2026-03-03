#!/usr/bin/python3
# coding=utf-8

import sys
import cv2
import time

sys.path.append('/home/pi/ArmPi/')
import Camera

from perception import ColorBlockPerception


def main():
    perceiver = ColorBlockPerception(
        target_colors=("red", "green", "blue"),
        stable_dist_thresh=0.5,
        stable_time_sec=1.0,
        vote_len=3,
        roi_mask_mode="after_detect",
        ready_frames_required=5,
        ready_hold_frames=15,
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

            # IMPORTANT: keep perception + drawing in the same coordinate frame
            frame_rs = cv2.resize(frame, perceiver.size, interpolation=cv2.INTER_NEAREST)

            det = perceiver.process_frame(frame_rs, picking_phase=False)
            vis = perceiver.annotate(frame_rs, det)

            if det is not None and det.ready:
                cv2.putText(vis, "READY TO PICK", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                print(f"READY: color={det.voted_color} x={det.world_x:.2f} y={det.world_y:.2f} rot={det.rotation_deg:.1f}")

            cv2.imshow("Perception Demo", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

    finally:
        cam.camera_close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()