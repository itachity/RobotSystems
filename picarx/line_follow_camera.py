import time
import cv2
import logging
from picarx_improved import Picarx

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

from vilib import Vilib

import time
import cv2
from vilib import Vilib

class Sensor():
    """
    Uses Vilib as the camera owner (web stream),
    and reads frames from the MJPG stream via OpenCV.
    """
    def __init__(self, url="http://127.0.0.1:9000/mjpg", warmup_s=0.5,
                 vflip=False, hflip=False, local=False, web=True):

        # Start Vilib streaming (this is the only thing that touches the camera)
        Vilib.camera_start(vflip=vflip, hflip=hflip)
        Vilib.display(local=local, web=web)

        time.sleep(warmup_s)

        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open MJPG stream at {url}")

    def read(self):
        # Grab a fresh frame
        ret, frame = self.cap.read()
        if not ret or frame is None:
            # quick reconnect attempt
            self.cap.release()
            time.sleep(0.1)
            self.cap = cv2.VideoCapture("http://127.0.0.1:9000/mjpg")
            ret, frame = self.cap.read()
            if not ret or frame is None:
                raise RuntimeError("Failed to read frame from MJPG stream")
        return frame

    def poll(self):
        while True:
            print(self.read)
            time.sleep(0.05)

    def close(self):
        try:
            self.cap.release()
        except:
            pass


    

class Interpreter():
    """
    Turns camera readings into an offset in [-1, 1].
    """
    def __init__(self, polarity = "dark"):
        if polarity not in ("dark", "light"):
            raise ValueError('polarity must be "dark" or "light"')
        self.polarity = polarity

    def process(self, frame_bgr):
        h, w = frame_bgr.shape[:2]

        # Use bottom part of image where line is near the wheels
        roi = frame_bgr[int(h * 0.60):h, 0:w]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Threshold
        if self.polarity == "dark":
            # line is dark => invert threshold so line becomes white in mask
            _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        else:
            _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Clean noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (3, 3), iterations=1)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return 0.0

        # Largest contour = most likely the line
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 50:
            return 0.0, 0.0

        M = cv2.moments(c)
        if M["m00"] == 0:
            return 0.0, 0.0

        cx = int(M["m10"] / M["m00"])  # x location in ROI
        # Convert cx to offset: left positive, right negative
        offset = (w / 2.0 - cx) / (w / 2.0)

        # clamp
        offset = max(-1.0, min(1.0, float(offset)))
        return offset

class Controller():
    """
    Maps offset in [-1, 1] to a steering angle command.
    """
    def __init__(self, car, gain_deg=22.0, max_deg=None, speed_scale=0.6, min_speed=25):

        self.car = car
        self.gain_deg = float(gain_deg)
        self.max_deg = float(max_deg) if max_deg is not None else float(getattr(car, "DIR_MAX", 30))
        self.speed_scale = float(speed_scale)
        self.min_speed = int(min_speed)

    def steer_angle(self, offset):
        # positive offset => line left => steer LEFT
        angle = -self.gain_deg * float(offset)
        angle = max(-self.max_deg, min(self.max_deg, angle))
        self.car.set_dir_servo_angle(angle)
        return angle

    def speed_cmd(self, base_speed: int, offset: float):
        # Reduce speed as |offset| grows. Helps prevent oscillation.
        base = int(base_speed)
        k = self.speed_scale
        s = int(base * (1.0 - k * min(1.0, abs(float(offset)))))
        return max(self.min_speed, min(100, s))

def line_follow_loop(car, sensor, interpreter, controller, base_speed = 25, dt = 0.05):
    try:
        while True:
            readings = sensor.read()
            offset = interpreter.process(readings)
            angle = controller.steer_angle(offset)
            speed = controller.speed_cmd(base_speed, offset)
            #car.forward(speed)

            logging.info(
                f"offset={offset:+.2f}  angle={angle:+.1f} speed={speed:3d} "
            )

            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
        car.set_dir_servo_angle(0)


def main():
    car = Picarx()

    car.set_cam_pan_angle(0)
    car.set_cam_tilt_angle(-45)
    time.sleep(0.2)

    sensor = Sensor()
    interpreter = Interpreter()
    controller = Controller(car)
    line_follow_loop(car, sensor, interpreter, controller)


if __name__ == "__main__":
    main()
