import time
import cv2

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

class Sensor():
    """
    Reads camera
    """
    def __init__(self, width: int = 320, height: int = 240):
        if Picamera2 is None:
            raise RuntimeError("picamera2 not available. Install python3-picamera2 on the Pi.")
        self.w = width
        self.h = height
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(0.2)

    def read_frame(self):
        # Returns RGB; OpenCV wants BGR
        rgb = self.picam2.capture_array()
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return bgr


    def poll(self):
        while True:
            print(self.read_frame)
            time.sleep(0.05)

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
    def __init__(self, car, gain_deg: float = 25.0, max_deg: float = 35.0):
        self.car = car
        self.gain_deg = float(gain_deg)
        self.max_deg = float(max_deg)

    def steer(self, offset: float) -> float:
        # positive offset means line is left of robot, so steer left (positive angle)
        angle = self.gain_deg * offset
        angle = max(-self.max_deg, min(self.max_deg, angle))

        # SunFounder PiCar-X typically uses set_dir_servo_angle(angle)
        # If your method name differs, fix it here.
        self.car.set_dir_servo_angle(angle)
        return angle

sense = Sensor()
interpret = Interpreter()

while True:
    print(interpret.process(sense.read()))
    time.sleep(0.5)
