import time
import cv2

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
    Turns 3 sensor readings into an offset in [-1, 1].
      - convert raw readings into a "line score" relative to local mean
      - polarity decides whether low means line (dark) or high means line (light)
      - detect sharp change between adjacent sensors (edge detection)
      - fallback to centroid weighting when edges are weak
    """
    def __init__(self, sensitivity = 0.25, polarity = "dark", deadband = 0.05):
        if polarity not in ("dark", "light"):
            raise ValueError('polarity must be "dark" or "light"')

        self.sensitivity = float(sensitivity)
        self.polarity = polarity
        self.deadband = float(deadband)

    def process(self, readings):
        if len(readings) != 3:
            raise ValueError("readings must be a list of 3 values")

        x0, x1, x2 = [float(v) for v in readings]

        # Local normalization: compare each value to the local mean
        mu = (x0 + x1 + x2) / 3.0
        dev = [x0 - mu, x1 - mu, x2 - mu]

        # Convert to "line score" so higher score means "more likely line"
        # dark line => reading drops => dev negative => score = -dev
        # light line => reading rises => dev positive => score = +dev
        if self.polarity == "dark":
            score = [-d for d in dev]
        else:
            score = [d for d in dev]

        # Scale-free edge test using adjacent differences
        d01 = score[1] - score[0]
        d12 = score[2] - score[1]

        mag = max(abs(score[0]), abs(score[1]), abs(score[2]), 1.0)
        thr = self.sensitivity * mag

        edge01 = abs(d01) > thr
        edge12 = abs(d12) > thr

        # Helper: clamp to [-1, 1]
        def clamp(v):
            return max(-1.0, min(1.0, v))

        # Edge-based logic
        if edge01 or edge12:

            # Determine where the "line-like" region is: which sensor has the highest score
            best_i = max(range(3), key=lambda i: score[i])

            # Position map: left=+1, mid=0, right=-1
            pos_map = {0: 1.0, 1: 0.0, 2: -1.0}
            offset = pos_map[best_i]

            # Severity: if the edge is strong and the best sensor is an outer one, treat as "very off center"
            strength = max(abs(d01), abs(d12)) / (mag + 1e-9)
            confidence = min(1.0, strength)

            # Small deadband to stop twitching when basically centered
            if abs(offset) < self.deadband:
                offset = 0.0

            return offset

        # Fallback: centroid-like estimate (more robust than guessing)
        # Only use positive scores as "line evidence"
        w = [max(0.0, s) for s in score]
        ssum = w[0] + w[1] + w[2]

        if ssum < 1e-6:
            return 0.0

        # Weighted position: left=+1, mid=0, right=-1
        offset = (w[0] * 1.0 + w[1] * 0.0 + w[2] * -1.0) / ssum

        if abs(offset) < self.deadband:
            offset = 0.0

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
