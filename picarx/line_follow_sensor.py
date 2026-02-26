import time
from picarx_improved import Picarx
import logging

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


class Sensor():
    """
    Reads 3 sensors
    """
    def __init__(self):
        self.adcs = [ADC(p) for p in ["A0", "A1", "A2"]]

    def read(self):
        return [a.read() for a in self.adcs]

    def poll(self):
        while True:
            vals = self.read()
            print(vals)
            time.sleep(0.05)


class Interpreter():
    """
    Turns 3 sensor readings into an offset in [-1, 1].
      - Robust to lighting: uses local normalization (mean) + scale-free thresholds
      - Polarity: "dark" or "light"
      - Edge detection between adjacent sensors to infer which side the line is on
      - Returns magnitude for slight vs very off-center
        (small |offset| means slight, near 1 means very off-center)
      - Positive offset means the line is LEFT of the robot
    """
    def __init__(self, sensitivity=0.25, polarity="dark", deadband=0.05):

        if polarity not in ("dark", "light"):
            raise ValueError('polarity must be "dark" or "light"')

        self.sensitivity = float(sensitivity)
        self.polarity = polarity
        self.deadband = float(deadband)

        # Extra robustness knobs (still same class)
        self.noise_floor_frac = 0.05   # ignore tiny score changes
        self.snap_conf = 0.90          # snap to +-1 when very confident
        self.snap_mag = 0.60           # only snap when already far from center

    def process(self, readings):

        if len(readings) != 3:
            raise ValueError("readings must be a list of 3 values")

        x0, x1, x2 = [float(v) for v in readings]

        # Local normalization (robust to global lighting)
        mu = (x0 + x1 + x2) / 3.0
        dev = [x0 - mu, x1 - mu, x2 - mu]

        # Convert to "line-likelihood" score: higher score => more likely on the line
        # dark line => sensor value drops => dev negative => score = -dev
        if self.polarity == "dark":
            score = [-d for d in dev]
        else:
            score = [d for d in dev]

        # Make scores non-negative and scale-free
        smin = min(score)
        score = [s - smin for s in score]  # shift so min is 0

        mag = max(score[0], score[1], score[2], 1.0)
        thr = self.sensitivity * mag

        # Edge detection between adjacent sensors (left-mid, mid-right)
        e01 = abs(score[0] - score[1])
        e12 = abs(score[1] - score[2])
        edge01 = e01 > thr
        edge12 = e12 > thr

        # Weights for centroid estimate (continuous offset)
        # Ignore tiny noise by subtracting a small floor
        noise_floor = self.noise_floor_frac * mag
        w = [max(0.0, s - noise_floor) for s in score]
        wsum = w[0] + w[1] + w[2]

        # If no strong "line evidence", return centered (or you could keep last offset)
        if wsum < 1e-6:
            return 0.0

        # Weighted position: left=+1, mid=0, right=-1
        offset = (w[0] * 1.0 + w[1] * 0.0 + w[2] * -1.0) / wsum

        # Determine sign + severity using edge location + strength
        # edge_strength is scale-free: 0..~1+
        edge_strength = max(e01, e12) / (mag + 1e-9)
        # normalize confidence against the chosen sensitivity
        confidence = min(1.0, edge_strength / (self.sensitivity + 1e-9))

        # If we have a strong single edge, we can bias the magnitude slightly
        # to represent “very off-center” vs “slightly off-center”
        if (edge01 or edge12) and confidence > self.snap_conf and abs(offset) > self.snap_mag:
            offset = 1.0 if offset > 0 else -1.0

        # Deadband to prevent twitching near center
        if abs(offset) < self.deadband:
            offset = 0.0

        # Clamp to [-1, 1]
        if offset > 1.0:
            offset = 1.0
        elif offset < -1.0:
            offset = -1.0

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
            car.forward(speed)

            logging.info(
                f"adc={readings}  offset={offset:+.2f}  angle={angle:+.1f} speed={speed:3d} "
            )
            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
        car.set_dir_servo_angle(0)


def main():

    car = Picarx()
    sensor = Sensor()
    interpreter = Interpreter()
    controller = Controller(car)
    line_follow_loop(car, sensor, interpreter, controller)


if __name__ == "__main__":
    main()
