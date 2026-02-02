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


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


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
            vals = [a.read() for a in self.adcs]
            print(vals)
            time.sleep(0.05)


class Interpreter():
    """
    Turns 3 sensor readings into an offset in [-1, 1].

    Improvements:
      - Always produces a *continuous* offset (not just -1/0/+1).
      - Returns confidence + mode ("track"/"lost") so controller can handle missing line.
      - Robust to lighting by normalizing to local mean.
    """
    def __init__(self, sensitivity=0.20, polarity="dark", deadband=0.03):
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
        if self.polarity == "dark":
            score = [-d for d in dev]
        else:
            score = [d for d in dev]

        # Use only positive evidence as "line"
        w = [max(0.0, s) for s in score]
        ssum = w[0] + w[1] + w[2]

        mag = max(abs(score[0]), abs(score[1]), abs(score[2]), 1.0)
        conf = clamp(ssum / (mag + 1e-9), 0.0, 1.0)

        # If contrast is too weak, declare lost
        if conf < self.sensitivity or ssum < 1e-6:
            return 0.0, conf, "lost"

        # Continuous centroid offset: left=+1, mid=0, right=-1
        offset = (w[0] * 1.0 + w[1] * 0.0 + w[2] * -1.0) / (ssum + 1e-9)
        offset = clamp(offset, -1.0, 1.0)

        if abs(offset) < self.deadband:
            offset = 0.0

        return offset, conf, "track"


class Controller():
    """
    Maps offset in [-1, 1] to a steering angle command.

    Improvements:
      - PD steering (damping) to reduce oscillation.
      - Speed scaling also depends on confidence (slow when unsure).
      - Lost-line behavior: stop (or creep if you set lost_speed > 0).
      - Avoids the old bug where min_speed > base_speed.
    """
    def __init__(
        self,
        car,
        kp_deg=18.0,            # proportional gain (deg per offset)
        kd_deg=6.0,             # derivative gain (deg per (offset/sec))
        max_deg=None,
        base_speed=35,          # IMPORTANT: >= 30 with your current picarx_improved MIN_TURN_PWM behavior
        speed_scale=0.5,        # slow down on big offsets or low confidence
        min_speed=30,           # keep >= 30 to avoid MIN_TURN_PWM weirdness (unless you fix car code)
        lost_speed=0,           # 0 = stop when lost, else creep forward
        angle_alpha=0.6         # smoothing factor for steering
    ):
        self.car = car
        self.kp_deg = float(kp_deg)
        self.kd_deg = float(kd_deg)
        self.max_deg = float(max_deg) if max_deg is not None else float(getattr(car, "DIR_MAX", 30))

        self.base_speed = int(base_speed)
        self.speed_scale = float(speed_scale)
        self.min_speed = int(min_speed)
        self.lost_speed = int(lost_speed)
        self.angle_alpha = float(angle_alpha)

        self._prev_offset = 0.0
        self._prev_angle = 0.0
        self._prev_time = time.time()

    def steer_angle(self, offset, dt):
        # PD control
        derr = (float(offset) - self._prev_offset) / max(1e-3, dt)
        angle_cmd = self.kp_deg * float(offset) + self.kd_deg * derr
        angle_cmd = clamp(angle_cmd, -self.max_deg, self.max_deg)

        # smooth steering (reduces twitch)
        angle = (1.0 - self.angle_alpha) * self._prev_angle + self.angle_alpha * angle_cmd

        self.car.set_dir_servo_angle(angle)

        self._prev_offset = float(offset)
        self._prev_angle = float(angle)
        return angle

    def speed_cmd(self, offset, conf, mode):
        if mode != "track":
            return 0 if self.lost_speed <= 0 else self.lost_speed

        # scale speed down if far off center OR low confidence
        off_term = min(1.0, abs(float(offset)))
        conf_term = 1.0 - clamp(float(conf), 0.0, 1.0)
        scale = 1.0 - self.speed_scale * max(off_term, conf_term)

        s = int(self.base_speed * scale)
        # IMPORTANT: keep >= min_speed (default 30) with current car behavior
        return clamp(s, self.min_speed, 100)


def line_follow_loop(car, sensor, interpreter, controller, dt=0.05):
    try:
        while True:
            t0 = time.time()

            readings = sensor.read()
            offset, conf, mode = interpreter.process(readings)

            # steering first (updates car.dir_current_angle)
            angle = controller.steer_angle(offset, dt)

            # then speed + forward (Picarx.forward uses current steering angle)
            speed = controller.speed_cmd(offset, conf, mode)
            if speed <= 0:
                car.stop()
                car.set_dir_servo_angle(0)
                angle = 0.0
            else:
                car.forward(speed)

            logging.info(
                f"adc={readings}  offset={offset:+.2f}  angle={angle:+.1f}  "
                f"speed={speed:3d}  conf={conf:.2f}  mode={mode}"
            )

            # keep loop timing stable
            elapsed = time.time() - t0
            time.sleep(max(0.0, dt - elapsed))

    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
        car.set_dir_servo_angle(0)


def main():
    car = Picarx()
    sensor = Sensor()

    # If it's steering the wrong way, flip polarity or swap A0/A2.
    interpreter = Interpreter(sensitivity=0.20, polarity="dark", deadband=0.03)

    # Start stable; if it oscillates, drop kp_deg (18 -> 14) or increase kd_deg (6 -> 8).
    controller = Controller(
        car,
        kp_deg=18.0,
        kd_deg=6.0,
        base_speed=35,    # >=30 recommended with current car code
        min_speed=30,
        speed_scale=0.5,
        lost_speed=0
    )

    line_follow_loop(car, sensor, interpreter, controller, dt=0.05)


if __name__ == "__main__":
    main()
