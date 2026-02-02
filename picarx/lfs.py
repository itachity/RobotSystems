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
    Reads 3 sensors (with oversampling + EMA to reduce noise).
    """
    def __init__(self, pins=("A0", "A1", "A2"), oversample=3, ema_alpha=0.30):
        self.adcs = [ADC(p) for p in pins]
        self.oversample = int(oversample)
        self.ema_alpha = float(ema_alpha)
        self._ema = None

    def read(self):
        acc = [0.0, 0.0, 0.0]
        for _ in range(self.oversample):
            r = [a.read() for a in self.adcs]
            for i in range(3):
                acc[i] += r[i]
        avg = [v / self.oversample for v in acc]

        if self._ema is None:
            self._ema = avg
        else:
            a = self.ema_alpha
            self._ema = [(1.0 - a) * self._ema[i] + a * avg[i] for i in range(3)]

        return [int(v) for v in self._ema]


class Interpreter():
    """
    Returns (offset, confidence, mode) where:
      offset in [-1, 1] (left positive)
      mode is "track" or "lost"

    Key fix: hysteresis on contrast so we don't "lost" too early.
    """
    def __init__(
        self,
        polarity="dark",
        deadband=0.06,
        contrast_on=90,     # enter track if contrast >= this
        contrast_off=55,    # stay track until contrast < this
        offset_alpha=0.35
    ):
        if polarity not in ("dark", "light"):
            raise ValueError('polarity must be "dark" or "light"')
        if contrast_off >= contrast_on:
            raise ValueError("contrast_off must be < contrast_on")

        self.polarity = polarity
        self.deadband = float(deadband)
        self.contrast_on = float(contrast_on)
        self.contrast_off = float(contrast_off)
        self.offset_alpha = float(offset_alpha)

        self._tracking = False
        self._offset_ema = 0.0

    def process(self, readings):
        x0, x1, x2 = [float(v) for v in readings]
        contrast = max(x0, x1, x2) - min(x0, x1, x2)

        # map contrast to confidence with hysteresis band
        conf = clamp((contrast - self.contrast_off) / (self.contrast_on - self.contrast_off), 0.0, 1.0)

        # hysteresis state machine
        if self._tracking:
            if contrast < self.contrast_off:
                self._tracking = False
        else:
            if contrast >= self.contrast_on:
                self._tracking = True

        if not self._tracking:
            return self._offset_ema, conf, "lost"

        # normalize to local mean (lighting robustness)
        mu = (x0 + x1 + x2) / 3.0
        dev = [x0 - mu, x1 - mu, x2 - mu]

        # score: higher means "more likely line"
        if self.polarity == "dark":
            score = [-d for d in dev]
        else:
            score = [d for d in dev]

        # positive evidence only
        w = [max(0.0, s) for s in score]
        ssum = w[0] + w[1] + w[2]
        if ssum < 1e-6:
            self._tracking = False
            return self._offset_ema, conf, "lost"

        offset = (w[0] * 1.0 + w[1] * 0.0 + w[2] * -1.0) / (ssum + 1e-9)
        offset = clamp(offset, -1.0, 1.0)

        # deadband then smooth
        if abs(offset) < self.deadband:
            offset = 0.0

        a = self.offset_alpha
        self._offset_ema = (1.0 - a) * self._offset_ema + a * offset

        if abs(self._offset_ema) < self.deadband:
            self._offset_ema = 0.0

        return self._offset_ema, conf, "track"


class Controller():
    """
    Adds lost recovery:
      - when lost: creep forward + sweep steering until track returns
    Also fixes twitch:
      - slew-rate limit + angle smoothing
    """
    def __init__(
        self,
        car,
        kp_deg=16.0,
        max_deg=None,
        base_speed=35,
        min_speed=30,
        speed_scale=0.50,
        search_speed=32,            # creep speed during search
        search_max_deg=22.0,        # sweep amplitude
        search_period_s=1.2,        # sweep speed
        hold_last_s=0.35,           # first hold steering toward last known side
        max_slew_deg_per_s=120.0,
        angle_alpha=0.35
    ):
        self.car = car
        self.kp_deg = float(kp_deg)
        self.max_deg = float(max_deg) if max_deg is not None else float(getattr(car, "DIR_MAX", 30))

        self.base_speed = int(base_speed)
        self.min_speed = int(min_speed)
        self.speed_scale = float(speed_scale)

        self.search_speed = int(search_speed)
        self.search_max_deg = float(search_max_deg)
        self.search_period_s = float(search_period_s)
        self.hold_last_s = float(hold_last_s)

        self.max_slew = float(max_slew_deg_per_s)
        self.angle_alpha = float(angle_alpha)

        self._prev_angle = 0.0
        self._last_offset = 0.0
        self._lost_since = None

    def _slew_and_smooth(self, target, dt):
        target = clamp(target, -self.max_deg, self.max_deg)

        max_delta = self.max_slew * max(1e-3, dt)
        delta = clamp(target - self._prev_angle, -max_delta, max_delta)
        limited = self._prev_angle + delta

        a = self.angle_alpha
        angle = (1.0 - a) * self._prev_angle + a * limited

        self._prev_angle = angle
        self.car.set_dir_servo_angle(angle)
        return angle

    def steer_angle(self, offset, conf, mode, dt):
        if mode == "track":
            self._last_offset = float(offset)
            self._lost_since = None
            target = self.kp_deg * float(offset)
            return self._slew_and_smooth(target, dt)

        # LOST: search mode (this is what gets you back to track)
        now = time.time()
        if self._lost_since is None:
            self._lost_since = now

        t = now - self._lost_since

        # First, bias toward last known side for a short time
        sign = 1.0 if self._last_offset > 0 else (-1.0 if self._last_offset < 0 else 0.0)
        if t < self.hold_last_s and sign != 0.0:
            target = sign * (0.6 * self.search_max_deg)
        else:
            # triangle wave sweep [-A, +A]
            A = self.search_max_deg
            P = max(0.4, self.search_period_s)
            phase = (t % P) / P  # 0..1
            tri = (4.0 * phase - 1.0) if phase < 0.5 else (-4.0 * phase + 3.0)  # -1..+1
            target = A * tri + sign * 3.0  # tiny bias toward last side

        return self._slew_and_smooth(target, dt)

    def speed_cmd(self, offset, conf, mode):
        if mode != "track":
            return self.search_speed

        off_term = min(1.0, abs(float(offset)))
        conf_term = 1.0 - clamp(float(conf), 0.0, 1.0)
        scale = 1.0 - self.speed_scale * max(off_term, conf_term)

        s = int(self.base_speed * scale)
        return clamp(s, self.min_speed, 100)


def line_follow_loop(car, sensor, interpreter, controller, dt=0.08):
    try:
        while True:
            t0 = time.time()

            readings = sensor.read()
            offset, conf, mode = interpreter.process(readings)

            angle = controller.steer_angle(offset, conf, mode, dt)
            speed = controller.speed_cmd(offset, conf, mode)

            car.forward(speed)

            logging.info(
                f"adc={readings}  offset={offset:+.2f}  angle={angle:+.1f}  "
                f"speed={speed:3d}  conf={conf:.2f}  mode={mode}"
            )

            elapsed = time.time() - t0
            time.sleep(max(0.0, dt - elapsed))

    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
        car.set_dir_servo_angle(0)


def main():
    car = Picarx()
    sensor = Sensor(oversample=3, ema_alpha=0.30)

    # If it still flips to lost too easily, lower contrast_on/off (ex: 80/45).
    interpreter = Interpreter(polarity="dark", deadband=0.06, contrast_on=90, contrast_off=55, offset_alpha=0.35)

    controller = Controller(
        car,
        kp_deg=16.0,
        base_speed=18,
        min_speed=18,
        speed_scale=0.50,
        search_speed=32,
        search_max_deg=22.0,
        search_period_s=1.2,
        hold_last_s=0.35,
        max_slew_deg_per_s=120.0,
        angle_alpha=0.35
    )

    line_follow_loop(car, sensor, interpreter, controller, dt=0.08)


if __name__ == "__main__":
    main()
