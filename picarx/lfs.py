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
    Reads 3 sensors (with smoothing to reduce noise).
    """
    def __init__(self, pins=("A0", "A1", "A2"), oversample=3, ema_alpha=0.35):
        self.adcs = [ADC(p) for p in pins]
        self.oversample = int(oversample)
        self.ema_alpha = float(ema_alpha)
        self._ema = None

    def read(self):
        # oversample then average
        acc = [0.0, 0.0, 0.0]
        for _ in range(self.oversample):
            r = [a.read() for a in self.adcs]
            acc[0] += r[0]
            acc[1] += r[1]
            acc[2] += r[2]
        avg = [v / self.oversample for v in acc]

        # EMA filter
        if self._ema is None:
            self._ema = avg
        else:
            a = self.ema_alpha
            self._ema = [(1.0 - a) * self._ema[i] + a * avg[i] for i in range(3)]

        return [int(v) for v in self._ema]

    def poll(self):
        while True:
            vals = self.read()
            contrast = max(vals) - min(vals)
            logging.info(f"adc={vals}  contrast={contrast}")
            time.sleep(0.10)


class Interpreter():
    """
    Turns 3 sensor readings into an offset in [-1, 1].

    Improvements:
      - Uses a real contrast threshold (max-min) to decide track vs lost.
      - Always returns a continuous offset (no bang-bang -1/0/+1).
      - Low-pass filters offset to reduce twitch.
    """
    def __init__(
        self,
        polarity="dark",
        deadband=0.06,
        contrast_thresh=120,     # ADC counts; tune this
        offset_alpha=0.35        # smoothing of offset
    ):
        if polarity not in ("dark", "light"):
            raise ValueError('polarity must be "dark" or "light"')

        self.polarity = polarity
        self.deadband = float(deadband)
        self.contrast_thresh = float(contrast_thresh)
        self.offset_alpha = float(offset_alpha)

        self._offset_ema = 0.0

    def process(self, readings):
        if len(readings) != 3:
            raise ValueError("readings must be a list of 3 values")

        x0, x1, x2 = [float(v) for v in readings]
        contrast = max(x0, x1, x2) - min(x0, x1, x2)

        # Confidence based on contrast only (simple, reliable)
        conf = clamp(contrast / max(1.0, self.contrast_thresh), 0.0, 1.0)

        # If contrast is too low, call it lost (but keep last filtered offset)
        if contrast < self.contrast_thresh:
            return self._offset_ema, conf, "lost"

        # Normalize to local mean (lighting robustness)
        mu = (x0 + x1 + x2) / 3.0
        dev = [x0 - mu, x1 - mu, x2 - mu]

        # Score: higher means "more likely line"
        if self.polarity == "dark":
            score = [-d for d in dev]
        else:
            score = [d for d in dev]

        # Use only positive evidence
        w = [max(0.0, s) for s in score]
        ssum = w[0] + w[1] + w[2]

        if ssum < 1e-6:
            return self._offset_ema, conf, "lost"

        # Continuous centroid (left=+1, mid=0, right=-1)
        offset = (w[0] * 1.0 + w[1] * 0.0 + w[2] * -1.0) / (ssum + 1e-9)
        offset = clamp(offset, -1.0, 1.0)

        # Deadband
        if abs(offset) < self.deadband:
            offset = 0.0

        # Smooth offset to reduce jitter/twitch
        a = self.offset_alpha
        self._offset_ema = (1.0 - a) * self._offset_ema + a * offset

        # Deadband again after smoothing
        if abs(self._offset_ema) < self.deadband:
            self._offset_ema = 0.0

        return self._offset_ema, conf, "track"


class Controller():
    """
    Maps offset in [-1, 1] to steering + speed.

    Improvements:
      - Removes fast twitch by limiting steering rate (slew limit).
      - Adds mild smoothing of angle.
      - Stops cleanly when lost (or creep if you set lost_speed > 0).
      - Avoids min_speed > base_speed bug.
    """
    def __init__(
        self,
        car,
        kp_deg=16.0,
        max_deg=None,
        base_speed=30,            # keep >= 30 with your current picarx_improved MIN_TURN_PWM
        speed_scale=0.60,
        min_speed=30,
        lost_speed=0,             # 0 = stop when lost, else creep
        max_slew_deg_per_s=120.0, # rate limit steering changes
        angle_alpha=0.35          # smooth the commanded angle
    ):
        self.car = car
        self.kp_deg = float(kp_deg)
        self.max_deg = float(max_deg) if max_deg is not None else float(getattr(car, "DIR_MAX", 30))

        self.base_speed = int(base_speed)
        self.speed_scale = float(speed_scale)
        self.min_speed = int(min_speed)
        self.lost_speed = int(lost_speed)

        self.max_slew = float(max_slew_deg_per_s)
        self.angle_alpha = float(angle_alpha)

        self._prev_angle = 0.0
        self._lost_count = 0

    def steer_angle(self, offset, dt, mode):
        # If lost, do NOT thrash steering
        if mode != "track":
            self._lost_count += 1
            # hold last angle for a moment, then gently return to center
            if self._lost_count >= 6:
                target = 0.0
            else:
                target = self._prev_angle
        else:
            self._lost_count = 0
            target = self.kp_deg * float(offset)

        target = clamp(target, -self.max_deg, self.max_deg)

        # Slew-rate limit: cap how fast angle can change
        max_delta = self.max_slew * max(1e-3, dt)
        delta = clamp(target - self._prev_angle, -max_delta, max_delta)
        limited = self._prev_angle + delta

        # Smooth angle
        a = self.angle_alpha
        angle = (1.0 - a) * self._prev_angle + a * limited

        self.car.set_dir_servo_angle(angle)
        self._prev_angle = angle
        return angle

    def speed_cmd(self, base_speed, offset, conf, mode):
        if mode != "track":
            return 0 if self.lost_speed <= 0 else self.lost_speed

        base = int(base_speed)

        # slow down if far off center or low confidence
        off_term = min(1.0, abs(float(offset)))
        conf_term = 1.0 - clamp(float(conf), 0.0, 1.0)
        scale = 1.0 - self.speed_scale * max(off_term, conf_term)

        s = int(base * scale)
        return clamp(s, self.min_speed, 100)


def line_follow_loop(car, sensor, interpreter, controller, base_speed=30, dt=0.08):
    try:
        while True:
            t0 = time.time()

            readings = sensor.read()
            offset, conf, mode = interpreter.process(readings)

            angle = controller.steer_angle(offset, dt, mode)
            speed = controller.speed_cmd(base_speed, offset, conf, mode)

            if speed <= 0:
                car.stop()
            else:
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

    # Strongly reduces noise compared to raw reads
    sensor = Sensor(oversample=3, ema_alpha=0.35)

    # If "lost" too often, reduce contrast_thresh (ex: 120 -> 80).
    # If twitchy, increase deadband (0.06 -> 0.08).
    interpreter = Interpreter(polarity="dark", deadband=0.06, contrast_thresh=120, offset_alpha=0.35)

    # Twitch fix: kp lower, dt slower, slew limit on, angle smoothing.
    controller = Controller(
        car,
        kp_deg=16.0,
        base_speed=22,
        min_speed=22,
        speed_scale=0.60,
        lost_speed=0,
        max_slew_deg_per_s=120.0,
        angle_alpha=0.35
    )

    line_follow_loop(car, sensor, interpreter, controller, base_speed=30, dt=0.08)


if __name__ == "__main__":
    main()
