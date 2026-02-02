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
    - On robot: uses Picarx grayscale module (recommended, matches SunFounder)
    - On desktop: falls back to ADC reads (sim)
    """
    def __init__(self, car: Picarx = None):
        self.car = car
        self.adcs = [ADC(p) for p in ["A0", "A1", "A2"]]  # fallback only

    def read(self):
        if self.car is not None and hasattr(self.car, "get_grayscale_data"):
            return self.car.get_grayscale_data()
        return [a.read() for a in self.adcs]

    def poll(self):
        while True:
            vals = self.read()
            print(vals)
            time.sleep(0.05)


class Interpreter():
    """
    SunFounder-style 4-state interpreter based on get_line_status():
      _state = [b0, b1, b2] where 0 means line, 1 means background

    Mapping copied from the SunFounder sample (as in the transcript):
      if _state == [0,0,0] -> 'stop'
      elif _state[1] == 1  -> 'forward'
      elif _state[0] == 1  -> 'right'
      elif _state[2] == 1  -> 'left'
    """
    def __init__(self, car: Picarx, offset_deg: float = 20.0, px_power: int = 10):
        self.car = car
        self.offset_deg = float(offset_deg)
        self.px_power = int(px_power)

        self.state = "stop"
        self.last_state = "stop"

    def get_status(self, val_list):
        _state = self.car.get_line_status(val_list)  # [bool/binary, bool/binary, bool/binary]
        # Normalize in case it returns True/False
        _state = [1 if bool(x) else 0 for x in _state]

        if _state == [0, 0, 0]:
            return "stop"
        elif _state[1] == 1:
            return "forward"
        elif _state[0] == 1:
            return "right"
        elif _state[2] == 1:
            return "left"
        return "stop"

    def outHandle(self, sensor: Sensor, timeout_s: float = 1.5):
        """
        SunFounder recovery logic:
          - if last_state == 'left': steer -DIR_MAX, reverse slowly
          - if last_state == 'right': steer +DIR_MAX, reverse slowly
          - keep reversing until state changes away from last_state
        """
        if self.last_state == "left":
            self.car.set_dir_servo_angle(-float(getattr(self.car, "DIR_MAX", 30)))
            self.car.backward(self.px_power)
        elif self.last_state == "right":
            self.car.set_dir_servo_angle(+float(getattr(self.car, "DIR_MAX", 30)))
            self.car.backward(self.px_power)
        else:
            self.car.set_dir_servo_angle(0)
            self.car.backward(self.px_power)

        t0 = time.time()
        while True:
            gm_val_list = sensor.read()
            gm_state = self.get_status(gm_val_list)

            logging.debug(f"outHandle adc={gm_val_list} state={gm_state} last={self.last_state}")

            # break when it changes (SunFounder code logic)
            if gm_state != self.last_state:
                break

            # safety so you don't get stuck forever
            if (time.time() - t0) > timeout_s:
                logging.debug("outHandle timeout, breaking to avoid infinite loop")
                break

            time.sleep(0.01)

        self.car.stop()
        time.sleep(0.01)

    def process(self, readings):
        """
        Returns offset in [-1, 1] but is actually discrete:
          left  -> +1
          right -> -1
          forward/stop -> 0
        Also updates self.state and self.last_state.
        """
        self.state = self.get_status(readings)

        if self.state != "stop":
            self.last_state = self.state

        if self.state == "left":
            return +1.0
        elif self.state == "right":
            return -1.0
        else:
            return 0.0


class Controller():
    """
    Maps offset in [-1, 1] to a steering angle command.
    For SunFounder discrete tracking:
      offset is in {-1,0,+1} and gain_deg should be 'offset_deg' (e.g., 20).
    """
    def __init__(self, car, gain_deg = 20.0, max_deg = None, speed_scale = 0.0, min_speed = 25):
        self.car = car
        self.gain_deg = float(gain_deg)
        self.max_deg = float(max_deg) if max_deg is not None else float(getattr(car, "DIR_MAX", 30))
        self.speed_scale = float(speed_scale)
        self.min_speed = int(min_speed)

    def steer_angle(self, offset):
        angle = self.gain_deg * float(offset)
        angle = max(-self.max_deg, min(self.max_deg, angle))
        self.car.set_dir_servo_angle(angle)
        return angle

    def speed_cmd(self, base_speed: int, offset: float):
        # SunFounder behavior is basically constant power; keep speed_scale at 0.0
        base = int(base_speed)
        k = self.speed_scale
        s = int(base * (1.0 - k * min(1.0, abs(float(offset)))))
        return max(self.min_speed, min(100, s))


def line_follow_loop(car, sensor, interpreter, controller, base_speed = 25, dt = 0.05):
    try:
        while True:
            readings = sensor.read()
            offset = interpreter.process(readings)

            # SunFounder: if stop -> recovery behavior
            if interpreter.state == "stop":
                logging.info(f"adc={readings} state=stop last={interpreter.last_state} -> outHandle()")
                interpreter.outHandle(sensor)
                time.sleep(dt)
                continue

            angle = controller.steer_angle(offset)
            speed = controller.speed_cmd(base_speed, offset)

            car.forward(speed)

            logging.info(
                f"adc={readings} state={interpreter.state:7s} offset={offset:+.0f} angle={angle:+.1f} speed={speed:3d}"
            )
            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
        car.set_dir_servo_angle(0)


def main():
    car = Picarx()
    sensor = Sensor(car)
    interpreter = Interpreter(car, offset_deg=20.0, px_power=10)
    controller = Controller(car, gain_deg=interpreter.offset_deg, speed_scale=0.0, min_speed=interpreter.px_power)

    line_follow_loop(car, sensor, interpreter, controller, base_speed=interpreter.px_power, dt=0.05)


if __name__ == "__main__":
    main()
