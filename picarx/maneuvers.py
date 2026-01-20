import time
import logging
from picarx_improved import Picarx

ttd = 2.0
sp = 60

SERVO_SETTLE = 0.15  # let steering move before driving

def _hard_turn(px: Picarx):
    # 90% of max steering to avoid binding
    return int(0.90 * px.DIR_MAX)

def _med_turn(px: Picarx):
    return int(0.60 * px.DIR_MAX)

def drive(px: Picarx, speed: int, steering: int, duration: float):
    px.set_dir_servo_angle(steering)
    time.sleep(SERVO_SETTLE)

    if speed >= 0:
        px.forward(speed)
    else:
        px.backward(abs(speed))

    time.sleep(duration)
    px.stop()
    px.set_dir_servo_angle(0)

def parallel_park_left(px: Picarx, speed=sp):
    logging.info("Parallel park LEFT")
    H = _hard_turn(px)

    # Reverse while steering LEFT (full-ish lock)
    px.set_dir_servo_angle(-H)
    time.sleep(SERVO_SETTLE)
    px.backward(speed); time.sleep(ttd)

    # Reverse while steering RIGHT (counter-steer)
    px.set_dir_servo_angle(+H)
    time.sleep(SERVO_SETTLE)
    px.backward(speed); time.sleep(ttd)

    px.stop()
    px.set_dir_servo_angle(0)

def parallel_park_right(px: Picarx, speed=sp):
    logging.info("Parallel park RIGHT")
    H = _hard_turn(px)

    # Reverse while steering RIGHT
    px.set_dir_servo_angle(+H)
    time.sleep(SERVO_SETTLE)
    px.backward(speed); time.sleep(ttd)

    # Reverse while steering LEFT (counter-steer)
    px.set_dir_servo_angle(-H)
    time.sleep(SERVO_SETTLE)
    px.backward(speed); time.sleep(ttd)

    px.stop()
    px.set_dir_servo_angle(0)

def k_turn_left(px: Picarx, speed=sp):
    logging.info("K-turn LEFT")
    H = int(0.90 * px.DIR_MAX)

    # 1) Forward left
    px.set_dir_servo_angle(-H); time.sleep(SERVO_SETTLE)
    px.forward(speed); time.sleep(ttd)
    px.stop()

    # 2) Reverse right
    px.set_dir_servo_angle(+H); time.sleep(SERVO_SETTLE)
    px.backward(speed); time.sleep(ttd)
    px.stop()

    # 3) Forward left again 
    px.set_dir_servo_angle(-H); time.sleep(SERVO_SETTLE)
    px.forward(speed); time.sleep(0.65 * ttd)
    px.stop()

    # straighten
    px.set_dir_servo_angle(0)


def k_turn_right(px: Picarx, speed=sp):
    logging.info("K-turn RIGHT")
    H = int(0.90 * px.DIR_MAX)

    # 1) Forward right
    px.set_dir_servo_angle(+H); time.sleep(SERVO_SETTLE)
    px.forward(speed); time.sleep(ttd)
    px.stop()

    # 2) Reverse left
    px.set_dir_servo_angle(-H); time.sleep(SERVO_SETTLE)
    px.backward(speed); time.sleep(ttd)
    px.stop()

    # 3) Forward right again
    px.set_dir_servo_angle(+H); time.sleep(SERVO_SETTLE)
    px.forward(speed); time.sleep(0.65 * ttd)
    px.stop()

    # straighten
    px.set_dir_servo_angle(0)
