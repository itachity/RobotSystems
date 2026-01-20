import time
import logging
from picarx_improved import Picarx

ttd = 3.0

def drive(px: Picarx, speed: int, steering: int, duration: float):
    px.set_dir_servo_angle(steering)
    if speed >= 0:
        px.forward(speed)
    else:
        px.backward(abs(speed))
    time.sleep(duration)
    px.stop()
    px.set_dir_servo_angle(0)

def parallel_park_left(px: Picarx, speed=35):
    logging.info("Parallel park LEFT")
    px.set_dir_servo_angle(-25)
    px.backward(speed); time.sleep(ttd)
    px.set_dir_servo_angle(25)
    px.backward(speed); time.sleep(ttd)
    px.stop()
    px.set_dir_servo_angle(0)

def parallel_park_right(px: Picarx, speed=35):
    logging.info("Parallel park RIGHT")
    px.set_dir_servo_angle(25)
    px.backward(speed); time.sleep(ttd)
    px.set_dir_servo_angle(-25)
    px.backward(speed); time.sleep(ttd)
    px.stop()
    px.set_dir_servo_angle(0)

def k_turn_left(px: Picarx, speed=40):
    logging.info("K-turn LEFT")
    # forward left
    px.set_dir_servo_angle(-25)
    px.forward(speed); time.sleep(ttd)
    px.stop()

    # reverse right
    px.set_dir_servo_angle(25)
    px.backward(speed); time.sleep(ttd)
    px.stop()

    # forward straighten
    px.set_dir_servo_angle(0)
    px.forward(speed); time.sleep(5/8*ttd)
    px.stop()

def k_turn_right(px: Picarx, speed=40):
    logging.info("K-turn RIGHT")
    px.set_dir_servo_angle(25)
    px.forward(speed); time.sleep(ttd)
    px.stop()

    px.set_dir_servo_angle(-25)
    px.backward(speed); time.sleep(ttd)
    px.stop()

    px.set_dir_servo_angle(0)
    px.forward(speed); time.sleep(5/8*ttd)
    px.stop()
