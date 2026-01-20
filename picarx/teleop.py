import logging
import time
from picarx_improved import Picarx
from maneuvers import drive, parallel_park_left, parallel_park_right, k_turn_left, k_turn_right

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)  # comment this out to silence DEBUG

def print_menu():
    print("\nCommands:")
    print("  w  = forward straight")
    print("  s  = backward straight")
    print("  a  = forward left")
    print("  d  = forward right")
    print("  z  = backward left")
    print("  c  = backward right")
    print("  pl = parallel park left")
    print("  pr = parallel park right")
    print("  kl = k-turn left")
    print("  kr = k-turn right")
    print("  x  = stop")
    print("  q  = quit")

def main():
    px = Picarx()
    print_menu()

    sp = 80
    ttd = 3.0

    try:
        while True:
            cmd = input("\nEnter command: ").strip().lower()

            if cmd == "q":
                break
            elif cmd == "w":
                drive(px, speed=sp, steering=0, duration=ttd)
            elif cmd == "s":
                drive(px, speed=-sp, steering=0, duration=ttd)
            elif cmd == "a":
                drive(px, speed=sp, steering=-20, duration=ttd)
            elif cmd == "d":
                drive(px, speed=sp, steering=20, duration=ttd)
            elif cmd == "z":
                drive(px, speed=-sp, steering=-20, duration=ttd)
            elif cmd == "c":
                drive(px, speed=-sp, steering=20, duration=ttd)
            elif cmd == "pl":
                parallel_park_left(px)
            elif cmd == "pr":
                parallel_park_right(px)
            elif cmd == "kl":
                k_turn_left(px)
            elif cmd == "kr":
                k_turn_right(px)
            elif cmd == "x":
                px.stop()
                px.set_dir_servo_angle(0)
            else:
                print("Unknown command.")
                print_menu()

    finally:
        # safety shutdown
        px.stop()
        px.set_dir_servo_angle(0)

if __name__ == "__main__":
    main()
