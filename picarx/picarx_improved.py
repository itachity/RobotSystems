import os

on_the_robot = True
try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic, utils
except ImportError:
    on_the_robot = False
    import sys
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic, utils
import time

# Debugging Section

import logging

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

#from logdecorator import log_on_start, log_on_end, log_on_error

# Code Improvement imports
import atexit
import math

def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # Rough geometry
    WHEELBASE_M = 0.10     # meters (front axle to rear axle)
    TRACK_WIDTH_M = 0.12   # meters (left-to-right wheel distance)

    MIN_TURN_PWM = 30      # prevents inside wheel stalling during turns


    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: trig, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P13', 'P12'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # reset robot_hat
        if on_the_robot:
            utils.reset_mcu()
            time.sleep(0.2)


        # --------- config_flie ---------
        if on_the_robot:
            self.config_flie = fileDB(config, 777, os.getlogin())
        else:
            self.config_flie = fileDB(config, 777, owner=None)


        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        trig, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(trig), Pin(echo, mode=Pin.IN, pull=Pin.PULL_DOWN))

        # stop motors on exit
        atexit.register(self.stop)

    
    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        logging.debug(f"direction: {direction}, speed: {speed}")
        
        speed = int(speed)
        speed = speed - self.cali_speed_value[motor]
        speed = constrain(speed, 0, 100) # valid pwm range

        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    # Code Improvement helper functions
    def _ackermann_inner_ratio(self, angle_deg: float) -> float:
        """
        Returns inner/outer speed ratio based on a simple Ackermann approximation.
        Output is clamped to [0.4..1.0] so it doesn't get crazy.
        """
        a = abs(angle_deg)
        a = constrain(a, 0.0, self.DIR_MAX)

        if a < 1e-6:
            return 1.0

        delta = math.radians(a)
        # Turning radius of the vehicle centerline
        R = self.WHEELBASE_M / math.tan(delta)

        # ratio = v_inner / v_outer
        ratio = (R - self.TRACK_WIDTH_M / 2.0) / (R + self.TRACK_WIDTH_M / 2.0)

        return constrain(ratio, 0.4, 1.0)


    def backward(self, speed):
        angle = self.dir_current_angle
        ratio = self._ackermann_inner_ratio(angle)

        outer = float(speed)
        inner = outer * ratio

        if abs(angle) > 1e-3 and inner > 0 and inner < self.MIN_TURN_PWM:
            inner = float(self.MIN_TURN_PWM)

        if angle > 0:
            left_cmd = inner
            right_cmd = outer
        elif angle < 0:
            left_cmd = outer
            right_cmd = inner
        else:
            left_cmd = outer
            right_cmd = outer

        logging.debug(
            f"backward speed={speed}, angle={angle}, ratio={ratio:.2f}, "
            f"left_cmd={left_cmd:.1f}, right_cmd={right_cmd:.1f}"
        )

        self.set_motor_speed(1, -left_cmd)
        self.set_motor_speed(2, right_cmd)



    def forward(self, speed):
        angle = self.dir_current_angle
        ratio = self._ackermann_inner_ratio(angle)

        outer = float(speed)
        inner = outer * ratio

        # prevent inside wheel stall when turning
        if abs(angle) > 1e-3 and inner > 0 and inner < self.MIN_TURN_PWM:
            inner = float(self.MIN_TURN_PWM)

        if angle > 0:
            # steering +: assume left is inner (swap if your steering is opposite)
            left_cmd = inner
            right_cmd = outer
        elif angle < 0:
            left_cmd = outer
            right_cmd = inner
        else:
            left_cmd = outer
            right_cmd = outer

        logging.debug(
            f"forward speed={speed}, angle={angle}, ratio={ratio:.2f}, "
            f"left_cmd={left_cmd:.1f}, right_cmd={right_cmd:.1f}"
        )

        self.set_motor_speed(1, left_cmd)
        self.set_motor_speed(2, -right_cmd)

                   

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

        logging.debug("stop(): PWM -> 0")

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def reset(self):
        self.stop()
        self.set_dir_servo_angle(0)
        self.set_cam_tilt_angle(0)
        self.set_cam_pan_angle(0)

    def close(self):
        self.reset()
        self.ultrasonic.close()

if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    px.backward(50)
    px.stop()
