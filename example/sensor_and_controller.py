import time

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC

class Sensor():

    def __init__(self):

        self.adcs = [ADC(p) for p in ["A0", "A1", "A2"]]

    def poll(self):
        while True:
            vals = [a.read() for a in self.adcs]
            print(vals)
            time.sleep(0.05)


sense = Sensor()
sense.poll()
