import time

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC

adc_pins = ["A0", "A1", "A2"]
adcs = [ADC(p) for p in adc_pins]

while True:
    vals = [a.read() for a in adcs]
    print(vals)
    time.sleep(0.05)
