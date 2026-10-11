from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

servo = kit.servo[2]

servo.actuation_range = 180
servo.set_pulse_width_range(450, 2550)

servo.angle = 0
time.sleep(2)

servo.angle = 90
time.sleep(2)

servo.angle = 180