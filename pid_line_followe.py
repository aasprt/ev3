from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# P.I.D. line follower

# assegnazione porte
ev3 = EV3Brick()
left_motor = Motor(Port.1)
right_motor = Motor(Port.2)
left_light = ColorSensor(Port.S1)
right_light = ColorSensor(Port.S2)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# variabili

error = 0
last_error = 0  # viene modificata nel proramma
integral = 0  # viene modificata nel proramma
derivative = 0  # viene modificata nel proramma
Kp = 1  # valore arbitrario da decidere in fase di testing
Ki = 1  # valore arbitrario da decidere in fase di testing
Kd = 1  # valore arbitrario da decidere in fase di testing
go = true  # serve per fermare l'esecuzione del programma se necessario
velocity = 50

# PID line follower: programma

while go:
    error = left_light.reflection() - right_light.reflection()
    integral += error
    derivative = error - last_error
    velocity = (error * Kp) + (integral * Ki) + (derivative * Kd)
    left_motor.run(velocity)
    right_motor = 50
    last_error = error

    if Button.DOWN in brick.buttons():
    go = false
