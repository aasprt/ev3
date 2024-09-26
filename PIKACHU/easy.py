#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

def follow_line(motor_left, motor_right, color_left, color_right):
    
    Kp = """ valore da decidere """
        
    if (color_left <= 40 and color_right <= 40) or (color_left <= 150 and color_right <= 150):
        motor_left.dc(""" valore da decidere """)
        motor_right.dc(""" valore da decidere """)
    else:
        error = color_left - color_right
        velocity = error * Kp

        motor_left.dc(velocity)
        motor_right.dc(""" valore da decidere """)
        
def make_intersection(motor_left, motor_right, direction):
    
    if direction == "left":
        motor_right.run_time(""" velocita da decidere """, """ tempo da decidere """)
    elif direction == "right":
        motor_left.run_time(""" velocita da decidere """, """ tempo da decidere """)            
    elif direction == "back":
        motor_left.dc(""" velocita da decidere """)
        motor_right.dc(""" velocita da decidere """)
        time.sleep(""" tempo da decidere """)
        motor_left.dc(0)
        motor_right.dc(0)

def avoid_obstacle(motor_left, motor_right):
    motor_right.run_target(""" velocita da decidere """, """ terget da decidere """)

    motor_left.dc(20)
    motor_right.dc(20)
    time.sleep(""" tempo da decidere """)
    motor_left.dc(0)
    motor_right.dc(0)
    
    motor_left.run_target(""" velocita da decidere """, """ terget da decidere """)

    motor_left.dc(20)
    motor_right.dc(20)
    time.sleep(""" tempo da decidere """)
    motor_left.dc(0)
    motor_right.dc(0)

    motor_left.run_target(""" velocita da decidere """, """ terget da decidere """)
    
    motor_left.dc(20)
    motor_right.dc(20)
    time.sleep(""" tempo da decidere (stesso di prima piu o meno) """)
    motor_left.dc(0)
    motor_right.dc(0)

    motor_right.run_target(""" velocita da decidere """, """ terget da decidere """)
  
#algoritmo principale
motor_left = Motor(Port.A)
motor_right = Motor(Port.D)
color_sensor_left = ColorSensor(Port.S1)
color_sensor_right = ColorSensor(Port.S2)
ultrasonic_sensor = UltrasonicSensor(Port.S3)

in_room = False

while not in_room:
    distance = ultrasonic_sensor.distance()
    rgb_right = color_sensor_right.rgb()
    rgb_left = color_sensor_left.rgb()
    color_sum_left = rgb_left[0] + rgb_left[1] + rgb_left[2]
    color_sum_right = rgb_right[0] + rgb_right[1] + rgb_right[2]

    if distance <= 100:
        motor_left.hold()
        motor_right.hold()
        ev3.speaker.beep()
        avoid_obstacle(motor_left, motor_right)
    elif (rgb_left[0] <= """ valore da decidere """ and rgb_left[1] >= """ valore da decidere """ and rgb_left[2] <= """ valore da decidere """) and color_sum_right <= """ valore da decidere """:
        motor_left.hold()
        motor_right.hold()
        ev3.speaker.beep()
        make_inetrsection(motor_left, motor_right, "left")
    elif (rgb_right[0] <= """ valore da decidere """ and rgb_right[1] >= """ valore da decidere """ and rgb_right[2] <= """ valore da decidere """ ) and color_sum_left <= """ valore da decidere """:
        motor_left.hold()
        motor_right.hold()
        ev3.speaker.beep()
        make_intersection(motor_left, motor_right, "right")   
    elif (rgb_left[0] <= """ valore da decidere """ and rgb_left[1] >= """ valore da decidere """ and rgb_left[2] <= """ valore da decidere """) and (rgb_right[0] <= """ valore da decidere """ and rgb_right[1] >= """ valore da decidere """ and rgb_right[2] <= """ valore da decidere """ ):
        motor_left.hold()
        motor_right.hold()
        ev3.speaker.beep()
        make_intersection(motor_left, motor_right, "back")
    else:
        follow_line(motor_left, motor_right, color_sum_left, color_sum_right)