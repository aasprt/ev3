#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

class Robot:
    def __init__(self, kp = 1.9, ki = 0.007, kd = 8):

        self.color_sensor_left = ColorSensor(Port.S1)
        self.color_sensor_right = ColorSensor(Port.S2)
        #self.ultrasonic_sensor = UltrasonicSensor(Port.S3)
        self.motor_left = Motor(Port.A)
        self.motor_right = Motor(Port.D)

        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

        self.error = 0
        self.last_error = 0
        self.integral = 0
        self.derivative = 0

    def follow_line(self):
    
        if (self.color_sensor_left.reflection() >= 70 and self.color_sensor_right.reflection() >= 70) or (self.color_sensor_left.reflection() <= 13 and self.color_sensor_right.reflection() <= 13):
            self.motor_left.dc(30)
            self.motor_right.dc(30)
        else:
            self.error = self.color_sensor_left.reflection() - self.color_sensor_right.reflection()
            self.integral += self.error
            self.drivative = self.error - self.last_error

            velocity = int((self.error * self.Kp) + (self.integral * self.Ki) + (self.derivative * self.Kd))
        
            self.motor_left.dc(velocity)
            self.motor_right.dc(25)

            self.last_error = self.error
        
            # print(self.color_sensor_left.reflection(), self.color_sensor_right.reflection(), self.motor_left.speed(), self.motor_right.speed(), int(self.error * self.Kp), int(self.integral * self.Ki), int(self.derivative * self.Kd))
    
    def avoid_obstacle(self):

        # gira a destra
        self.motor_left.run_time(300, 2500)

        # vai dritto 
        self.motor_left.dc(50)
        self.motor_right.dc(50)
        time.sleep(0.5)
        self.motor_left.dc(0)
        self.motor_right.dc(0)

        #gira a sinistra
        self.motor_right.run_time(300, 2500)

        self.motor_left.dc(50)
        self.motor_right.dc(50)
        time.sleep(2.0)
        self.motor_left.dc(0)
        self.motor_right.dc(0)

        self.motor_right.run_time(300, 2500)

        self.motor_left.run_time(300, 2500)

    def turn(self, direction):

        if direction == "left":
            self.motor_right.run_time(300, 2500)
        else:
            self.motor_left.run_time(300, 2500)

ev3 = EV3Brick()
Robot = Robot()
inside_room = False

while not inside_room:
    if (ColorSensor(Port.S1).reflection() >= 14 and ColorSensor(Port.S1).reflection() <= 16) or (ColorSensor(Port.S2).reflection() <= 14 and ColorSensor(Port.S2).reflection() >= 16):
        Motor(Port.A).hold()
        Motor(Port.D).hold()
        if (ColorSensor(Port.S1).color() == Color.GREEN or ColorSensor(Port.S1).color() == Color.BLUE):
            Robot.turn("left")
        elif (ColorSensor(Port.S2).color() == Color.GREEN or ColorSensor(Port.S1).color() == Color.BLUE):
            Robot.turn("right")
    #elif UltrasonicSensor(Port.S3).distance() <= 100:
        #Motor(Port.A).hold()
        #Motor(Port.D).hold()
        #ev3.speaker.beep()
        #Robot.avoid_obstacle()
    else:
        Robot.follow_line()