#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

# oggetto robot
class Robot:
    
    #inizializzo l'ogetto e assegno i coefficienti del PID
    def __init__(self, kp = 1.9, ki = 0.007, kd = 8):

        #configurazione delle porte dei motori e dei sensori
        self.color_sensor_left = ColorSensor(Port.S1)
        self.color_sensor_right = ColorSensor(Port.S2)
        self.ultrasonic_sensor = UltrasonicSensor(Port.S3)  
        self.motor_left = Motor(Port.A)
        self.motor_right = Motor(Port.D)
        
        #assegnazione coefficienti PID
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

        #variabili PID da conservare
        self.error = 0
        self.last_error = 0
        self.integral = 0
        self.derivate = 0

    
    #metodo per girare all'incrocio
    def turn(self, direction):
        
        #girare a sinistra caso verde a sinistra
        if direction == "left":
            self.motor_right.run_time(300, 2500)
        
        #girare a destra caso verde a destra
        elif direction == "right":
            self.motor_left.run_time(300, 2500)
            
        #tonare indietro caso doppio verde
        elif direction == "back":
            self.motor_left.dc(-30)
            self.motor_right.dc(30)
            time.sleep(1)
            self.motor_left.dc(0)
            self.motor_right.dc(0)
            
            
    def follow_line(self, rgb_left, rgb_right):
        
        #somma dei tuple del ritorno dei sensori        
        color_sum_left = rgb_left[0] + rgb_left[1] + rgb_left[2]
        color_sum_right = rgb_right[0] + rgb_right[1] + rgb_right[2]
        
        # caso doppio bianco o doppio nero --> 
        if (color_sum_left >= 190 and color_sum_right >= 190) or (color_sum_left <= 40 and color_sum_right <= 40):

            #va dritto
            self.motor_left.dc(30)
            self.motor_right.dc(30)
            
        #caso verde a sinistra e nero a destra -- valori da definire
        elif (rgb_left[0] <= 30 and rgb_left[1] >= 50 and rgb_left[2] <= 30) and color_sum_right <= 40:

            #gira  a sinistra
            turn("left")
            
        #caso verde a destra e nero a sinistra -- valori da definire
        elif (rgb_right[0] <= 30 and rgb_right[1] >= 50 and rgb_right[2] <= 30) and color_sum_left <= 40:

            #gira a destra
            turn("right")
        
        #caso doppio verde -- valori da definire
        elif (rgb_left[0] <= 30 and rgb_left[1] >= 50 and rgb_left[2] <= 30) and (rgb_right[0] <= 30 and rgb_right[1] >= 50 and rgb_right[2] <= 30):

            #torna indietro
            turn("back")
            #gira  a sinistra
            turn("left")
            
        #caso verde a destra e nero a sinistra -- valori da definire
        elif (rgb_right[0] <= 30 and rgb_right[1] >= 50 and rgb_right[2] <= 30) and color_sum_left <= 40:

            #gira a destra
            turn("right")
        
        #caso doppio verde -- valori da definire
        elif (rgb_left[0] <= 30 and rgb_left[1] >= 50 and rgb_left[2] <= 30) and (rgb_right[0] <= 30 and rgb_right[1] >= 50 and rgb_right[2] <= 30):

            #torna indietro
            turn("back")
            
        #se non viene rispettato nessuno dei citeri precedenti segui la linea normalmente
        else:
            
            #calcolo errore come differenza delle somme dei tuple RGB
            self.error = color_sum_left - color_sum_right

            #calcolo integrale come somma degli errori nel tempo(in cicli)
            self.integral += self.error
            
            #calcolo della derivata come differenza tra lerrore corrente e l'ultimo errore
            self.derivate = self.error - self.last_error
            
            #assegnazione della velocita del motore destro
            velocity = int((self.error * self.Kp) + (self.integral * self.Ki) + (self.derivative * self.Kd))

            #velocita motori
            self.motor_left.dc(velocity)
            self.motor_right.dc(20)
            
            #passo l'errore alla variabile ultimo errore
            self.last_error = self.error

            #cose per il debug
            #print(color_sum_left, color_sum_right, self.error)
            #print(rgb_left, rgb_right)
                    
                    
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

#! ostacoli da vedere

#inizializzo brick
ev3 = EV3Brick()

#definisco Robot come robot
Robot = Robot()

#il robot si trova dentro la stanza?
inside_room = False

#cosa fare se non sta dentro la stanza
while not inside_room:
        
    if Robot.ultrasonic_sensor.distance() <= 100:
        Motor(Port.A).hold()
        Motor(Port.D).hold()
        ev3.speaker.beep()
        Robot.avoid_obstacle() 
    else:
        TupleR=Robot.color_sensor_right.rgb()
        TupleL=Robot.color_sensor_left.rgb()
        Robot.follow_line(TupleR,TupleL)
        
    
        
