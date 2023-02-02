from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()
motor_left = Motor(Port.A)
motor_right = Motor(Port.D)
light_left = Motor(Port.S1)
light_right = Motor(Port.S2)

robot = DriveBase(motor_left, motor_right, wheel_diameter=55.5, axle_track=104)

taget_path = 45
proportional_gain = 0.5
dirve_speed = 50
go = true

ev3.speaker.beep()
while go:
  #obstacle_avoidance
  

  #line_follower
  if light_right.reflection() < target_path or light_left > target_path:
    motor_left.run(int(proportional_gain * light_left.reflection() - target_path))
    motor_right.run(drive_speed)
  elif light_left.reflection() < target_path or light_right > target_path:
    motor_right.run(int(proportional_gain * light_left.reflection() - target_path))
    motor_left.run(drive_speed)
  else:
    motor_left.run(drive_speed)
    motor_right.run(drive_speed)
  
  if Button.DOWN in brick.buttons():
    go = false
    
