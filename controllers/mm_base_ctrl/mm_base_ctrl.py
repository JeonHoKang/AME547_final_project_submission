"""ur5_webot_ctrl controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Robot, DistanceSensor, Motor
import math
import time

# create the Robot instance.
robot = Robot()


# get the time step of the current world.
pi = math.pi
timestep = 2*int(robot.getBasicTimeStep())
base = []
state = 'start'
base_joints = ['front_left_wheel_joint','front_right_wheel_joint','back_left_wheel_joint','back_right_wheel_joint']
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:

for j in range(len(base_joints)):
    base.append(robot.getDevice(base_joints))

# Main loop:
# - perform simulation steps until Wem[0].setVelocity(0.1)bots is s,topping the controller

class MoveBase(object):
    
    def __init__(self):
        self.state = 'start'
       
    def move_base(self):
        base[0].setVelocity(1)
        base[1].setVelocity(1)
        base[2].setVelocity(1)
        base[3].setVelocity(1)
        
command = MoveTheRobot()
    
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
    # Enter here functions to send actuator commands, like:
    time = robot.getTime()
    print(time)
    if time >= 0:
        command.move_base()
    pass