"""python_base_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
from math import pi, sin
import time

# create the Robot instance.
robot = Robot()
wheel1 = robot.getDevice("front_left_wheel_joint")
wheel2 = robot.getDevice("front_right_wheel_joint")
wheel3 = robot.getDevice("back_left_wheel_joint")
wheel4 = robot.getDevice("back_right_wheel_joint")

#Set up the acceleration and PID parameters for smooth movement
timestep = int(robot.getBasicTimeStep())
acc = 20
KP = 3
KI = 0
KD = 0

wheel1.setAcceleration(acc)
wheel2.setAcceleration(acc)
wheel3.setAcceleration(acc)
wheel4.setAcceleration(acc)

wheel1.setControlPID(KP,KI,KD)
wheel2.setControlPID(KP,KI,KD)
wheel3.setControlPID(KP,KI,KD)
wheel4.setControlPID(KP,KI,KD)


#Function to move the wheels to the desired location
def move(wheel_pos,velo):

    wheel1.setVelocity(velo)
    wheel2.setVelocity(velo)
    wheel3.setVelocity(velo)
    wheel4.setVelocity(velo)
    
    
    wheel1.setPosition(wheel_pos[0])
    wheel2.setPosition(wheel_pos[1])
    wheel3.setPosition(wheel_pos[2])
    wheel4.setPosition(wheel_pos[3])


#Function to calculate the desired wheel position for the movement
def CalcNewPos(wheel_pos,dist,direction):
    if direction == "forward":
        off1 = wheel_pos[0] + dist
        off2 = wheel_pos[1] + dist
        off3 = wheel_pos[2] + dist
        off4 = wheel_pos[3] + dist
        
    if direction == "backward":
        off1 = wheel_pos[0] - dist
        off2 = wheel_pos[1] - dist
        off3 = wheel_pos[2] - dist
        off4 = wheel_pos[3] - dist
        
    if direction == "left":
        off1 = wheel_pos[0] - dist
        off2 = wheel_pos[1] + dist
        off3 = wheel_pos[2] + dist
        off4 = wheel_pos[3] - dist
        
    if direction == "right":
        off1 = wheel_pos[0] + dist
        off2 = wheel_pos[1] - dist
        off3 = wheel_pos[2] - dist
        off4 = wheel_pos[3] + dist 
        
    if direction == "rot_right":
        dist = dist/17
        off1 = wheel_pos[0] + dist
        off2 = wheel_pos[1] - dist
        off3 = wheel_pos[2] + dist
        off4 = wheel_pos[3] - dist 
          
          
    if direction == "rot_left":
        dist = dist/17
        off1 = wheel_pos[0] - dist
        off2 = wheel_pos[1] + dist
        off3 = wheel_pos[2] - dist
        off4 = wheel_pos[3] + dist 
        
    return off1,off2,off3,off4


#Get intial position before moving
old_wheel_pos = [wheel1.getTargetPosition(),wheel2.getTargetPosition(),wheel3.getTargetPosition(),wheel4.getTargetPosition()]
t = 0

#Add movements to the list here
#moveX = [completion,distance/angle,direction,start time, end time]
moves = [[0,5,"backward",0,5],
        [0,90,"rot_left",5,10],
        [0,90,"rot_right",10,15],
        [0,5,"right",15,20],
        [0,5,"forward",20,25],
        [0,5,"backward",25,30],
        [0,5,"forward",30,35]]

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    for x in range(0,len(moves)):
        print(round(t,2))
        if t > moves[x][3] and t < moves[x][4]:
            if moves[x][0] == 0:
                new_wheel_pos = CalcNewPos(old_wheel_pos,moves[x][1],moves[x][2])
                old_wheel_pos = new_wheel_pos
                moves[x][0] = 1
                print(new_wheel_pos)
            else:
                move(new_wheel_pos,10)
                
    t += timestep / 1000.0
    
    pass

# Enter here exit cleanup code.
