from controller import Robot
import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctl

def run_robot(robot):

    time_step = 32
    max_speed = 6.28

    #motor
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    #ir
    right_ir_a = robot.getDevice('ir0')
    right_ir_a.enable(time_step)

    right_ir_b = robot.getDevice('ir1')
    right_ir_b.enable(time_step)
    
    mid_ir_a = robot.getDevice('ir2')
    mid_ir_a.enable(time_step)
    
    mid_ir_b = robot.getDevice('ir3')
    mid_ir_b.enable(time_step)
    
    mid_ir_c = robot.getDevice('ir4')
    mid_ir_c.enable(time_step)
    
    left_ir_a = robot.getDevice('ir5')
    left_ir_a.enable(time_step)
    
    left_ir_b = robot.getDevice('ir6')
    left_ir_b.enable(time_step)

    while robot.step(time_step) != -1:

        #pembacaan ir
        left_ir_b_value = np.fmax(left_ir_b.getValue(), left_ir_a.getValue())
        mid_ir_value = np.fmax(np.fmax(mid_ir_a.getValue(), mid_ir_b.getValue()), mid_ir_c.getValue())
        right_ir_a_value = np.fmax(right_ir_a.getValue(), right_ir_b.getValue())

        # left_ir_b_value = left_ir_b.getValue()
        # mid_ir_value = mid_ir.getValue()
        # right_ir_a_value = right_ir_a.getValue()

        print('Left {} Right {} Mid {}'.format(left_ir_b_value, right_ir_a_value, mid_ir_value))
        
        left_speed = -max_speed
        right_speed = -max_speed
        
        if left_ir_b_value < 400 and right_ir_a_value < 400 and mid_ir_value >= 400:
            #print("Maju")
            #print(left_speed)
            #print(right_speed)
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
        
        if left_ir_b_value < 400 and right_ir_a_value >= 400 and mid_ir_value >= 400:
            #print("Belok Kanan")
            #print(left_speed)
            #print(right_speed)
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(0)
        
        if left_ir_b_value >= 400 and right_ir_a_value < 400 and mid_ir_value >= 400:
            #print("Belok Kiri")
            #print(left_speed)
            #print(right_speed)
            left_motor.setVelocity(0)
            right_motor.setVelocity(right_speed)
        
        if left_ir_b_value >= 400 and right_ir_a_value < 400 and mid_ir_value < 400:
            #print("Belok Kiri")
            #print(left_speed)
            #print(right_speed)
            left_motor.setVelocity(0)
            right_motor.setVelocity(right_speed)
        
        if left_ir_b_value < 400 and right_ir_a_value >= 400 and mid_ir_value < 400:
            #print("Belok Kanan")
            #print(left_speed)
            #print(right_speed)
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(0)
        
        if left_ir_b_value < 400 and right_ir_a_value < 400 and mid_ir_value < 400:
            #print("Lurus")
            #print(left_speed)
            #print(right_speed)
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            
        if left_ir_b_value >= 400 and right_ir_a_value >= 400 and mid_ir_value >= 400:
            #print("Lurus banget")
            #print(left_speed)
            #print(right_speed)
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

        if left_ir_b_value >= 400 and right_ir_a_value >= 400 and mid_ir_value < 400:
            #print("robot mencari jalur di kiri")  
            #print(left_speed)
            #print(right_speed)        
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(0)

        # left_motor.setVelocity(left_speed)
        # right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)