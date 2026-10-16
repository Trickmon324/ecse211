import time

import math

from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

######## PROGRAM INPUTS #########

## MODE ##
MOTOR_POWER: True #!!!!!!! ONLY ONE CAN BE TRUE !!!!!!!!
MOTOR_ANGLE: False #To be implemented. Motor_power works with motor.set_limits(), motor_angle works with motor.setlimits(), then motor.set_position()

## TROUBLESHOOTER'S HEAVEN ##
GLOBAL_POWER = 10 #for MOTOR_POWER mode
GLOBAL_LIMIT = 30 #for MOTOR_ANGLE mode
OUTPUT_POSITION = False #print current position

## PORT MAPPING ##
left_motor = Motor("A")
right_motor = Motor("D")
left_color = EV3ColorSensor(1)
right_color = EV3ColorSensor(2)
wait_ready_sensors(True)

## PHYSICAL DIMENSIONS ##
WHEEL_RADIUS = 2.1
WHEEL_CIRCUMFERENCE = 2*WHEEL_RADIUS * math.pi
ROBOT_WIDTH = 12.5
TILE = 30.48


## GLOBAL POSITION VARIABLES ##
X = 15.24 #why is this something???
Y = 15.24 #this too
THETA = 0.0
RIGHT_SPEED = GLOBAL_POWER
LEFT_SPEED = GLOBAL_POWER

prev_left_encoder = left_motor.get_position()
prev_right_encoder = right_motor.get_position()

def set_motors_forward(GLOBAL_POWER):
    right_motor.set_power(GLOBAL_POWER)
    left_motor.set_power(GLOBAL_POWER)
    
    
def set_right_motors_speed(i = GLOBAL_POWER):
    right_motor.set_limits(i)
    
def set_left_motors_speed(i = GLOBAL_POWER):
    left_motor.set_limits(i)


def stop_motors():
    left_motor.set_power(0)
    right_motor.set_power(0)
    

    
    
    
def update_odometer(Verbose = False): #MOST IMPORTANT! Run at every loop
    global prev_left_encoder, prev_right_encoder, THETA, X, Y
    phi_l = left_motor.get_position()
    d_phi_l = (phi_l - prev_left_encoder)
    phi_r = right_motor.get_position()
    d_phi_r = (phi_r - prev_right_encoder)
    d_l = d_phi_l*WHEEL_CIRCUMFERENCE
    d_r = d_phi_r*WHEEL_CIRCUMFERENCE
    d_theta = math.degrees(math.atan(math.radians((d_l-d_r)/ROBOT_WIDTH))) #could have been a big bug
    THETA += d_theta
    Distance = (d_l+d_r)/2
    d_x = Distance * math.cos(THETA) #could be a source of error
    d_y = Distance * math.sin(THETA) #this too. not sure if sin and cos are in the right place after fixing line 87
    X += d_x
    Y += d_y

    prev_left_encoder = phi_l
    prev_right_encoder = phi_r
    if Verbose:
        print(f"X: {X}, Y: {Y}, THETA: {THETA}, Distance: {Distance}")

def motor_speed_compensation(target_angle): #Use motor values to compensate for weaker motors by giving them more speed
    global THETA, RIGHT_SPEED, LEFT_SPEED,GLOBAL_POWER,ROBOT_WIDTH,TILE
    new_speed = GLOBAL_POWER*( 1+math.tan(target_angle-THETA)*ROBOT_WIDTH/GLOBAL_POWER)
    RIGHT_SPEED = new_speed
    right_motor.set_power(new_speed)


if __name__ == "__main__":
    try:
        current_total_left = 0
        current_total_right = 0
        past_total_left = 420
        past_total_right = 500
        time.sleep(1)
        while True:
            set_motors_forward(GLOBAL_POWER)
            leftMotorSleep = False
            rightMotorSleep = False
            first_motor = None
            time.sleep(.5)

            while True:
                update_odometer(OUTPUT_POSITION)
                if (first_motor == None) and  False: #stop compensating when reaching the line, set to True for troubleshooting
                    motor_speed_compensation(0)
                    
                left_val = left_color.get_value()
                right_val = right_color.get_value()


                current_total_left = left_val[0] + left_val[1] + left_val[2]
                current_total_right = right_val[0] + right_val[1] + right_val[2]
                print(str(current_total_left - past_total_left) + " left difference")
                print(str(current_total_right - past_total_right), " right differenece")

                if current_total_left - past_total_left < -150: #If left arrives first
                    left_motor.set_power(0)
                    if (first_motor == None):
                        first_motor = 1
                    leftMotorSleep = True

                if current_total_right - past_total_right < -150:#if right arrives first
                    right_motor.set_power(0)
                    if (first_motor == None):
                        first_motor = 0
                    rightMotorSleep = True

                if leftMotorSleep and rightMotorSleep:
                    if first_motor == 1:
                        left_motor.set_power(-10)
                        if current_total_left - past_total_left < -150:
                            left_motor.set_power(0)
                            break

                    else:
                        right_motor.set_power(-10)
                        if current_total_right - past_total_right < -150:
                            right_motor.set_power(0)
                            break

                # past_total_left = current_total_left
                # past_total_right = current_total_right


    except KeyboardInterrupt:
        pass
    finally:
        reset_brick()

