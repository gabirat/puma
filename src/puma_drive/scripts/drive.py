#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist

class MotorController:
    
    PINOUT = {
        "M0_I1": 13,  #Motor 0, Input 1
        "M0_I2": 15,  #Motor 0, Input 2
        "M0_PWM": 32, #Motor 0, Enable Pin
        "M1_I1": 16,  #Motor 1, Input 1
        "M1_I2": 18,  #Motor 1, Input 2
        "M1_PWM": 33  #Motor 1, Enable Pin
    }

    # Truth table of driver L298
    # MX_I1 | MX_I2 | OUTPUT
    #   1   |   0   |  Clockwise direction 
    #   0   |   1   |  Counterclockwise direction 
    #   1   |   1   |  When indicated high state at the PWM input - fast braking of the motors (fast stop). 
    #   0   |   0   |  When indicated high state at the PWM input - fast braking of the motors (fast stop). 
    #   1   |   1   |  When indictaed low state at the PWM input - free braking (soft stop). 

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        
        # Set all pins as outputs
        for pin in MotorController.PINOUT:
            GPIO.setup(MotorController.PINOUT[pin], GPIO.OUT)

        self.motors = []
        self.motors.append({
            "I1":  MotorController.PINOUT["M0_I1"],
            "I2":  MotorController.PINOUT["M0_I2"],
            "PWM": GPIO.PWM(MotorController.PINOUT["M0_PWM"], 100)
        })
        self.motors.append({
            "I1":  MotorController.PINOUT["M1_I1"],
            "I2":  MotorController.PINOUT["M1_I2"],
            "PWM": GPIO.PWM(MotorController.PINOUT["M1_PWM"], 100)
        })

        for motor in self.motors:
            motor["PWM"].start(0)

        rospy.Subscriber('cmd_vel', Twist, self.drive_data)

    def set_motor_speed(self, motor_id, speed):
        speed = 1.0 if speed > 1.0 else speed
        speed = -1.0 if speed < -1.0 else speed

        if speed > 0:
            # Clockwise
            GPIO.output(self.motors[motor_id]["I1"], 0)
            GPIO.output(self.motors[motor_id]["I2"], 1)
        else:
            # Counterclockwise
            GPIO.output(self.motors[motor_id]["I1"], 1)
            GPIO.output(self.motors[motor_id]["I2"], 0)

        self.motors[motor_id]["PWM"].ChangeDutyCycle(100.0 * abs(speed))

    def drive_data(self, data):
        angular_speed = abs(data.angular.z)
        forward_speed = data.linear.x

        # Rotation in place
        if data.linear.x == 0:
            direction = 1 if data.angular.z > 0 else -1
            self.set_motor_speed(0, angular_speed * direction)
            self.set_motor_speed(1, angular_speed * -direction)
            return
        
        # Normal drive
        # Turn right
        if data.angular.z > 0:
            self.set_motor_speed(0, forward_speed)
            self.set_motor_speed(1, (1 - angular_speed) * forward_speed)
        # Turn left
        else:
            self.set_motor_speed(0, (1 - angular_speed) * forward_speed)
            self.set_motor_speed(1, forward_speed)


    def turn_off(self):
        for motor in self.motors:
            motor["PWM"].stop()
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        rospy.init_node('puma_drive')
        drive = MotorController()
        rospy.on_shutdown(drive.turn_off)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass