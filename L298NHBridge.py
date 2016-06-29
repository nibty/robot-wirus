#!/usr/bin/env python
# coding: latin-1

# Autor:	Ingmar Stapel
# Date:		20141229
# Version:	1.0
# Homepage:	www.raspberry-pi-car.com

# This module is designed to control two motors with a L298N H-Bridge

# Use this module by creating an instance of the class. To do so call the Init function, then command as desired, e.g.
# import L298NHBridge
# HBridge = L298NHBridge.L298NHBridge()
# HBridge.Init()

# Import the libraries the class needs
import RPi.GPIO as GPIO
from Adafruit_PWM_Servo_Driver import PWM
import config

GPIO.setmode(GPIO.BCM)

# Constant values
PWM_MAX                 = 100

# Disable warning from GPIO
GPIO.setwarnings(False)

# Here we configure the GPIO settings for the left and right motors spinning direction. 
# It defines the four GPIO pins used as input on the L298 H-Bridge to set the motor mode (forward, reverse and stopp).

leftmotor_in1_pin = config.LEFT_MOTOR_BACKWARD
leftmotor_in2_pin = config.LEFT_MOTOR_FORWARD
GPIO.setup(leftmotor_in1_pin, GPIO.OUT)
GPIO.setup(leftmotor_in2_pin, GPIO.OUT)

rightmotor_in1_pin = config.RIGHT_MOTOR_BACKWARD
rightmotor_in2_pin = config.RIGHT_MOTOR_FORWARD
GPIO.setup(rightmotor_in1_pin, GPIO.OUT)
GPIO.setup(rightmotor_in2_pin, GPIO.OUT)

GPIO.output(leftmotor_in1_pin, False)
GPIO.output(leftmotor_in2_pin, False)
GPIO.output(rightmotor_in1_pin, False)
GPIO.output(rightmotor_in2_pin, False)

# Here we configure the GPIO settings for the left and right motors spinning speed. 
# It defines the two GPIO pins used as input on the L298 H-Bridge to set the motor speed with a PWM signal.

pwm = PWM(config.PWM_ADDRESS)
pwm.setPWMFreq(config.PWM_FREQUENCY)

# leftmotorpwm_pin = 4
# rightmotorpwm_pin = 17
#
# GPIO.setup(leftmotorpwm_pin, GPIO.OUT)
# GPIO.setup(rightmotorpwm_pin, GPIO.OUT)
#
# leftmotorpwm = GPIO.PWM(leftmotorpwm_pin,100)
# rightmotorpwm = GPIO.PWM(rightmotorpwm_pin,100)
#
# leftmotorpwm.start(0)
# leftmotorpwm.ChangeDutyCycle(0)
#
# rightmotorpwm.start(0)
# rightmotorpwm.ChangeDutyCycle(0)


def set_left_motor_speed(speed):
    print "set_left_motor_speed " + str(speed)
    pwm.setPWM(config.PWM_LEFT_MOTOR_SPEED_CHANNEL, 0, int(speed * 40))


def set_right_motor_speed(speed):
    print "set_right_motor_speed " + str(speed)
    pwm.setPWM(config.PWM_RIGHT_MOTOR_SPEED_CHANNEL, 0, int(speed * 40))


def setMotorMode(motor, mode):

    # setMotorMode()

    # Sets the mode for the L298 H-Bridge which motor is in which mode.

    # This is a short explanation for a better understanding:
    # motor		-> which motor is selected left motor or right motor
    # mode		-> mode explains what action should be performed by the H-Bridge

    # setMotorMode(leftmotor, reverse)	-> The left motor is called by a function and set into reverse mode
    # setMotorMode(rightmotor, stopp)	-> The right motor is called by a function and set into stopp mode

    if motor == "leftmotor":
        if mode == "reverse":
            GPIO.output(leftmotor_in1_pin, True)
            GPIO.output(leftmotor_in2_pin, False)
        elif  mode == "forward":
            GPIO.output(leftmotor_in1_pin, False)
            GPIO.output(leftmotor_in2_pin, True)
        else:
            GPIO.output(leftmotor_in1_pin, False)
            GPIO.output(leftmotor_in2_pin, False)

    elif motor == "rightmotor":
        if mode == "reverse":
            GPIO.output(rightmotor_in1_pin, False)
            GPIO.output(rightmotor_in2_pin, True)
        elif  mode == "forward":
            GPIO.output(rightmotor_in1_pin, True)
            GPIO.output(rightmotor_in2_pin, False)
        else:
            GPIO.output(rightmotor_in1_pin, False)
            GPIO.output(rightmotor_in2_pin, False)
    else:
        GPIO.output(leftmotor_in1_pin, False)
        GPIO.output(leftmotor_in2_pin, False)
        GPIO.output(rightmotor_in1_pin, False)
        GPIO.output(rightmotor_in2_pin, False)

def setMotorLeft(power):

    # SetMotorLeft(power)

    # Sets the drive level for the left motor, from +1 (max) to -1 (min).

    # This is a short explanation for a better understanding:
    # SetMotorLeft(0)     -> left motor is stopped
    # SetMotorLeft(0.75)  -> left motor moving forward at 75% power
    # SetMotorLeft(-0.5)  -> left motor moving reverse at 50% power
    # SetMotorLeft(1)     -> left motor moving forward at 100% power

    if power < 0:
        # Reverse mode for the left motor
        setMotorMode("leftmotor", "reverse")
        pwm = -int(PWM_MAX * power)
        if pwm > PWM_MAX:
            pwm = PWM_MAX
    elif power > 0:
        # Forward mode for the left motor
        setMotorMode("leftmotor", "forward")
        pwm = int(PWM_MAX * power)
        if pwm > PWM_MAX:
            pwm = PWM_MAX
    else:
        # Stopp mode for the left motor
        setMotorMode("leftmotor", "stopp")
        pwm = 0
    	# print "SetMotorLeft", pwm

    set_left_motor_speed(pwm)

def setMotorRight(power):

    # SetMotorRight(power)

    # Sets the drive level for the right motor, from +1 (max) to -1 (min).

    # This is a short explanation for a better understanding:
    # SetMotorRight(0)     -> right motor is stopped
    # SetMotorRight(0.75)  -> right motor moving forward at 75% power
    # SetMotorRight(-0.5)  -> right motor moving reverse at 50% power
    # SetMotorRight(1)     -> right motor moving forward at 100% power

    if power < 0:
        # Reverse mode for the right motor
        setMotorMode("rightmotor", "reverse")
        pwm = -int(PWM_MAX * power)
        if pwm > PWM_MAX:
            pwm = PWM_MAX
    elif power > 0:
        # Forward mode for the right motor
        setMotorMode("rightmotor", "forward")
        pwm = int(PWM_MAX * power)
        if pwm > PWM_MAX:
            pwm = PWM_MAX
    else:
        # Stopp mode for the right motor
        setMotorMode("rightmotor", "stopp")
        pwm = 0
    	# print "SetMotorRight", pwm

    set_right_motor_speed(pwm)

def exit():
    # Program will clean up all GPIO settings and terminates
    GPIO.output(leftmotor_in1_pin, False)
    GPIO.output(leftmotor_in2_pin, False)
    GPIO.output(rightmotor_in1_pin, False)
    GPIO.output(rightmotor_in2_pin, False)
    GPIO.cleanup()
