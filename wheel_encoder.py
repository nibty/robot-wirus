import RPi.GPIO as GPIO
import time

RIGHT_MOTOR_ENCODER = 22
LEFT_MOTOR_ENCODER = 27

GPIO.setmode(GPIO.BCM)

left = 0
right = 0

def left_motor_encoder_callback(channel):
    global left
    left += 1

    print "left %d" % left

def right_motor_encoder_callback(channel):
    global right
    right += 1

    print "left %d" % right

GPIO.setup(LEFT_MOTOR_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RIGHT_MOTOR_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(LEFT_MOTOR_ENCODER, GPIO.RISING, callback=left_motor_encoder_callback)
GPIO.add_event_detect(RIGHT_MOTOR_ENCODER, GPIO.RISING, callback=right_motor_encoder_callback)

GPIO.cleanup()

#while True:
#    time.sleep(1)
