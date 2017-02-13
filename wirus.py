#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import sys
import signal
import logging
import config
from threading import Thread
import ultrasonic_senor
from Adafruit_PWM_Servo_Driver import PWM


class Wirus:
    def __init__(self):
        self.pwm = PWM(0x40)
        self.moving = False
        self.reverse = False
        self.left_motor_speed = 0
        self.right_motor_speed = 0
        self.moving = False
        self.current_step = 0
        self.default_speed_power = 70
        self.left_motor_speed_handicap = 0
        self.right_motor_speed_handicap = 0
        self.time_of_last_wheel_check = time.time()
        self.time_of_last_turn = time.time()
        self.time_of_last_stuck = time.time()

        self.last_turn_right = True
        self.left_motor_rotation = 0
        self.right_motor_rotation = 0
        self.right_motor_rotation_per_min = 0
        self.left_motor_rotation_per_min = 0
        self.detect_running = False

        self.thread = Thread(target=self.wheel_check_thread, args=())
        self.wheel_check_running = True
        self.setup()
        self.main()

    def setup(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        logging.basicConfig(format='%(asctime)s %(message)s', level=logging.INFO)

        GPIO.setmode(GPIO.BCM)
        self.pwm.setPWMFreq(config.PWM_FREQUENCY)
        self.pwm.setPWM(config.PWM_DISTANCE_SERVO_CHANNEL, 0, config.SERVO_HALF)
        time.sleep(5)

        GPIO.setup(config.LEFT_MOTOR_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(config.RIGHT_MOTOR_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(config.LEFT_MOTOR_ENCODER, GPIO.RISING, callback=self.left_motor_encoder_callback)
        GPIO.add_event_detect(config.RIGHT_MOTOR_ENCODER, GPIO.RISING, callback=self.right_motor_encoder_callback)

        GPIO.setup(config.LEFT_EDGE_SENSOR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(config.RIGHT_EDGE_SENSOR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(config.LEFT_EDGE_SENSOR, GPIO.RISING, callback=self.left_edge_detect, bouncetime=100)
        GPIO.add_event_detect(config.RIGHT_EDGE_SENSOR, GPIO.RISING, callback=self.right_edge_detect, bouncetime=100)

        self.set_motor_speed(config.DEFAULT_SPEED)

        logging.info("get ready")
        time.sleep(1)
        self.start_forward()
        time.sleep(.5)

    def left_edge_detect(self, channel):
        if GPIO.input(channel) and not self.detect_running:
            self.detect_running = True
            print "left edge detect"
            self.stop_forward()
            self.backward(config.BACKWARD_TIME)
            self.auto_turn(config.SERVO_MIN)
            if not GPIO.input(config.RIGHT_EDGE_SENSOR) and not GPIO.input(config.LEFT_EDGE_SENSOR):
                self.start_forward()

            self.detect_running = False

    def right_edge_detect(self, channel):
        if GPIO.input(channel) and not self.detect_running:
            self.detect_running = True
            print "right edge detect"
            self.stop_forward()
            self.backward(config.BACKWARD_TIME)
            self.auto_turn(config.SERVO_MAX)

            if not GPIO.input(config.RIGHT_EDGE_SENSOR) and not GPIO.input(config.LEFT_EDGE_SENSOR):
                self.start_forward()

            self.detect_running = False

    def main(self):
        # self.start_wheel_check_thread()
        self.start_forward()

        while True:
            if GPIO.input(config.LEFT_EDGE_SENSOR) or GPIO.input(config.RIGHT_EDGE_SENSOR):
                print "don't move forward"
                self.stop_forward()

            time.sleep(.1)
            #
            # if not self.detect_running:
            #     self.start_forward()

            # print self.detect_running

            if config.SERVO_ON:
                servo = self.next_step()
                self.set_servo(servo)

            # distance = self.get_top_distance()
            #
            # if self.object_detected(distance, servo):
            #     self.stop_forward()
            #     self.pwm.setPWM(config.PWM_DISTANCE_SERVO_CHANNEL, 0, config.SERVO_HALF)
            #     time.sleep(2)
            #         # self.backward(config.BACKWARD_TIME)
            #         # self.auto_turn(servo)
            #         #
            #     # elif self.is_robot_stuck():
            #     #     self.stop_forward()
            #     #     self.get_unstuck()
            #
            # self.detect_running = False

    def get_unstuck(self):
        # if self.get_time_since_last_turn() < TIME_SINCE_LAST_TURN_THRESHOLD and not self.last_turn_right:
            self.backward(2)
            self.rotate_right(1)
        # else:
        #     self.forward(1)
        #     self.rotate_left(1)

    def signal_handler(self, signal, frame):
        logging.info("clean up")
        GPIO.cleanup()
        self.wheel_check_running = False
        self.thread.join()
        sys.exit()

    def start_wheel_check_thread(self):
        self.wheel_check_running = True
        self.thread.start()

    def wheel_check_thread(self):
        while self.wheel_check_running:
            self.wheel_check()

    def wheel_check(self):
        logging.info("running wheel check")

        self.left_motor_rotation_per_min = self.left_motor_rotation * 120
        self.left_motor_rotation = 0

        self.right_motor_rotation_per_min = self.right_motor_rotation * 120
        self.right_motor_rotation = 0

        logging.info("left wheel rotations: %d" % self.left_motor_rotation_per_min)
        logging.info("right wheel rotations: %d" % self.right_motor_rotation_per_min)

        self.time_of_last_wheel_check = time.time()
        self.balance_wheel_speed()

        time.sleep(.5)

    def left_motor_encoder_callback(self, channel):
        logging.debug("left motor callback")
        self.left_motor_rotation += 1

    def right_motor_encoder_callback(self, channel):
        logging.debug("right motor callback")
        self.right_motor_rotation += 1

    def get_time_since_last_turn(self):
        logging.debug("get time since last turn")
        return time.time() - self.time_of_last_turn

    def next_step(self):
        if self.current_step == 0:
            self.reverse = False

        elif self.current_step >= len(config.STEP_POS) - 1:
            self.reverse = True

        if self.reverse:
            self.current_step -= 1
        else:
            self.current_step += 1

        return config.STEP_POS[self.current_step]

    def rotate_left(self, secs):
        logging.debug("rotate left: %d", secs)

        old_left_speed = self.left_motor_speed
        old_right_speed = self.right_motor_speed
        self.set_motor_speed(config.ROTATE_SPEED)

        GPIO.setup(config.RIGHT_MOTOR_FORWARD, GPIO.OUT)
        GPIO.setup(config.LEFT_MOTOR_BACKWARD, GPIO.OUT)
        GPIO.output(config.RIGHT_MOTOR_FORWARD, True)
        GPIO.output(config.LEFT_MOTOR_BACKWARD, True)
        time.sleep(secs)
        GPIO.output(config.RIGHT_MOTOR_FORWARD, False)
        GPIO.output(config.LEFT_MOTOR_BACKWARD, False)

        self.set_left_motor_speed(old_left_speed)
        self.set_right_motor_speed(old_right_speed)

        self.time_of_last_turn = time.time()
        self.last_turn_right = False

    def rotate_right(self, secs):
        logging.debug("rotate right: %d", secs)

        old_left_speed = self.left_motor_speed
        old_right_speed = self.right_motor_speed
        self.set_motor_speed(config.ROTATE_SPEED)

        GPIO.setup(config.RIGHT_MOTOR_BACKWARD, GPIO.OUT)
        GPIO.setup(config.LEFT_MOTOR_FORWARD, GPIO.OUT)
        GPIO.output(config.RIGHT_MOTOR_BACKWARD, True)
        GPIO.output(config.LEFT_MOTOR_FORWARD, True)
        time.sleep(secs)
        GPIO.output(config.RIGHT_MOTOR_BACKWARD, False)
        GPIO.output(config.LEFT_MOTOR_FORWARD, False)

        self.set_left_motor_speed(old_left_speed)
        self.set_right_motor_speed(old_right_speed)

        self.time_of_last_turn = time.time()
        self.last_turn_right = True

    def forward(self, secs):
        logging.debug("move forwards: %d", secs)
        GPIO.setup(config.LEFT_MOTOR_FORWARD, GPIO.OUT)
        GPIO.setup(config.RIGHT_MOTOR_FORWARD, GPIO.OUT)
        GPIO.output(config.LEFT_MOTOR_FORWARD, True)
        GPIO.output(config.RIGHT_MOTOR_FORWARD, True)
        if secs > 0:
            time.sleep(secs)
        GPIO.output(config.LEFT_MOTOR_FORWARD, False)
        GPIO.output(config.RIGHT_MOTOR_FORWARD, False)

    def backward(self, secs):
        logging.debug("move backwards: %d", secs)
        GPIO.setup(config.LEFT_MOTOR_BACKWARD, GPIO.OUT)
        GPIO.setup(config.RIGHT_MOTOR_BACKWARD, GPIO.OUT)
        GPIO.output(config.LEFT_MOTOR_BACKWARD, True)
        GPIO.output(config.RIGHT_MOTOR_BACKWARD, True)
        time.sleep(secs)
        GPIO.output(config.LEFT_MOTOR_BACKWARD, False)
        GPIO.output(config.RIGHT_MOTOR_BACKWARD, False)

    def object_detected(self, distance, servo_position):
        logging.debug("detecting object")
        quarter_amount = (config.SERVO_MAX - config.SERVO_MIN) / 4
        bottom_limit = config.SERVO_HALF - quarter_amount
        top_limit = config.SERVO_HALF + quarter_amount

        if servo_position <= top_limit >= bottom_limit:

            if distance <= config.DISTANCE_THRESHOLD:
                logging.info("detected object in front: %d", distance)
                return True

        elif distance <= config.SIDE_DISTANCE_THRESHOLD:
            logging.info("detected object on side: %d", distance)
            return True

        return False

    def set_motor_speed(self, speed):
        self.set_left_motor_speed(speed)
        self.set_right_motor_speed(speed)

    def set_left_motor_speed(self, speed):
        if self.left_motor_speed != speed and (100 >= speed > 0):
            logging.info("set left speed: %d", speed)
            self.pwm.setPWM(config.PWM_LEFT_MOTOR_SPEED_CHANNEL, 0, int(speed * 40))
            self.left_motor_speed = speed

    def set_right_motor_speed(self, speed):
        if self.right_motor_speed != speed and (100 >= speed > 0):
            logging.info("set right speed: %d", speed)
            self.pwm.setPWM(config.PWM_RIGHT_MOTOR_SPEED_CHANNEL, 0, int(speed * 40))
            self.right_motor_speed = speed

    def distance_to_speed(self, speed):
        logging.debug("distance to speed setting: %d", speed)
        if speed < 80:
            out_speed = 50
        else:
            out_speed = 100

        return out_speed

    def get_top_distance(self):
        logging.debug("getting distance")
        value = ultrasonic_senor.Measurement(config.TRIG, config.ECHO)
        raw_measurement = value.raw_distance(config.SAMPLE_SIZE, config.SAMPLE_WAIT)
        distance = int(value.distance_metric(raw_measurement))
        logging.info("TOP sensor distance %d", distance)
        return distance

    def stop_forward(self):
        if self.moving:
            logging.info("stop forward movement")

            self.moving = False
            GPIO.output(config.LEFT_MOTOR_FORWARD, False)
            GPIO.output(config.RIGHT_MOTOR_FORWARD, False)

    def start_forward(self):
        if not self.moving:
            logging.info("start forward movement")
            self.moving = True

            GPIO.setup(config.LEFT_MOTOR_FORWARD, GPIO.OUT)
            GPIO.setup(config.RIGHT_MOTOR_FORWARD, GPIO.OUT)
            GPIO.output(config.LEFT_MOTOR_FORWARD, True)
            GPIO.output(config.RIGHT_MOTOR_FORWARD, True)

    def is_robot_stuck(self):
        if time.time() - self.time_of_last_stuck > 2:
            if self.left_motor_rotation_per_min < config.WHEEL_MOVEMENT_STUCK_THRESHOLD \
                    or self.right_motor_rotation_per_min < config.WHEEL_MOVEMENT_STUCK_THRESHOLD:
                print "ROBOT STUCK!!"
                self.time_of_last_stuck = time.time()
                return True

        return False

    def balance_wheel_speed(self):
        if self.left_motor_rotation_per_min > self.right_motor_rotation_per_min:
            left_speed = self.left_motor_speed - 1
            right_speed = self.right_motor_speed + 1
            self.set_left_motor_speed(left_speed)
            self.set_right_motor_speed(right_speed)

        elif self.right_motor_rotation_per_min > self.left_motor_rotation_per_min:
            left_speed = self.left_motor_speed + 1
            right_speed = self.right_motor_speed - 1
            self.set_left_motor_speed(left_speed)
            self.set_right_motor_speed(right_speed)

    def auto_turn(self, servo):
        logging.debug("auto turning")

        if servo >= config.SERVO_HALF:
            turn_right = True
        else:
            turn_right = False

        if self.get_time_since_last_turn() < config.TIME_SINCE_LAST_TURN_THRESHOLD \
                and self.last_turn_right != turn_right:
            self.rotate_left(config.ROTATE_TIME)
        elif turn_right:
            self.rotate_right(config.ROTATE_TIME)
        else:
            self.rotate_left(config.ROTATE_TIME)

    def set_servo(self, servo):
        logging.debug("set servo: %d", servo)
        self.pwm.setPWM(config.PWM_DISTANCE_SERVO_CHANNEL, 0, servo)

