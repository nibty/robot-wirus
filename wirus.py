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
import termios
import tty
import L298NHBridge as HBridge
import os


class Wirus:
    def __init__(self):
        self.pwm = PWM(0x40)
        self.moving = False
        self.reverse = False
        self.left_motor_speed = 0
        self.right_motor_speed = 0
        self.moving = False
        self.current_step = 0
        self.default_speed_power = config.DEFAULT_SPEED
        self.left_motor_speed_handicap = 0
        self.right_motor_speed_handicap = 0
        self.time_of_last_wheel_check = time.time()
        self.time_of_last_turn = time.time()
        self.time_of_last_stuck = time.time()
        self.speed_left = 0
        self.speed_right = 0

        self.last_turn_right = True
        self.left_motor_rotation = 0
        self.right_motor_rotation = 0
        self.right_motor_rotation_per_min = 0
        self.left_motor_rotation_per_min = 0
        self.current_distance = 100

        self.wheel_check_thread = Thread(target=self.wheel_check_thread_function, args=())
        self.keyboard_thread = Thread(target=self.keyboard_thread_function, args=())
        self.autonomous_thread = Thread(target=self.autonomous, args=())
        self.servo_pos = config.SERVO_HALF

        self.keyboard_check_running = False
        self.wheel_check_running = False
        self.autonomous_thread_running = False
        self.autonomous_thread_pause = False

        self.printscreen()
        self.setup()
        time.sleep(1)

        self.start_wheel_check_thread()
        self.start_autonomous_thread()
        self.start_keyboard_thread()

    def setup(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        logging.basicConfig(filename='/var/log/wirus.log', format='%(levelname)s %(asctime)s %(message)s', level=config.LOG_LEVEL)
        GPIO.setmode(GPIO.BCM)
        self.pwm.setPWMFreq(config.PWM_FREQUENCY)
        self.pwm.setPWM(config.PWM_DISTANCE_SERVO_CHANNEL, 0, config.SERVO_HALF)

        GPIO.setup(config.LEFT_MOTOR_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(config.RIGHT_MOTOR_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(config.LEFT_MOTOR_ENCODER, GPIO.RISING, callback=self.left_motor_encoder_callback)
        GPIO.add_event_detect(config.RIGHT_MOTOR_ENCODER, GPIO.RISING, callback=self.right_motor_encoder_callback)

        self.set_motor_speed(config.DEFAULT_SPEED)

    def get_unstuck(self):
        # if self.get_time_since_last_turn() < TIME_SINCE_LAST_TURN_THRESHOLD and not self.last_turn_right:
            self.backward(config.BACKWARD_TIME * 2)
            self.rotate_right(1)
        # else:
        #     self.forward(1)
        #     self.rotate_left(1)

    def signal_handler(self, signal, frame):
        logging.info("clean up")
        GPIO.cleanup()
        self.exit()

    def exit(self):
        self.wheel_check_running = False
        self.keyboard_check_running = False
        self.autonomous_thread_running = False

        try:
            self.autonomous_thread.join()
            self.keyboard_thread.join()
            self.wheel_check_thread.join()
        except RuntimeError:
            pass

        GPIO.cleanup()
        sys.exit()

    def start_wheel_check_thread(self):
        self.wheel_check_running = True
        self.wheel_check_thread.start()

    def start_keyboard_thread(self):
        self.keyboard_check_running = True
        self.keyboard_thread.start()

    def keyboard_thread_function(self):
        while self.keyboard_check_running:
            self.keyboard_check()

    def wheel_check_thread_function(self):
        while self.wheel_check_running:
            if self.autonomous_thread_pause:
                time.sleep(.1)
                continue

            self.wheel_check()

    def keyboard_check(self):
        while True:
            char = self.getch()

            if char == "x" or char == "z":
                self.exit()

            elif char == "q":
                self.stop()

            elif char == "w":
                self.forwards()

            elif char == "s":
                self.backwards()

            elif char == "d":
                self.right()

            elif char == "a":
                self.left()

            elif char == "i":
                self.sensor_left()

            elif char == "o":
                self.sensor_center()

            elif char == "p":
                self.sensor_right()

            elif char == "e" and self.autonomous_thread_pause:
                self.start_autonomous()

            elif char == "e":
                self.pause_autonomous()

    def forwards(self):
        if not self.autonomous_thread_pause:
            self.pause_autonomous()

        if self.speed_left > self.speed_right:
            self.speed_right = self.speed_left
        else:
            self.speed_left = self.speed_right

        self.speed_left += 0.1
        self.speed_right += 0.1

        if self.speed_left > 1:
            self.speed_left = 1

        if self.speed_right > 1:
            self.speed_right = 1

        HBridge.setMotorLeft(self.speed_left)
        HBridge.setMotorRight(self.speed_right)
        self.printscreen()

    def backwards(self):
        if not self.autonomous_thread_pause:
            self.pause_autonomous()

        if self.speed_left < self.speed_right:
            self.speed_right = self.speed_left
        else:
            self.speed_left = self.speed_right

        self.speed_left -= 0.1
        self.speed_right -= 0.1

        if self.speed_left < -1:
            self.speed_left = -1
        if self.speed_right < -1:
            self.speed_right = -1

        HBridge.setMotorLeft(self.speed_left)
        HBridge.setMotorRight(self.speed_right)
        self.printscreen()

    def right(self):
        if not self.autonomous_thread_pause:
            self.pause_autonomous()

        self.speed_right -= 0.1
        self.speed_left += 0.1

        if self.speed_right < -1:
            self.speed_right = -1

        if self.speed_left > 1:
            self.speed_left = 1

        HBridge.setMotorLeft(self.speed_left)
        HBridge.setMotorRight(self.speed_right)
        self.printscreen()

    def left(self):
        if not self.autonomous_thread_pause:
            self.pause_autonomous()

        self.speed_left -= 0.1
        self.speed_right += 0.1

        if self.speed_left < -1:
            self.speed_left = -1

        if self.speed_right > 1:
            self.speed_right = 1

        HBridge.setMotorLeft(self.speed_left)
        HBridge.setMotorRight(self.speed_right)
        self.printscreen()

    def stop(self):
        if not self.autonomous_thread_pause:
            self.pause_autonomous()

        self.speed_left = 0
        self.speed_right = 0
        HBridge.setMotorLeft(0)
        HBridge.setMotorRight(0)
        self.printscreen()

    def sensor_right(self):
        if not self.autonomous_thread_pause:
            self.pause_autonomous()
        self.set_servo(config.STEP_POS[0])
        self.current_distance = self.get_top_distance()
        self.printscreen()

    def sensor_left(self):
        if not self.autonomous_thread_pause:
            self.pause_autonomous()
        self.set_servo(config.STEP_POS[4])
        self.current_distance = self.get_top_distance()
        self.printscreen()

    def sensor_center(self):
        if not self.autonomous_thread_pause:
            self.pause_autonomous()
        self.set_servo(config.STEP_POS[2])
        self.current_distance = self.get_top_distance()

        self.printscreen()

    def start_autonomous(self):
        logging.info("Start autonomous mode")
        self.stop()
        self.start_forward()
        self.autonomous_thread_pause = False
        self.printscreen()

    def pause_autonomous(self):
        logging.info("Pause autonomous mode")
        self.stop_forward()
        self.autonomous_thread_pause = True
        self.printscreen()

    def start_autonomous_thread(self):
        self.autonomous_thread_running = True
        self.autonomous_thread.start()

    def autonomous(self):
        while self.autonomous_thread_running:
            if self.autonomous_thread_pause:
                time.sleep(.1)
                self.time_of_last_stuck = time.time()
                continue

            self.start_forward()

            if config.SERVO_ON:
                servo = self.next_step()
                self.set_servo(servo)

            self.current_distance = self.get_top_distance()

            if self.object_detected(self.current_distance, servo):
                self.stop_forward()
                self.backward(config.BACKWARD_TIME)
                self.auto_turn(servo)

            elif self.is_robot_stuck():
                self.stop_forward()
                self.get_unstuck()

    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def wheel_check(self):
        logging.debug("running wheel check")

        self.left_motor_rotation_per_min = self.left_motor_rotation * 120
        self.left_motor_rotation = 0

        self.right_motor_rotation_per_min = self.right_motor_rotation * 120
        self.right_motor_rotation = 0

        logging.debug("left wheel rotations: %d" % self.left_motor_rotation_per_min)
        logging.debug("right wheel rotations: %d" % self.right_motor_rotation_per_min)

        self.time_of_last_wheel_check = time.time()
        self.balance_wheel_speed()
        time.sleep(.5)

    def left_motor_encoder_callback(self, channel):
        self.left_motor_rotation += 1

    def right_motor_encoder_callback(self, channel):
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
            logging.debug("set left speed: %d", speed)
            self.pwm.setPWM(config.PWM_LEFT_MOTOR_SPEED_CHANNEL, 0, int(speed * 40))
            self.left_motor_speed = speed

    def set_right_motor_speed(self, speed):
        if self.right_motor_speed != speed and (100 >= speed > 0):
            logging.debug("set right speed: %d", speed)
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
        logging.debug("TOP sensor distance %d", distance)
        return distance

    def stop_forward(self):
        if self.moving:
            logging.debug("stop forward movement")

            self.moving = False
            GPIO.output(config.LEFT_MOTOR_FORWARD, False)
            GPIO.output(config.RIGHT_MOTOR_FORWARD, False)

    def start_forward(self):
        if not self.moving:
            logging.debug("start forward movement")
            self.moving = True

            GPIO.setup(config.LEFT_MOTOR_FORWARD, GPIO.OUT)
            GPIO.setup(config.RIGHT_MOTOR_FORWARD, GPIO.OUT)
            GPIO.output(config.LEFT_MOTOR_FORWARD, True)
            GPIO.output(config.RIGHT_MOTOR_FORWARD, True)

    def is_robot_stuck(self):
        if time.time() - self.time_of_last_stuck > config.WHEEL_MOVEMENT_STUCK_TIME:
            if self.left_motor_rotation_per_min < config.WHEEL_MOVEMENT_STUCK_THRESHOLD \
                    or self.right_motor_rotation_per_min < config.WHEEL_MOVEMENT_STUCK_THRESHOLD:
                logging.error("robot stuck")
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
        self.servo_pos = servo
        self.pwm.setPWM(config.PWM_DISTANCE_SERVO_CHANNEL, 0, servo)

    def printscreen(self):
        # Print the motor speed just for interest
        os.system('clear')
        if not self.autonomous_thread_pause:
            print "******* autonomous mode ********"
            print ""

        print("i/o/p: distance sensor position: ", self.servo_pos, ", distance: ", self.current_distance)
        print("w/s: direction")
        print("a/d: steering")
        print("q: stops the motors")
        print("e: autonomous mode")
        print("x: exit")
        print("========== Speed Control ==========")
        print "left motor:  ", self.speed_left
        print "right motor: ", self.speed_right