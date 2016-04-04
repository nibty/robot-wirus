# PWM
PWM_ADDRESS = 0x40
PWM_FREQUENCY = 60
PWM_DISTANCE_SERVO_CHANNEL = 0
PWM_LEFT_MOTOR_SPEED_CHANNEL = 1
PWM_RIGHT_MOTOR_SPEED_CHANNEL = 2

SERVO_ON = True
SERVO_MAX = 500
SERVO_MIN = 150
SERVO_HALF = 325
STEP_POS = [150, 200, 325, 450, 500]

DISTANCE_THRESHOLD = 25
SIDE_DISTANCE_THRESHOLD = 20
TIME_SINCE_LAST_TURN_THRESHOLD = 5

SAMPLE_SIZE = 4
SAMPLE_WAIT = 0.02

DEFAULT_SPEED_MPM = 15
DEFAULT_SPEED = 70
ROTATE_SPEED = 80

ROTATE_TIME = 0.5
BACKWARD_TIME = 0.5

# sensor1
TRIG = 23
ECHO = 24

# motor1
LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 18
LEFT_MOTOR_ENCODER = 27
LEFT_MOTOR_HANDICAP = 12

# motor2
RIGHT_MOTOR_FORWARD = 12
RIGHT_MOTOR_BACKWARD = 16
RIGHT_MOTOR_ENCODER = 22

WHEEL_MOVEMENT_STUCK_TIME = 1
WHEEL_MOVEMENT_STUCK_THRESHOLD = 400

PULSE_PER_METER = 78.75
