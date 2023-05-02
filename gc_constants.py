from math import pi

PIN_MOTOR_FRONT_LEFT_PRI = 31
PIN_MOTOR_FRONT_LEFT_SEC = 33
PIN_MOTOR_BACK_RIGHT_PRI = 37
PIN_MOTOR_BACK_RIGHT_SEC = 35

SLOW_TRANSLATION_MOTOR_PWM_FRONT_LEFT = 19
SLOW_TRANSLATION_MOTOR_PWM_BACK_RIGHT = 22

PIN_ENCODER_FRONT_LEFT = 7
PIN_ENCODER_BACK_RIGHT = 12

PIN_GRIPPER = 36
GRIPPER_FREQ = 50
GRIPPER_DUTY_CYCLE_MIN = 5.5
GRIPPER_DUTY_CYCLE_MAX = 9.5
GRIPPER_INIT_DUTY_CYCLE = 9.5

PIN_DISTANCE_SENSOR_TRIG = 16
PIN_DISTANCE_SENSOR_ECHO = 18
DISTANCE_SENSOR_ROLLING_AVERAGE_SAMPLE_SIZE = 4
DISTANCE_SENSOR_MAX_RELIABLE_READING = 50.0

IMU_PORT = "/dev/ttyACM0"
IMU_BAUD_RATE = 9600

FIND_TARGET_BLOCK_TURN_DEGREES = 10
FIND_TARGET_BLOCK_SLEEP_SECONDS = 0.5

LOCALIZE_TARGET_BLOCK_IMAGE_TOP_CUTOFF = 220
LOCALIZE_TARGET_BLOCK_IMAGE_BOTTOM_CUTOFF = 330
LOCALIZE_TARGET_BLOCK_PIXEL_AREA_APPROACHED = 3500
LOCALIZE_TARGET_BLOCK_PIXEL_AREA_POSITIONED = 11500
LOCALIZE_TARGET_BLOCK_MAX_DEGREE_OFFSET_ORIENTED = 8.0

GET_TO_DROP_POINT_WALL_BUFFER = 30.0
GET_TO_DROP_POINT_MIN_EST_X_CM = 92
GET_TO_DROP_POINT_MIN_EST_Y_CM = 183

RELATIVE_ORIENTATION_TO_FACE_OPPOSITE_WALL = 90.0
RELATIVE_ORIENTATION_TO_FACE_ADJACENT_WALL = 180.0
RELATIVE_ORIENTATION_TO_FACE_AFTER_DROP_POINT = 270.0

STOP_TRANSLATION_SLEEP_SECONDS = 0.8

APPROX_CM_PER_TICK = (pi * 6.5 / 20)

EMAIL_SENDER_ADDR = "nickvand.enpm809t@gmail.com"
EMAIL_SENDER_PSWD = "gaxnwemivlqvyxkh"

THRESHOLD_HSV_RED_MIN   = (150,  20,  40)
THRESHOLD_HSV_RED_MAX   = (195, 255, 255)
THRESHOLD_HSV_GREEN_MIN = ( 55,  45,  90)
THRESHOLD_HSV_GREEN_MAX = (100, 150, 220)
THRESHOLD_HSV_BLUE_MIN  = (100,  30,  75)
THRESHOLD_HSV_BLUE_MAX  = (140, 255, 255)

THRESHOLD_HSV_PAIRS = [
    (THRESHOLD_HSV_RED_MIN,   THRESHOLD_HSV_RED_MAX  ),
    (THRESHOLD_HSV_GREEN_MIN, THRESHOLD_HSV_GREEN_MAX),
    (THRESHOLD_HSV_BLUE_MIN,  THRESHOLD_HSV_BLUE_MAX ),
]

THRESHOLD_HSV_INDICES_ORDER = [0, 1, 2, 0, 1, 2, 0, 1, 2]
