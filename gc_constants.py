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

IMU_PORT = "/dev/ttyACM0"
IMU_BAUD_RATE = 9600

LOCALIZE_TARGET_BLOCK_IMAGE_TOP_CUTOFF = 220
LOCALIZE_TARGET_BLOCK_IMAGE_BOTTOM_CUTOFF = 330
LOCALIZE_TARGET_BLOCK_PIXEL_AREA_APPROACHED = 3500
LOCALIZE_TARGET_BLOCK_PIXEL_AREA_POSITIONED = 11000
LOCALIZE_TARGET_BLOCK_MAX_DEGREE_OFFSET_ORIENTED = 6.0

APPROX_CM_PER_TICK = (pi * 6.5 / 20)

EMAIL_SENDER_ADDR = "nickvand.enpm809t@gmail.com"
EMAIL_SENDER_PSWD = "gaxnwemivlqvyxkh"

THRESHOLD_HSV_RED_MIN   = (150,  20,  40)
THRESHOLD_HSV_RED_MAX   = (195, 255, 255)
THRESHOLD_HSV_GREEN_MIN = ( 95, 110,  75)
THRESHOLD_HSV_GREEN_MAX = (135, 180, 160)
THRESHOLD_HSV_BLUE_MIN  = (100,  30,  80)
THRESHOLD_HSV_BLUE_MAX  = (140, 140, 130)

THRESHOLD_HSV_PAIRS = [
    (THRESHOLD_HSV_RED_MIN,   THRESHOLD_HSV_RED_MAX  ),
    (THRESHOLD_HSV_GREEN_MIN, THRESHOLD_HSV_GREEN_MAX),
    (THRESHOLD_HSV_BLUE_MIN,  THRESHOLD_HSV_BLUE_MAX ),
]

THRESHOLD_HSV_INDICES_ORDER = [0, 1, 2, 0, 1, 2, 0, 1, 2]
