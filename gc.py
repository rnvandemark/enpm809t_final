#! /usr/bin/python3

from sys import argv as sargv
from os import mkdir
from os.path import join as ojoin
import RPi.GPIO as gpio
from numpy import uint64, pi, deg2rad, rad2deg
import cv2
from math import sin, cos
from time import sleep
from datetime import datetime
import matplotlib.pyplot as plt
from threading import Thread

from bufferless_imu import BufferlessImu
from bufferless_video_capture import BufferlessVideoCapture
from gc_constants import *
from image_analysis import *
from email_tools import send_images

start_timestamp = datetime.now().strftime("%Y%m%d%H%M%S")

class Gripper(object):
    def __init__(self, init_dc):
        gpio.setup(PIN_GRIPPER, gpio.OUT)
        self.pwm = gpio.PWM(PIN_GRIPPER, GRIPPER_FREQ)
        self.set_duty_cycle(init_dc, first=True)

    def close(self, final_dc):
        self.set_duty_cycle(final_dc)
        sleep(2)
        self.pwm.stop()

    def set_duty_cycle(self, dc, first=False):
        fixed_dc = dc
        if fixed_dc > GRIPPER_DUTY_CYCLE_MAX:
            fixed_dc = GRIPPER_DUTY_CYCLE_MAX
        elif fixed_dc < GRIPPER_DUTY_CYCLE_MIN:
            fixed_dc = GRIPPER_DUTY_CYCLE_MIN

        if first:
            self.pwm.start(fixed_dc)
        else:
            self.pwm.ChangeDutyCycle(fixed_dc)

        return fixed_dc

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(PIN_MOTOR_FRONT_LEFT_PRI, gpio.OUT)
    gpio.setup(PIN_MOTOR_FRONT_LEFT_SEC, gpio.OUT)
    gpio.setup(PIN_MOTOR_BACK_RIGHT_SEC, gpio.OUT)
    gpio.setup(PIN_MOTOR_BACK_RIGHT_PRI, gpio.OUT)
    gpio.setup(PIN_ENCODER_FRONT_LEFT, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(PIN_ENCODER_BACK_RIGHT, gpio.IN, pull_up_down=gpio.PUD_UP)

def move(v1, v2, v3, v4):
    gpio.output(PIN_MOTOR_FRONT_LEFT_PRI, v1)
    gpio.output(PIN_MOTOR_FRONT_LEFT_SEC, v2)
    gpio.output(PIN_MOTOR_BACK_RIGHT_SEC, v3)
    gpio.output(PIN_MOTOR_BACK_RIGHT_PRI, v4)

def stop():
    move(False, False, False, False)

def gameover():
    stop()
    gpio.cleanup()

def forward():
    move(True, False, False, True)

def reverse():
    move(False, True, True, False)

def pivot_right():
    move(True, False, True, False)

def pivot_left():
    move(False, True, False, True)

def handle_turn_using_encoders(desi_input):
    # d=0.145m between each wheel
    FF = 1.0
    desi_dist_cm = pi * 14.5 * (abs(desi_input)/360) * FF

    desi_wheel_revs = desi_dist_cm / (pi * 6.5)
    desi_enc_ticks = 20 * desi_wheel_revs
    print("Turn: {0}d ({1}m arc) -> {2} wheel revs -> {3} encoder ticks".format(
        desi_input,
        desi_dist_cm / 100.0,
        desi_wheel_revs,
        desi_enc_ticks
    ))

    if desi_enc_ticks <= 1:
        return True

    is_positive_rotation = (desi_input >= 0.0)
    move_func = (pivot_left if is_positive_rotation else pivot_right)

    new_fl = False
    new_br = False
    counter_fl = uint64(0)
    button_fl = int(gpio.input(PIN_ENCODER_FRONT_LEFT))
    counter_br = uint64(0)
    button_br = int(gpio.input(PIN_ENCODER_BACK_RIGHT))

    try:
        i = 0
        move_func()
        while True:
            signal_fl = int(gpio.input(PIN_ENCODER_FRONT_LEFT))
            signal_br = int(gpio.input(PIN_ENCODER_BACK_RIGHT))

            new_fl = False
            new_br = False
            if signal_fl != button_fl:
                button_fl = signal_fl
                counter_fl += 1
                new_fl = True
            if signal_br != button_br:
                button_br = signal_br
                counter_br += 1
                new_br = True

#            if new_fl or new_br:
#                print("fl=[c={0},s={1}], br=[c={2},s={3}]".format(
#                    counter_fl,
#                    signal_fl,
#                    counter_br,
#                    signal_br
#                ))

            if new_fl and (counter_fl >= desi_enc_ticks):
                print("- FL finished at {0}!".format(i))
                break
            if new_br and (counter_br >= desi_enc_ticks):
                print("- BR finished at {0}!".format(i))
                break
            i += 1

    except KeyboardInterrupt:
        print("User executed SIGTERM")
    except Exception as e:
        print("Exception: " + str(e))

    stop()

    return False

def handle_turn_using_imu(desi_input, imu):
    is_positive_rotation = (desi_input >= 0.0)
    move_func = pivot_left if is_positive_rotation else pivot_right

    init_orientation = imu.read()
    desi_orientation_change = deg2rad(desi_input)
    orientation_change = 0.0
    print("{0} degs to {1} degs".format(
        rad2deg(init_orientation),
        rad2deg(init_orientation + desi_orientation_change)
    ))

    move_func()

    while abs(orientation_change) < abs(desi_orientation_change):
        curr_orientation = imu.read()
        if is_positive_rotation and ((curr_orientation + deg2rad(0.5)) < init_orientation):
            curr_orientation += (2 * pi)
        elif (not is_positive_rotation) and ((curr_orientation - deg2rad(0.5)) > init_orientation):
            curr_orientation -= (2 * pi)
        orientation_change = curr_orientation - init_orientation
        print("orientation change: {0} degs (of {1} degs)".format(
            rad2deg(orientation_change),
            rad2deg(desi_orientation_change),
        ))
        sleep(0.01)

    stop()

    return False

def handle_translation(move_func, desi_input, imu, x_i, y_i):
    desi_wheel_revs = desi_input / (pi * 6.5)
    desi_enc_ticks = 20 * desi_wheel_revs
    print("Translation: {0}m -> {1} wheel revs -> {2} encoder ticks".format(
        desi_input / 100.0,
        desi_wheel_revs,
        desi_enc_ticks
    ))

    desi_enc_ticks = max(desi_enc_ticks, 2)

    data_x = []
    data_y = []
    x_f, y_f = x_i, y_i

    new_fl = False
    new_br = False
    counter_fl = uint64(0)
    button_fl = int(gpio.input(PIN_ENCODER_FRONT_LEFT))
    counter_br = uint64(0)
    button_br = int(gpio.input(PIN_ENCODER_BACK_RIGHT))

    try:
        i = 0
        move_func()
        while True:
            signal_fl = int(gpio.input(PIN_ENCODER_FRONT_LEFT))
            signal_br = int(gpio.input(PIN_ENCODER_BACK_RIGHT))

            new_fl = False
            new_br = False
            if signal_fl != button_fl:
                button_fl = signal_fl
                counter_fl += 1
                new_fl = True
            if signal_br != button_br:
                button_br = signal_br
                counter_br += 1
                new_br = True

            if new_fl or new_br:
#                print("fl=[c={0},s={1}], br=[c={2},s={3}]".format(
#                    counter_fl,
#                    signal_fl,
#                    counter_br,
#                    signal_br
#                ))
                orientation_rads = imu.read()
                x_f += (APPROX_CM_PER_TICK * cos(orientation_rads))
                y_f += (APPROX_CM_PER_TICK * sin(orientation_rads))
                data_x.append(x_f)
                data_y.append(y_f)

            if new_fl and (counter_fl >= desi_enc_ticks):
                print("- FL finished at {0}!".format(i))
                break
            if new_br and (counter_br >= desi_enc_ticks):
                print("- BR finished at {0}!".format(i))
                break
            i += 1

    except KeyboardInterrupt:
        print("User executed SIGTERM")
    except Exception as e:
        print("Exception: " + str(e))

    stop()

    return data_x, data_y, x_f, y_f

def get_to_next_block_pick_point(cap, hsv_threshold_pair_idx, imu, x_i, y_i):
    target_hsv_min, target_hsv_max = THRESHOLD_HSV_PAIRS[hsv_threshold_pair_idx]

    positioned = False
    found_block = True
    fast_translating = False
    fine_tune_translating = False
    bgr_image = None

    pwm_fl = None
    pwm_br = None

    x_f = x_i
    y_f = y_i
    data_x = []
    data_y = []

    translation_monitoring_thread = None
    continue_monitoring_translation = False

    def monitor_translation():
        nonlocal continue_monitoring_translation, \
            x_f, y_f, data_x, data_y

        new_fl = False
        new_br = False
        counter_fl = uint64(0)
        button_fl = int(gpio.input(PIN_ENCODER_FRONT_LEFT))
        counter_br = uint64(0)
        button_br = int(gpio.input(PIN_ENCODER_BACK_RIGHT))

        while continue_monitoring_translation:
            signal_fl = int(gpio.input(PIN_ENCODER_FRONT_LEFT))
            signal_br = int(gpio.input(PIN_ENCODER_BACK_RIGHT))

            new_fl = False
            new_br = False
            if signal_fl != button_fl:
                button_fl = signal_fl
                counter_fl += 1
                new_fl = True
            if signal_br != button_br:
                button_br = signal_br
                counter_br += 1
                new_br = True

            if new_fl or new_br:
                orientation_rads = imu.read()
                x_f += (APPROX_CM_PER_TICK * cos(orientation_rads))
                y_f += (APPROX_CM_PER_TICK * sin(orientation_rads))
                data_x.append(x_f)
                data_y.append(y_f)

    def start_translation_monitoring_thread():
        nonlocal continue_monitoring_translation, \
            translation_monitoring_thread
        continue_monitoring_translation = True
        translation_monitoring_thread = Thread(target=monitor_translation)
        translation_monitoring_thread.start()

    def stop_translation_monitoring_thread():
        nonlocal continue_monitoring_translation, \
            translation_monitoring_thread
        continue_monitoring_translation = False
        translation_monitoring_thread.join()
        translation_monitoring_thread = None

    def start_fine_tune_translation():
        nonlocal pwm_fl, pwm_br, fine_tune_translating
        start_translation_monitoring_thread()
        pwm_fl = gpio.PWM(PIN_MOTOR_FRONT_LEFT_PRI, 50)
        pwm_br = gpio.PWM(PIN_MOTOR_BACK_RIGHT_PRI, 50)
        pwm_fl.start(SLOW_TRANSLATION_MOTOR_PWM_FRONT_LEFT)
        pwm_br.start(SLOW_TRANSLATION_MOTOR_PWM_BACK_RIGHT)
        fine_tune_translating = True

    def stop_fine_tune_translation():
        nonlocal pwm_fl, pwm_br, fine_tune_translating
        pwm_fl.stop()
        pwm_br.stop()
        sleep(0.8)
        pwm_fl = None
        pwm_br = None
        fine_tune_translating = False
        stop_translation_monitoring_thread()

    def start_fast_translation():
        nonlocal fast_translating
        start_translation_monitoring_thread()
        forward()
        fast_translating = True

    def stop_fast_translation():
        nonlocal fast_translating
        stop()
        sleep(0.8)
        fast_translating = False
        stop_translation_monitoring_thread()

    while found_block and (not positioned):
        bgr_image = cap.read()
        bgr_image, found_block, block_pixel_area, degrees_away = find_target_block(bgr_image, target_hsv_min, target_hsv_max)
#        print("{0} | {1}".format(block_pixel_area, degrees_away))
        cv2.imshow("Found block", bgr_image)
        print(block_pixel_area)
        if found_block:
            oriented = (abs(degrees_away) < FIND_TARGET_BLOCK_MAX_DEGREE_OFFSET_ORIENTED)
            if oriented:
#                print("Oriented!")
                positioned = (block_pixel_area >= FIND_TARGET_BLOCK_PIXEL_AREA_POSITIONED)
                if positioned:
                    print("Positioned!")
                else:
                    approached = (block_pixel_area >= FIND_TARGET_BLOCK_PIXEL_AREA_APPROACHED)
                    if approached:
#                        print("Approached!")
                        if not fine_tune_translating:
                            if fast_translating:
                                stop_fast_translation()
                                fast_translating = False
                            start_fine_tune_translation()
                    elif not fast_translating:
                        start_fast_translation()
                        fast_translating = True
            else:
                if fine_tune_translating:
                    stop_fine_tune_translation()
                if fast_translating:
                    stop_fast_translation()
                handle_turn_using_imu(degrees_away, imu)

        if 27 == (cv2.waitKey(1) & 0xEFFFFF):
            break

    if fine_tune_translating:
        stop_fine_tune_translation()
    if fast_translating:
        stop_fast_translation()

    return bgr_image, data_x, data_y, x_f, y_f

if "__main__" == __name__:
    # Capture default camera
    cap = BufferlessVideoCapture(0)

#    # Create video writer
#    writer = cv2.VideoWriter(
#        "HW9.mp4",
#        cv2.VideoWriter_fourcc(*'DIVX'),
#        20,
#        (int(cap.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
#    )

    # Create interface to IMU
    imu = BufferlessImu(IMU_PORT, IMU_BAUD_RATE)

    # Init motor control
    init()

    # Init gripper
    gripper = Gripper(GRIPPER_INIT_DUTY_CYCLE)
    sleep(2)

    # The name of the images directory to write images to
    image_dir = ojoin("images", start_timestamp)
    mkdir(image_dir)

    x_i = 0.0
    y_i = 0.0

    data_x = []
    data_y = []

    for i, hsv_threshold_pair_idx in enumerate(THRESHOLD_HSV_INDICES_ORDER):
        block_image_path = ojoin(image_dir, "block_{0}.jpg".format(i))

        grabbed = False
        try:
            positioned_image, new_data_x, new_data_y, x_f, y_f = get_to_next_block_pick_point(cap, 0, imu, x_i, y_i)
            gripper.set_duty_cycle(GRIPPER_DUTY_CYCLE_MIN)
            sleep(2)
            print("Motion: [{0}, {1}] -> [{2}, {3}]".format(x_i, y_i, x_f, y_f))
            data_x.extend(new_data_x)
            data_y.extend(new_data_y)
            cv2.imshow("Found block", positioned_image)
            cv2.imwrite(block_image_path, positioned_image)
            grabbed = True
        except KeyboardInterrupt:
            print("User executed SIGTERM")

        trajectory_image_path = ojoin(image_dir, "trajectory.jpg")
        plt.plot(data_x, data_y)
        plt.title("Robot Trajectory")
        plt.xlabel("x(t) (cm)")
        plt.ylabel("y(t) (cm)")
        plt.savefig(trajectory_image_path, bbox_inches="tight")

        if grabbed:
            send_images(
                ["rnvandemark@gmail.com"],
                [],
                snapshot_locations=[
                    block_image_path,
                    trajectory_image_path,
                ]
            )

    gripper.close(GRIPPER_INIT_DUTY_CYCLE)
    gameover()
    cap.release()
#    writer.release()
    cv2.destroyAllWindows()
