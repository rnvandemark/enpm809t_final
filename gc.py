#! /usr/bin/python3

from sys import argv as sargv
from os import mkdir
from os.path import join as ojoin
import RPi.GPIO as gpio
from numpy import uint64, pi, deg2rad, rad2deg
import cv2
from math import sin, cos
from time import sleep, time
from datetime import datetime
import matplotlib.pyplot as plt
from threading import Thread, Lock
from queue import Queue

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

class BufferlessDistanceSensor(object):
    def __init__(self):
        gpio.setup(PIN_DISTANCE_SENSOR_TRIG, gpio.OUT)
        gpio.setup(PIN_DISTANCE_SENSOR_ECHO, gpio.IN)
        self.sample_queue = Queue()
        self.sample_sum = 0.0
        self.lock = Lock()
        t = Thread(target=self.run)
        t.daemon = True
        t.start()

    def run(self):
        while True:
            gpio.output(PIN_DISTANCE_SENSOR_TRIG, False)
            sleep(0.01)

            gpio.output(PIN_DISTANCE_SENSOR_TRIG, True)
            sleep(0.00001)
            gpio.output(PIN_DISTANCE_SENSOR_TRIG, False)

            pulse_start = time()
            while gpio.input(PIN_DISTANCE_SENSOR_ECHO) == 0:
                pulse_start = time()

            while gpio.input(PIN_DISTANCE_SENSOR_ECHO) == 1:
                pulse_end = time()

            new_distance = min(round((pulse_end-pulse_start) * 17150, 2), DISTANCE_SENSOR_MAX_RELIABLE_READING)
            old_distance = None
            if self.sample_queue.qsize() >= DISTANCE_SENSOR_ROLLING_AVERAGE_SAMPLE_SIZE:
                try:
                    old_distance = self.sample_queue.get_nowait()
                except Queue.Empty:
                    pass
            self.sample_queue.put(new_distance)
            with self.lock:
                self.sample_sum += new_distance
                if old_distance is not None:
                    self.sample_sum -= old_distance
#            print("Avg dist: " + str(self.read()))

    def read(self):
        with self.lock:
            return self.sample_sum / DISTANCE_SENSOR_ROLLING_AVERAGE_SAMPLE_SIZE
        return None

class TranslationWrapper(object):
    def __init__(self):
        self.x_f = None
        self.y_f = None
        self.data_x = None
        self.data_y = None

        self.pwm_fl = None
        self.pwm_br = None

        self.fast_translating = False
        self.fine_tune_translating = False

        self.translation_tracking_thread = None
        self.continue_tracking_translation = False

        self.lock = Lock()

    def start_translation_tracking_thread(self, imu, x_i, y_i):
        self.x_f = x_i
        self.y_f = y_i
        self.data_x = []
        self.data_y = []
        self.continue_tracking_translation = True
        self.translation_tracking_thread = Thread(target=self.track_translation, args=(imu,))
        self.translation_tracking_thread.start()

    def stop_translation_tracking_thread(self):
        self.continue_tracking_translation = False
        self.translation_tracking_thread.join()
        self.translation_tracking_thread = None
        return self.data_x, self.data_y, self.x_f, self.y_f

    def start_fine_tune_translation(self, imu, x_i, y_i):
        print("STARTING fine tune translation")
        self.start_translation_tracking_thread(imu, x_i, y_i)
        self.pwm_fl = gpio.PWM(PIN_MOTOR_FRONT_LEFT_PRI, 50)
        self.pwm_br = gpio.PWM(PIN_MOTOR_BACK_RIGHT_PRI, 50)
        self.pwm_fl.start(SLOW_TRANSLATION_MOTOR_PWM_FRONT_LEFT)
        self.pwm_br.start(SLOW_TRANSLATION_MOTOR_PWM_BACK_RIGHT)
        self.fine_tune_translating = True

    def stop_fine_tune_translation(self):
        print("STOPPING fine tune translation")
        self.pwm_fl.stop()
        self.pwm_br.stop()
        sleep(STOP_TRANSLATION_SLEEP_SECONDS)
        self.pwm_fl = None
        self.pwm_br = None
        self.fine_tune_translating = False
        return self.stop_translation_tracking_thread()

    def start_fast_translation(self, move_func, imu, x_i, y_i):
        print("STARTING fast translation")
        self.start_translation_tracking_thread(imu, x_i, y_i)
        move_func()
        self.fast_translating = True

    def stop_fast_translation(self, stop_func):
        print("STOPPING fast translation")
        stop_func()
        sleep(STOP_TRANSLATION_SLEEP_SECONDS)
        self.fast_translating = False
        return self.stop_translation_tracking_thread()

    def track_translation(self, imu):
        new_fl = False
        new_br = False
        counter_fl = uint64(0)
        button_fl = int(gpio.input(PIN_ENCODER_FRONT_LEFT))
        counter_br = uint64(0)
        button_br = int(gpio.input(PIN_ENCODER_BACK_RIGHT))

        while self.continue_tracking_translation:
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
                with self.lock:
                    self.x_f += (APPROX_CM_PER_TICK * cos(orientation_rads) / 2.0)
                    self.y_f += (APPROX_CM_PER_TICK * sin(orientation_rads) / 2.0)
                self.data_x.append(self.x_f)
                self.data_y.append(self.y_f)

    def get_current_displacement(self):
        with self.lock:
            return (self.x_f, self.y_f)
        return None

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

def handle_set_orientation_using_imu(desi_orientation, imu):
    curr_orientation = rad2deg(imu.read())
    desi_input = desi_orientation - rad2deg(imu.read())
#    print("desi={0}, curr={1}, diff={2}".format(desi_orientation, curr_orientation, desi_input))
    if desi_input > 180:
        desi_input -= 360
    elif desi_input < -180:
        desi_input += 360
    handle_turn_using_imu(desi_input, imu)

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

def find_next_block(cap, hsv_threshold_pair_idx, imu):
    target_hsv_min, target_hsv_max = THRESHOLD_HSV_PAIRS[hsv_threshold_pair_idx]

    found_block = False
    while not found_block:
        handle_turn_using_imu(FIND_TARGET_BLOCK_TURN_DEGREES, imu)
        sleep(FIND_TARGET_BLOCK_SLEEP_SECONDS)
        found_block = find_target_block(cap.read(), target_hsv_min, target_hsv_max)

def get_to_next_block_pick_point(cap, hsv_threshold_pair_idx, imu, x_i, y_i):
    target_hsv_min, target_hsv_max = THRESHOLD_HSV_PAIRS[hsv_threshold_pair_idx]

    positioned = False
    found_block = True
    bgr_image = None
    approached_start = None

    x_f = x_i
    y_f = y_i
    data_x = []
    data_y = []

    translation_wrapper = TranslationWrapper()

    def consume_translation_data(data):
        nonlocal data_x, data_y, x_f, y_f
        new_data_x, new_data_y, x_f, y_f = data
        data_x.extend(new_data_x)
        data_y.extend(new_data_y)

    while found_block and (not positioned):
        bgr_image = cap.read()
        bgr_image, found_block, block_pixel_area, degrees_away = localize_target_block(bgr_image, target_hsv_min, target_hsv_max)
#        print("{0} | {1}".format(block_pixel_area, degrees_away))
        cv2.imshow("Found block", bgr_image)
        if found_block:
            oriented = (abs(degrees_away) < LOCALIZE_TARGET_BLOCK_MAX_DEGREE_OFFSET_ORIENTED)
            if oriented:
#                print("Oriented!")
                positioned = (block_pixel_area >= LOCALIZE_TARGET_BLOCK_PIXEL_AREA_POSITIONED)
                if positioned:
                    print("Positioned!")
                else if (approached_start is not None) and ((time() - approached_start) >= LOCALIZE_TARGET_BLOCK_SLOW_TRANSLATION_TIMEOUT_SECONDS):
                    print("Approach timeout reached, assuming positioned!")
                    positioned = True
                else:
                    approached = (block_pixel_area >= LOCALIZE_TARGET_BLOCK_PIXEL_AREA_APPROACHED)
                    if approached:
#                        print("Approached!")
                        if not translation_wrapper.fine_tune_translating:
                            if translation_wrapper.fast_translating:
                                consume_translation_data(translation_wrapper.stop_fast_translation(stop))
                            translation_wrapper.start_fine_tune_translation(imu, x_f, y_f)
                        if approached_start is None:
                            approached_start = time()
                    else:
                        approached_start = None
                        if not translation_wrapper.fast_translating:
                            translation_wrapper.start_fast_translation(forward, imu, x_f, y_f)
            else:
                approached_start = None
                if translation_wrapper.fine_tune_translating:
                    consume_translation_data(translation_wrapper.stop_fine_tune_translation())
                if translation_wrapper.fast_translating:
                    consume_translation_data(translation_wrapper.stop_fast_translation(stop))
                handle_turn_using_imu(degrees_away, imu)

        if 27 == (cv2.waitKey(1) & 0xEFFFFF):
            break

    if translation_wrapper.fine_tune_translating:
        consume_translation_data(translation_wrapper.stop_fine_tune_translation())
    if translation_wrapper.fast_translating:
        consume_translation_data(translation_wrapper.stop_fast_translation(stop))

    return positioned, bgr_image, data_x, data_y, x_f, y_f

def change_block_grip(gripper, close):
    gripper.set_duty_cycle(GRIPPER_DUTY_CYCLE_MIN if close else GRIPPER_DUTY_CYCLE_MAX)
    sleep(1)

def get_to_next_block_drop_point(distance_sensor, imu, x_i, y_i):
    x_f = x_i
    y_f = y_i
    data_x = []
    data_y = []

    translation_wrapper = TranslationWrapper()

    def consume_translation_data(data):
        nonlocal data_x, data_y, x_f, y_f
        new_data_x, new_data_y, x_f, y_f = data
        data_x.extend(new_data_x)
        data_y.extend(new_data_y)

    handle_set_orientation_using_imu(RELATIVE_ORIENTATION_TO_FACE_OPPOSITE_WALL, imu)
    sleep(1.0)

    est_x, est_y = x_f, y_f
    if (est_y < GET_TO_DROP_POINT_MIN_EST_Y_CM) or (distance_sensor.read() > GET_TO_DROP_POINT_WALL_BUFFER):
        translation_wrapper.start_fast_translation(forward, imu, x_f, y_f)
        while (est_y < GET_TO_DROP_POINT_MIN_EST_Y_CM) or (distance_sensor.read() > GET_TO_DROP_POINT_WALL_BUFFER):
            sleep(0.02)
            est_x, est_y = translation_wrapper.get_current_displacement()
        consume_translation_data(translation_wrapper.stop_fast_translation(stop))
        sleep(1.0)

    handle_set_orientation_using_imu(RELATIVE_ORIENTATION_TO_FACE_ADJACENT_WALL, imu)
    sleep(1.0)

    if (est_x > GET_TO_DROP_POINT_MIN_EST_X_CM) or (distance_sensor.read() > GET_TO_DROP_POINT_WALL_BUFFER):
        translation_wrapper.start_fast_translation(forward, imu, x_f, y_f)
        while (est_x > GET_TO_DROP_POINT_MIN_EST_X_CM) or (distance_sensor.read() > GET_TO_DROP_POINT_WALL_BUFFER):
            sleep(0.02)
            est_x, est_y = translation_wrapper.get_current_displacement()
        consume_translation_data(translation_wrapper.stop_fast_translation(stop))

    return data_x, data_y, x_f, y_f

def back_off_block_drop_point(imu, x_i, y_i):
    translation_wrapper = TranslationWrapper()
    translation_wrapper.start_fast_translation(reverse, imu, x_i, y_i)
    sleep(1.0)
    data_x, data_y, x_f, y_f = translation_wrapper.stop_fast_translation(stop)
    sleep(1.0)
    handle_set_orientation_using_imu(RELATIVE_ORIENTATION_TO_FACE_AFTER_DROP_POINT, imu)
    return data_x, data_y, x_f, y_f

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

    # Init distance sensor
    distance_sensor = BufferlessDistanceSensor()

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

    image_paths = []
    for i, hsv_threshold_pair_idx in enumerate(THRESHOLD_HSV_INDICES_ORDER):
        try:
            positioned = False
            positioned_image = None
            new_data_x = None
            new_data_y = None
            x_f = None
            y_f = None

            while not positioned:
                find_next_block(cap, hsv_threshold_pair_idx, imu)
                positioned, positioned_image, new_data_x, new_data_y, x_f, y_f = get_to_next_block_pick_point(cap, hsv_threshold_pair_idx, imu, x_i, y_i)

            cv2.imshow("Found block", positioned_image)
            block_image_path = ojoin(image_dir, "block_{0}.jpg".format(i))
            cv2.imwrite(block_image_path, positioned_image)
            image_paths.append(block_image_path)

            print("TRANSLATION [get to block {0} pick point]: [{1}, {2}] -> [{3}, {4}]".format(i, x_i, y_i, x_f, y_f))
            data_x.extend(new_data_x)
            data_y.extend(new_data_y)
            x_i, y_i = x_f, y_f

            change_block_grip(gripper, True)

            new_data_x, new_data_y, x_f, y_f = get_to_next_block_drop_point(distance_sensor, imu, x_i, y_i)

            print("TRANSLATION [get to block {0} drop point]: [{1}, {2}] -> [{3}, {4}]".format(i, x_i, y_i, x_f, y_f))
            data_x.extend(new_data_x)
            data_y.extend(new_data_y)
            x_i, y_i = x_f, y_f

            change_block_grip(gripper, False)

            new_data_x, new_data_y, x_f, y_f = back_off_block_drop_point(imu, x_i, y_i)

            print("TRANSLATION [back off block {0} drop point]: [{1}, {2}] -> [{3}, {4}]".format(i, x_i, y_i, x_f, y_f))
            data_x.extend(new_data_x)
            data_y.extend(new_data_y)
            x_i, y_i = x_f, y_f
        except KeyboardInterrupt:
            print("User executed SIGTERM")
            break

    trajectory_image_path = ojoin(image_dir, "trajectory.jpg")
    plt.plot(data_x, data_y)
    plt.title("Robot Trajectory")
    plt.xlabel("x(t) (cm)")
    plt.ylabel("y(t) (cm)")
    plt.savefig(trajectory_image_path, bbox_inches="tight")
    image_paths.append(trajectory_image_path)

    send_images(
        ["rnvandemark@gmail.com"],
        [],
        snapshot_locations=image_paths
    )

    gripper.close(GRIPPER_INIT_DUTY_CYCLE)
    gameover()
    cap.release()
#    writer.release()
    cv2.destroyAllWindows()
