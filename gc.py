#! /usr/bin/python3

from sys import argv as sargv
import RPi.GPIO as gpio
from numpy import uint64, pi
import cv2
from time import sleep
from bufferless_video_capture import BufferlessVideoCapture
from gc_constants import *
from image_analysis import *

from email01 import send_snapshot

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

def do_motion(move_func, desi_dist_cm, desi_dist_str):
    desi_wheel_revs = desi_dist_cm / (pi * 6.5)
    desi_enc_ticks = int(20 * desi_wheel_revs)
    if desi_enc_ticks <= 0:
        return False

    print("{0} -> {1} wheel revs -> {2} encoder ticks".format(
        desi_dist_str,
        desi_wheel_revs,
        desi_enc_ticks
    ))

    desi_enc_ticks = max(desi_enc_ticks, 2)
    new_fl = False
    new_br = False
    counter_fl = uint64(0)
    button_fl = int(0)
    counter_br = uint64(0)
    button_br = int(0)

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
                print("fl=[c={0},s={1}], br=[c={2},s={3}]".format(
                    counter_fl,
                    signal_fl,
                    counter_br,
                    signal_br
                ))

            if new_fl and (counter_fl >= desi_enc_ticks):
                print("Front left finished at {0}!".format(i))
                break
            if new_br and (counter_br >= desi_enc_ticks):
                print("Back right finished at {0}!".format(i))
                break
            i += 1
    except KeyboardInterrupt:
        print("User executed SIGTERM")
    except Exception as e:
        print("Exception: " + str(e))

    stop()

    return True

def turn(desi_degrees):
    # d=14.5cm between each wheel
    FF = 4.0
    desi_dist_cm = pi * 14.5 * (abs(desi_degrees)/360) * FF
    desi_dist_str = "{0}d ({1}m arc)".format(desi_degrees, desi_dist_cm / 100.0)
    return do_motion(
        pivot_left if (desi_degrees < 0) else pivot_right,
        desi_dist_cm,
        desi_dist_str
    )

def translate(desi_dist_cm):
    desi_dist_str = "{0}m".format(desi_dist_cm / 100.0)
    return do_motion(
        reverse if (desi_dist_cm < 0) else forward,
        desi_dist_cm,
        desi_dist_str
    )

if "__main__" == __name__:
    show_img = (1 == int(sargv[1]))

    # Capture default camera
    cap = BufferlessVideoCapture(0)

    # Create video writer
    writer = cv2.VideoWriter(
        "HW9.mp4",
        cv2.VideoWriter_fourcc(*'DIVX'),
        20,
        (int(cap.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    )

    init()

    gripper = Gripper(GRIPPER_INIT_DUTY_CYCLE)
    sleep(3)

    grabbed = False
    while True:
        bgr_image = cap.read()
        bgr_image, have_block, enclosed_circle_area, degrees_away = find_target_block(bgr_image, BLUE_HSV_MIN, BLUE_HSV_MAX)

        if have_block:
            oriented = not turn(degrees_away)
            if oriented:
                print("Oriented!")
                positioned = (enclosed_circle_area >= 70000)
                if positioned:
                    print("Positioned! Grabbing...")
                    gripper.set_duty_cycle(5.0)
                    sleep(3)
                    grabbed = True
                    print("Done! Sending email...")
                    cv2.imwrite("trackblock02.jpg", bgr_image)
                    send_snapshot(
                        ["ENPM809TS19@gmail.com"],
                        ["rpatil10@umd.edu", "nickvand.enpm809t@gmail.com"],
                        snapshot_location="trackblock02.jpg"
                    )
                    print("Done!")
                    break
                else:
                    translate(2)

        if show_img:
            cv2.imshow("Track block", bgr_image)

        writer.write(bgr_image)

        if 27 == (cv2.waitKey(1) & 0xEFFFFF):
            break

    gripper.close(5.0)
    gameover()
    cap.release()
    writer.release()
    cv2.destroyAllWindows()
