#! /usr/bin/python3

from sys import argv as sargv
import RPi.GPIO as gpio
from numpy import uint64, pi
import cv2
from time import sleep
from bufferless_video_capture import BufferlessVideoCapture

from email01 import send_snapshot

BLUE_HSV_MIN = ( 95, 110,  75)
BLUE_HSV_MAX = (135, 180, 160)

class Gripper(object):
    def __init__(self, data_pin, freq, min_dc, max_dc, init_dc):
        self.data_pin = data_pin
        self.freq = freq
        self.min_dc = min_dc
        self.max_dc = max_dc
        gpio.setup(self.data_pin, gpio.OUT)
        self.pwm = gpio.PWM(self.data_pin, self.freq)
        self.set_duty_cycle(init_dc, first=True)

    def close(self, final_dc):
        self.set_duty_cycle(final_dc)
        sleep(2)
        self.pwm.stop()

    def set_duty_cycle(self, dc, first=False):
        fixed_dc = dc
        if fixed_dc > self.max_dc:
            fixed_dc = self.max_dc
        elif fixed_dc < self.min_dc:
            fixed_dc = self.min_dc

        if first:
            self.pwm.start(fixed_dc)
        else:
            self.pwm.ChangeDutyCycle(fixed_dc)

        return fixed_dc

class DefaultServoGripper(Gripper):
    def __init__(self):
        super().__init__(36, 50, 5.5, 9.5, 9.5)

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)
    gpio.setup(PIN_ENCODER_FRONT_LEFT, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(PIN_ENCODER_BACK_RIGHT, gpio.IN, pull_up_down=gpio.PUD_UP)

def move(v1, v2, v3, v4):
    gpio.output(31, v1)
    gpio.output(33, v2)
    gpio.output(35, v3)
    gpio.output(37, v4)

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

    gripper = DefaultServoGripper()
    sleep(3)

    grabbed = False
    while True:
        bgr_image = cap.read()
        height, width = bgr_image.shape[:2]
        half_height = height/2
        half_width = width/2
        bgr_image = cv2.warpAffine(
            src=bgr_image,
            M=cv2.getRotationMatrix2D(center=(half_width, half_height), angle=180, scale=1),
            dsize=(width, height)
        )
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        masked_image = cv2.inRange(hsv_image, BLUE_HSV_MIN, BLUE_HSV_MAX)
        _, thresh = cv2.threshold(masked_image, 127, 255, 0)
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        have_block = (len(contours) > 0)

        if have_block:
            (x,y),r = cv2.minEnclosingCircle(max(contours, key=cv2.contourArea))
            cv2.circle(bgr_image, (int(x), int(y)), int(r), (255,0,255), 2)
            if show_img:
                cv2.imshow("Track block", bgr_image)

            oriented = not turn((x - half_width) * 0.061)
            if oriented:
                print("Oriented!")
                positioned = ((pi * r * r) >= 70000)
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
        elif show_img:
            cv2.imshow("Track block", bgr_image)

        writer.write(bgr_image)

        if 27 == (cv2.waitKey(1) & 0xEFFFFF):
            break

    gripper.close(5.0)
    gameover()
    cap.release()
    writer.release()
    cv2.destroyAllWindows()
