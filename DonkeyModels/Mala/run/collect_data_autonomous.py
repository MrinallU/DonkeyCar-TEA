import cv2
import os
import pygame
from cv2.gapi import BGR2Gray
import tensorflow as tf
from Adafruit_GPIO import I2C
import time
import numpy as np
import Adafruit_PCA9685
import donkeycar as dk
import curses


def get_bus():
    return PCA9685_I2C_BUSNUM


def gstreamer_pipeline(
    capture_width=3280,
    capture_height=2464,
    output_width=224,
    output_height=224,
    framerate=21,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv flip-method=%d ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            output_width,
            output_height,
        )
    )


stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(True)
stdscr.nodelay(True)

PCA9685_I2C_BUSNUM = 1
PCA9685_I2C_ADDR = 0x40
I2C.get_default_bus = get_bus
PCA9685_I2C_ADDR = 0x40

pwm = Adafruit_PCA9685.PCA9685(address=PCA9685_I2C_ADDR)
pwm.set_pwm_freq(60)  # frequence of PWM

pygame.init()
"""
You need to change these PWM values.
"""
STEERING_CHANNEL = 1
STEERING_LEFT_PWM = 290  # pwm value for full left steering
STEERING_RIGHT_PWM = 490  # pwm value for full right steering

THROTTLE_CHANNEL = 0
THROTTLE_FORWARD_PWM = 430  # pwm value for max forward throttle
THROTTLE_STOPPED_PWM = 370  # pwm value for no movement
THROTTLE_REVERSE_PWM = 300  # pwm value for max reverse throttle

# init steering and ESC
pwm.set_pwm(THROTTLE_CHANNEL, 0, int(THROTTLE_FORWARD_PWM))
time.sleep(0.1)
pwm.set_pwm(THROTTLE_CHANNEL, 0, int(THROTTLE_REVERSE_PWM))
time.sleep(0.1)
pwm.set_pwm(THROTTLE_CHANNEL, 0, int(THROTTLE_STOPPED_PWM))
time.sleep(0.1)


def process_image(img):
    _cut_height = 60
    _cut_img = img[_cut_height:224, :]
    processed_img = cv2.cvtColor(_cut_img, cv2.COLOR_BGR2GRAY)
    return processed_img


if __name__ == "__main__":
    w = 224
    h = 224
    running = True
    frame = None
    flip_method = 6
    capture_width = 3280
    capture_height = 2464
    framerate = 21

    directory = "my_data"
    if not os.path.exists(directory):
        os.makedirs(directory)
    catalog_path = os.path.join(directory, "catalog.txt")
    camera = cv2.VideoCapture(
        gstreamer_pipeline(
            capture_width=capture_width,
            capture_height=capture_height,
            output_width=w,
            output_height=h,
            framerate=framerate,
            flip_method=flip_method,
        ),
        cv2.CAP_GSTREAMER,
    )

    model_path = "Your Model"
    ret = True
    model = tf.keras.models.load_model(model_path, compile=False)
    threshold = 204
    time1 = time.time()
    count = 0
    throttle_all = []
    steering_all = []
    throttle_pwm = []
    steering_pwm = []
    new_throttle = 0
    steering = 0
    curFrame = 0
    joysticks = {}
    initTime = time.time()
    print("Running...")

    while True:
        time0 = time.time()
        if time0 - initTime < 10:
            continue
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True  # Flag that we are done scatalog_path = os.path.join(directory, 'catalog.txt')o we exit this loop.

            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
                if event.button == 0:
                    joystick = joysticks[event.instance_id]
                    if joystick.rumble(0, 0.7, 500):
                        print(f"Rumble effect played on joystick {event.instance_id}")

            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every
                # joystick, filling up the list without needing to create them manually.
                joy = pygame.joystick.Joystick(event.device_index)
                joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connencted")

            if event.type == pygame.JOYDEVICEREMOVED:
                del joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")

        ret, img_raw = camera.read()
        img = img_raw.astype(float)
        height = 80

        # Turn into grayscale
        img = process_image(img)
        out = model(img[None, :])

        throttle = new_throttle

        steering = out[0][0]

        throttle_all.append(throttle)
        steering_all.append(steering)

        if abs(throttle) >= 0.05 and ret:
            curFrame += 1
            down_width = 224
            down_height = 224
            down_points = (down_width, down_height)
            img = cv2.resize(img_raw, down_points, interpolation=cv2.INTER_LINEAR)
            filename = f"image_{curFrame}.jpg"  # Create a filename with the number
            # Full path where the image will be saved
            file_path = os.path.join(directory, filename)
            cv2.imwrite(file_path, img)
            with open(catalog_path, "a") as catalog_file:
                catalog_file.write(
                    f"image: {filename}, index: {curFrame}, steering: {steering}, throttle: {throttle}\n"
                )

        if throttle > 0:
            throttle_pulse = dk.utils.map_range(
                throttle, 0, 1, THROTTLE_STOPPED_PWM, THROTTLE_FORWARD_PWM
            )
        else:
            throttle_pulse = dk.utils.map_range(
                throttle, -1, 0, THROTTLE_REVERSE_PWM, THROTTLE_STOPPED_PWM
            )

        steering_pulse = dk.utils.map_range(
            steering, -1, 1, STEERING_LEFT_PWM, STEERING_RIGHT_PWM
        )

        pwm.set_pwm(THROTTLE_CHANNEL, 0, int(throttle_pulse))
        pwm.set_pwm(STEERING_CHANNEL, 0, int(steering_pulse))

        count += 1
        c = stdscr.getch()

        for joystick in joysticks.values():
            name = joystick.get_name()
            if name != "Logitech Cordless RumblePad 2":
                continue
            new_throttle = joystick.get_button(0) + 0.02
            new_steering = joystick.get_button(1) - 0.02

        if c == ord("q"):
            steering = 0.0
            steering_pulse = dk.utils.map_range(
                steering, -1, 1, STEERING_LEFT_PWM, STEERING_RIGHT_PWM
            )
            pwm.set_pwm(STEERING_CHANNEL, 0, int(steering_pulse))
            break
        if c == ord("w"):
            new_throttle = new_throttle + 0.02
        if c == ord("s"):
            new_throttle = new_throttle - 0.02

    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()
    pwm.set_pwm(THROTTLE_CHANNEL, 0, int(THROTTLE_STOPPED_PWM))
    time2 = time.time()
    print("Running times: ", count)
    print("Time overall: ", time2 - time1)
    print("Average time: ", (time2 - time1) / count)
    print("SHUTDOWN...")
