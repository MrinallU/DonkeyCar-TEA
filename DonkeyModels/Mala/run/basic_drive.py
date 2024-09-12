import cv2
import tensorflow as tf
from Adafruit_GPIO import I2C
import time
import numpy as np
import Adafruit_PCA9685
import donkeycar as dk
import curses
import nanocamera as nano
import pygame

def get_bus():
    return PCA9685_I2C_BUSNUM


def gstreamer_pipeline(capture_width=3280, capture_height=2464, output_width=224, output_height=224, framerate=21, flip_method=0):   
        return 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv flip-method=%d ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
                capture_width, capture_height, framerate, flip_method, output_width, output_height)


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


if __name__ == '__main__':
    w = 224
    h = 224
    running = True
    frame = None
    flip_method = 6
    capture_width = 3280
    capture_height = 2464
    framerate = 21

    joysticks = {}
    camera = cv2.VideoCapture(
            gstreamer_pipeline(
                capture_width=capture_width,
                capture_height=capture_height,
                output_width=w,
                output_height=h,
                framerate=framerate,
                flip_method=flip_method),
            cv2.CAP_GSTREAMER)    
    #camera = nano.Camera(flip=0, width=3280, height=2464, fps=21)
    pygame.init()
    model_path = "Your Model"
    ret = True
    # model = tf.keras.models.load_model(model_path, compile=False)
    threshold = 204
    time1 = time.time()
    count = 0
    throttle_all = []
    steering_all = []
    throttle_pwm = []
    steering_pwm = []
    new_throttle = 0
    new_steering = 0
    steering = 0
    quitProgram = False
    print("Running...")

    while True:
        # Event processing step.
    # Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
        # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True  # Flag that we are done so we exit this loop.

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

        time0 = time.time()
        img_raw = camera.read()
        #print(img_raw)
        img = img_raw
        height = 80

        # Turn into grayscale
        img = np.dot(img[..., :3], [0.299, 0.587, 0.114])[height:224, :]
        out = model(img[None, :])

        throttle = new_throttle

        # steering = out[0][0]
        steering = new_steering

        throttle_all.append(throttle)
        steering_all.append(steering)

        if throttle > 0:
            throttle_pulse = dk.utils.map_range(
                throttle, 0, 1, THROTTLE_STOPPED_PWM, THROTTLE_FORWARD_PWM)
        else:
            throttle_pulse = dk.utils.map_range(
                throttle, -1, 0, THROTTLE_REVERSE_PWM, THROTTLE_STOPPED_PWM)

        steering_pulse = dk.utils.map_range(
            steering, -1, 1, STEERING_LEFT_PWM, STEERING_RIGHT_PWM)

        pwm.set_pwm(THROTTLE_CHANNEL, 0, int(throttle_pulse))
        pwm.set_pwm(STEERING_CHANNEL, 0, int(steering_pulse))

        count += 1
        c = stdscr.getch()
        for joystick in joysticks.values():
            jid = joystick.get_instance_id()
            # Get the name from the OS for the controller/joystick.
            name = joystick.get_name()
            if(name != "Logitech Gamepad F710"):
                continue
            guid = joystick.get_guid()
            power_level = joystick.get_power_level()
            # Usually axis run in pairs, up/down for one, and left/right for
            # the other. Triggers count as axes.
            axes = joystick.get_numaxes()

            #for i in range(axes):
             #   axis = joystick.get_axis(i)
            new_throttle = joystick.get_axis(1)*0.5
            new_steering = joystick.get_axis(3)*0.8
                
            buttons = joystick.get_numbuttons()

            #for i in range(buttons):
             #   button = joystick.get_button(i)
            #hats = joystick.get_numhats()

            # Hat position. All or nothing for direction, not a float like
            # get_axis(). Position is a tuple of int values (x, y).
            #for i in range(hats):
             #   hat = joystick.get_hat(i)


        if c == ord('q') :
            steering = 0.0
            steering_pulse = dk.utils.map_range(
                steering, -1, 1, STEERING_LEFT_PWM, STEERING_RIGHT_PWM)
            pwm.set_pwm(STEERING_CHANNEL, 0, int(steering_pulse))
            break
        
        if c == ord('w'):
            new_throttle = new_throttle + 0.02
        if c == ord('s'):
            new_throttle = new_throttle - 0.02
        if c == ord('a'): 
            new_steering = new_steering + 0.01
        if c == ord('d'): 
            new_steering = new_steering - 0.01

    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()
    pwm.set_pwm(THROTTLE_CHANNEL, 0, int(THROTTLE_STOPPED_PWM))
    time2 = time.time()
    print("Steering value: ", steering)
    print("Running times: ", count)
    print('Time overall: ', time2 - time1)
    print('Average time: ', (time2 - time1) / count)
    print("SHUTDOWN...")
