import cv2
import numpy
import time
import configparser
import picamera
import RPi.GPIO as GPIO


class PID:
    def __init__(self, p=1, i=0, d=0):
        # initialize gains
        self.kP = p
        self.kI = i
        self.kD = d

        self.currTime = time.time()
        self.prevTime = self.currTime

        self.prevError = 0

        self.cP = 0
        self.cI = 0
        self.cD = 0

    def update(self, error):
        # pause for a bit
        time.sleep(0.2)
        # grab the current time and calculate delta time
        self.currTime = time.time()
        delta_time = self.currTime - self.prevTime
        # delta error
        delta_error = error - self.prevError
        # proportional term
        self.cP = error
        # integral term
        self.cI += error * delta_time
        # derivative term and prevent divide by zero
        self.cD = (delta_error / delta_time) if delta_time > 0 else 0
        # save previous time and error for the next update
        self.prevTime = self.currTime
        self.prevError = error
        # sum the terms and return
        return sum([
            self.kP * self.cP,
            self.kI * self.cI,
            self.kD * self.cD])


def face_detection_visualize(face_cascade):
    # Read the input image
    img = cv2.imread('test.jpg')
    height, width, channels = img.shape
    img_x, img_y = (width // 2, height // 2)

    # Convert into gray-scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Detect faces
    faces = face_cascade.detectMultiScale(gray, 1.3, 3)
    (x, y, w, h) = faces[0][:]  # Ordered by probability?
    face_x, face_y = (x + w // 2, y + h // 2)

    # Draw rectangle around the faces
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
    cv2.circle(img, (face_x, face_y), 5, (255, 0, 0), 2)
    cv2.circle(img, (img_x, img_y), 5, (255, 0, 0), 2)
    cv2.circle(img, (img_x, img_y), 25, (255, 255, 0), 2)

    cv2.line(img, (face_x, face_y), (img_x, face_y), (0, 255, 0), 2)
    cv2.line(img, (img_x, img_y), (img_x, face_y), (0, 255, 0), 2)

    # Display the output
    cv2.imshow('img', img)
    cv2.waitKey()


def face_detection(face_cascade, img):
    # Convert into gray-scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Detect faces
    faces = face_cascade.detectMultiScale(gray, 1.3, 3)
    (x, y, w, h) = faces[0][:]  # Ordered by probability?
    face_x, face_y = (x + w // 2, y + h // 2)

    return face_x, face_y


def set_servos(pan_ctrl, tilt_ctrl):
    pass


def object_centered(obj_coord, img_coord):
    tol = 20
    return numpy.sqrt((img_coord[0]-obj_coord[0])**2+(img_coord[1]-obj_coord[1])**2) <= tol


def process(pan_params, tilt_params):
    # create a PID and initialize it
    pan_pid = PID(pan_params[0], pan_params[1], pan_params[2])
    tilt_pid = PID(tilt_params[0], tilt_params[1], tilt_params[2])

    # load Haar cascade object
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    with picamera.PiCamera() as camera:
        time.sleep(2)
        with picamera.array.PiRGBArray(camera) as img:
            # loop indefinitely
            while True:
                camera.capture(img, format='rgb')
                face_x, face_y = face_detection(face_cascade, img)

                # Read the input image
                height, width, channels = img.shape
                img_x, img_y = (width // 2, height // 2)

                # calculate the error
                pan_error = img_x - face_x
                tilt_error = img_y - face_y

                # update the control values
                pan_ctrl = pan_pid.update(pan_error)
                tilt_ctrl = tilt_pid.update(tilt_error)

                # update servo target
                set_servos(pan_ctrl, tilt_ctrl)


config = configparser.ConfigParser()
config.read('config.ini')
x = config['LIMITS']['...']
y = config['PID']['...']

TILT_PIN = 0    # RPi GPIO
PAN_PIN = 0     # RPi GPIO

# Calibration params
PAN_LEFT = 0
PAN_RIGHT = 0

TILT_DOWN = 0
TILT_UP = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(TILT_PIN, GPIO.OUT)
GPIO.setup(PAN_PIN, GPIO.OUT)

pwm_tilt = GPIO.PWM(TILT_PIN, 50)  # Tilt pin at 50 Hz
pwm_pan = GPIO.PWM(PAN_PIN, 50)

#pwm.stop()
#GPIO.cleanup()
