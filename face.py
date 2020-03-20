import cv2
import numpy
import time
#import configparser
import picamera
import picamera.array
#import RPi.GPIO as GPIO
from RPIO import PWM

# ADD TO CONFIG!!!
# Servo parameters
RIGHT = 2.0 # 45
CENTER = 4.4 # 0
LEFT = 7.0 # -45

DOWN = 6
MID = 3.5
UP = 1.75

# Camera Parameters
IMG_RES = (460, 340) # might need tuning
IMG_X, IMG_Y = (IMG_RES[0] // 2, IMG_RES[1] // 2)

HORZ_VIEW = 62.2
VERT_VIEW = 48.8

# Controller Parameters
PAN_PID = (0.05, 0.1, 0) # might need tuning
TILT_PID = (0.15, 0.2, 0) # might need tuning


class PID:
    def __init__(self, p=0, i=0, d=0):
        # initialize gains
        self.kP = p
        self.kI = i
        self.kD = d

    def reset(self):
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


def face_detection(face_cascade, img):
    # Convert into gray-scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Detect faces
    faces = face_cascade.detectMultiScale(gray, 1.3, 3)
    (x, y, w, h) = faces[0][:]  # Ordered by probability?
    face_x, face_y = (x + w // 2, y + h // 2)

    return face_x, face_y


def is_centered(obj_coord, img_coord):
    tol = 20
    return numpy.sqrt((img_coord[0]-obj_coord[0])**2+(img_coord[1]-obj_coord[1])**2) <= tol


def px2duty(px_error, center_pixel, view_angle, min_angle, max_angle,min_duty,max_duty):
	deg_error = numpy.interp(px_error, [-center_pixel,center_pixel],[-view_angle, view_angle])
	servo_error = numpy.interp(deg_error, [-view_angle,view_angle], [min_angle,max_angle])
	return numpy.interp(servo_error, [min_angle,max_angle],[min_duty, max_duty])


if __name__ == "__main__":
    # create a PID and initialize it
    pan_pid = PID(0,0,0)
    tilt_pid = PID(0.5,0,0)

    # load Haar cascade object
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    with picamera.PiCamera() as camera:
        time.sleep(1)
	camera.rotation = 180
	camera.resolution  = (IMG_RES[0],IMG_RES[1])
        with picamera.array.PiRGBArray(camera) as img:
	    camera.start_preview()
	    time.sleep(2)
            # loop indefinitely
            for foo in camera.capture_continuous(img, format='rgb'):
		# camera.capture(img, format='rgb')
                img.truncate()
		img.seek(0)
		face_x, face_y = face_detection(face_cascade, img.array)
                
		# calculate the error
                pan_error = px2duty(px_error, center_pixel, view_angle, min_angle, max_angle,min_duty,max_duty)
		tilt_error = px2duty(px_error, center_pixel, view_angle, min_angle, max_angle,min_duty,max_duty)

		pan_pid.update(pan_error)
		tilt_pid.update(

		# set servo motors
		set_servo(PAN_PIN, pan_error)
		set_servo(TILT_PIN, tilt_error)

		if is_centered((face_x, face_y), (img_x, img_y))
			break
#try:
#except: 
#finally:
#	pan_servo.stop_servo(PAN_PIN)
#	tilt_servo.stop_servo(TILT_PIN)