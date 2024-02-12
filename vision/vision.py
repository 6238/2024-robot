# import the opencv library 
import cv2
import numpy as np
from networktables import NetworkTables
from simple_pid import PID

NetworkTables.initialize(server="10.62.38.2")
sd = NetworkTables.getTable('vision')
sd.putNumber('right_joystick', 0)

p, i, d = .15, .1, .15
pid = PID(p, i, d, setpoint=.5)

cap = cv2.VideoCapture(0)
while True:
    _, orig = cap.read()
    frame = orig
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
    lower = np.array([0, 165, 30]) 
    upper = np.array([255, 255, 150]) 
    mask = cv2.inRange(hsv, lower, upper)    

    threshhold_area = 5000
    window_size = 1
    contours, hierarchy = cv2.findContours(mask,  cv2.RETR_LIST , cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    if contours and cv2.contourArea(contours[0]) > threshhold_area:
        contour = cv2.convexHull(contours[0])
        if len(contour) >= 5:
            ellipse = cv2.fitEllipse(contour)
            cv2.ellipse(frame, ellipse, (255,0,0), 5)
            control = pid(round((ellipse[0][0])/mask.shape[1], 2))
            sd.putNumber('right_joystick', np.tanh(control))
            #print((ellipse[0][0])/mask.shape[1])
            # print(np.tanh(control))
        else:
            pid = PID(p, i, d, setpoint=.5)
            sd.putNumber('right_joystick', 0)
            # print('1', 0)
    else:
        pid = PID(p, i, d, setpoint=.5)
        sd.putNumber('right_joystick', 0)
        # print('2', 0)
    
    # cv2.imshow('frame', orig)
    # cv2.imshow('frame', frame)
    # key = cv2.waitKey(5)
    # if key == ord('q'):
    #     break
