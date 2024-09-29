import GUI
import HAL
import cv2
import numpy as np
import time


def get_centroid(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask = mask1 + mask2

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        
        M = cv2.moments(largest_contour)
        
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            return (cX, cY)

    else:
        return (0, 0)
        
        
def pid_controller(error, prev_error, integral, Kp, Kd, Ki, dt):
    P = Kp * error
    
    D = Kd * (error - prev_error) / dt
    
    integral += error * dt
    I = Ki * integral
    
    control_signal = P + D + I
    return control_signal, integral



def center_object_pid(image, prev_error=0, integral=0, Kp=0.1, Kd=0.01, Ki=0.005):
    height, width, _ = image.shape
    image_center_x = width // 2

    centroid = get_centroid(image)
    centroid_x, centroid_y = centroid

    error = centroid_x - image_center_x

    current_time = time.time()

    dt = current_time - center_object_pid.prev_time if hasattr(center_object_pid, 'prev_time') else 0.1
    center_object_pid.prev_time = current_time

    control_signal, integral = pid_controller(error, prev_error, integral, Kp, Kd, Ki, dt)

    prev_error = error

    return control_signal, prev_error, integral


Kp = 0.005
Kd = 0.001
Ki = 0.001

prev_error = 0
integral = 0

HAL.setV(2.0)

while True:
    image = HAL.getImage()
    
    control_signal, prev_error, integral = center_object_pid(image, prev_error, integral, Kp, Kd, Ki)
    
    HAL.setW(control_signal * -1)
    
    time.sleep(0.1)
    