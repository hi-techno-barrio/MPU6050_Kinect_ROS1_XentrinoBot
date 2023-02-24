/*
The code is similar to the previously shown Python code, except that
it now uses the RPi.GPIO library for GPIO control instead of the gpiozero library.
In the set_motor_speeds() function, the left and right motor speeds are specified as input parameters, which can range from -100 to 100. A positive speed value 
indicates forward motion, while a negative value indicates backward motion. The function sets the appropriate GPIO pins to control the direction and speed of 
the motors.

In the get_depth() function, the freenect library is used to get the depth image from the Kinect sensor, which is then flipped horizontally using cv2.flip() 
to correct for the reversed image from the Kinect sensor.

In the main loop, the get_depth() function is called to get the current depth image from the Kinect sensor. The image is thresholded to obtain a binary image, 
which is then used to find the largest contour, assumed to be the line. The centroid of the contour is computed and used to determine the position of the line 
relative to the center of the image. Based on this position, the set_motor_speeds() function is called to control the motors to move the robot towards the line.

Finally, the code displays the binary image for debugging purposes, and exits when the 'q' key is pressed. The GPIO pins are cleaned up and the video capture
device is released before the code exits.
*/

import cv2
import freenect
import RPi.GPIO as GPIO

# Set up GPIO pins for motor control
GPIO.setmode(GPIO.BOARD)
left_motor_pin1 = 11
left_motor_pin2 = 13
right_motor_pin1 = 15
right_motor_pin2 = 16
GPIO.setup(left_motor_pin1, GPIO.OUT)
GPIO.setup(left_motor_pin2, GPIO.OUT)
GPIO.setup(right_motor_pin1, GPIO.OUT)
GPIO.setup(right_motor_pin2, GPIO.OUT)

# Function to control the motors
def set_motor_speeds(left_speed, right_speed):
    if left_speed > 0:
        GPIO.output(left_motor_pin1, GPIO.HIGH)
        GPIO.output(left_motor_pin2, GPIO.LOW)
    elif left_speed < 0:
        GPIO.output(left_motor_pin1, GPIO.LOW)
        GPIO.output(left_motor_pin2, GPIO.HIGH)
    else:
        GPIO.output(left_motor_pin1, GPIO.LOW)
        GPIO.output(left_motor_pin2, GPIO.LOW)
    if right_speed > 0:
        GPIO.output(right_motor_pin1, GPIO.HIGH)
        GPIO.output(right_motor_pin2, GPIO.LOW)
    elif right_speed < 0:
        GPIO.output(right_motor_pin1, GPIO.LOW)
        GPIO.output(right_motor_pin2, GPIO.HIGH)
    else:
        GPIO.output(right_motor_pin1, GPIO.LOW)
        GPIO.output(right_motor_pin2, GPIO.LOW)

# Initialize Kinect sensor
def get_depth():
    array, _ = freenect.sync_get_depth()
    array = array.astype('uint8')
    return cv2.flip(array, 1)

# Set up video capture device
cap = cv2.VideoCapture(0)

while True:
    # Get current depth image from Kinect sensor
    depth = get_depth()

    # Threshold the depth image to get a binary image
    threshold_value = 50
    _, binary = cv2.threshold(depth, threshold_value, 255, cv2.THRESH_BINARY_INV)

    # Find contours in the binary image
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour, which is assumed to be the line
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)

        # Find the centroid of the largest contour
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Control the motors based on the position of the line
            if cx < 320:
                set_motor_speeds(-50, 50) # Turn left
            else:
                set_motor_speeds(50, -50) # Turn right
        else:
            set_motor_speeds(50, 50) # Move forward
    else:
        set_motor_speeds(50, 50) # Move forward

    # Display the binary image for debugging purposes
    cv2.imshow("binary", binary)

    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up GPIO pins
GPIO.cleanup()

# Release video capture device
cap.release()

# Close all windows
cv2.destroyAllWindows()
