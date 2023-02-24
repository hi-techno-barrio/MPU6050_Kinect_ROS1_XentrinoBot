/*

This code assumes that the Kinect sensor is connected to the Raspberry Pi via the USB port and that the 2WD motor is connected to the GPIO pins of the Raspberry Pi. 

The code uses the `libfreenect` library to get the depth image from the Kinect sensor and OpenCV to process the image and control the motors. 

In the `set_motor_speeds()` function, the left and right motor speeds are specified as input parameters, which can range from -100 to 100.
A positive speed value indicates forward motion, while a negative value indicates backward motion. The function sets the appropriate GPIO pins
to control the direction and speed of the motors.

In the main loop, the `get_depth()` function is called to get the current depth image from the Kinect sensor. The image is thresholded to obtain 
a binary image, which is then used to find the largest contour, assumed to be the line. The centroid of the contour is computed and used to 
determine the position of the line relative to the center of the image. Based on this position, the `set_motor_speeds()` function is called to
control the motors to move the robot towards the line.

Finally, the code displays the binary image for debugging purposes, and exits when the 'q' key is pressed. The GPIO pins are released before the code exits.

*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <libfreenect.h>
#include <wiringPi.h>

using namespace std;
using namespace cv;

// Set up GPIO pins for motor control
const int left_motor_pin1 = 0; // GPIO 17
const int left_motor_pin2 = 1; // GPIO 18
const int right_motor_pin1 = 2; // GPIO 27
const int right_motor_pin2 = 3; // GPIO 22

// Function to control the motors
void set_motor_speeds(int left_speed, int right_speed)
{
    if (left_speed > 0) {
        digitalWrite(left_motor_pin1, HIGH);
        digitalWrite(left_motor_pin2, LOW);
    } else if (left_speed < 0) {
        digitalWrite(left_motor_pin1, LOW);
        digitalWrite(left_motor_pin2, HIGH);
    } else {
        digitalWrite(left_motor_pin1, LOW);
        digitalWrite(left_motor_pin2, LOW);
    }
    if (right_speed > 0) {
        digitalWrite(right_motor_pin1, HIGH);
        digitalWrite(right_motor_pin2, LOW);
    } else if (right_speed < 0) {
        digitalWrite(right_motor_pin1, LOW);
        digitalWrite(right_motor_pin2, HIGH);
    } else {
        digitalWrite(right_motor_pin1, LOW);
        digitalWrite(right_motor_pin2, LOW);
    }
}

// Initialize Kinect sensor
cv::Mat get_depth()
{
    static uint16_t* depth_buffer = NULL;
    static size_t depth_size = 0;

    freenect_device* device;
    freenect_context* context;
    freenect_init(&context, NULL);
    freenect_select_subdevices(context, FREENECT_DEVICE_CAMERA);

    if (freenect_num_devices(context) > 0) {
        if (freenect_open_device(context, &device, 0) == 0) {
            freenect_set_depth_mode(device, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
            freenect_start_depth(device);

            freenect_frame_mode depth_mode = freenect_get_current_depth_mode(device);
            size_t depth_buffer_size = depth_mode.width * depth_mode.height * 2;

            if (depth_size != depth_buffer_size) {
                depth_buffer = (uint16_t*)realloc(depth_buffer, depth_buffer_size);
                depth_size = depth_buffer_size;
            }

            freenect_raw_tilt_state* state;
            freenect_update_tilt_state(device);
            state = freenect_get_tilt_state(device);
            int angle = state->tilt_angle;

            freenect_frame* depth_frame;
            if (freenect_sync_get_depth((void**)&depth_frame, NULL, 0, FREENECT_DEPTH_11BIT) >= 0) {
                memcpy(depth_buffer, depth_frame->data, depth_buffer_size);
                freenect_free_frame(depth_frame);
            }

            freenect_stop_depth(device);
            freenect_close_device(device);

            Mat depth_image(depth_mode.height, depth_mode.width, CV_16UC1, depth_buffer);
            return depth_image.clone();
        }
    }

    freenect_shutdown(context);
    return cv::Mat();
}

int main(int argc, char** argv)
{
    // Set up wiringPi library for GPIO control
    wiringPiSetup();
    pinMode(left_motor_pin1, OUTPUT);
    pinMode(left_motor_pin2, OUTPUT);
    pinMode(right_motor_pin1, OUTPUT);
    pinMode(right_motor_pin2, OUTPUT);
  while (true) {
    // Get current depth image from Kinect sensor
    cv::Mat depth = get_depth();

    // Threshold the depth image to get a binary image
    int threshold_value = 50;
    cv::Mat binary;
    cv::threshold(depth, binary, threshold_value, 255, cv::THRESH_BINARY_INV);

    // Find contours in the binary image
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Find the largest contour, which is assumed to be the line
    if (contours.size() > 0) {
        int largest_contour_index = 0;
        double largest_contour_area = cv::contourArea(contours[0]);
        for (int i = 1; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > largest_contour_area) {
                largest_contour_area = area;
                largest_contour_index = i;
            }
        }
        vector<cv::Point> largest_contour = contours[largest_contour_index];

        // Find the centroid of the largest contour
        cv::Moments moments = cv::moments(largest_contour);
        if (moments.m00 != 0) {
            int cx = (int)(moments.m10 / moments.m00);
            int cy = (int)(moments.m01 / moments.m00);

            // Control the motors based on the position of the line
            if (cx < 320) {
                set_motor_speeds(-50, 50); // Turn left
            } else {
                set_motor_speeds(50, -50); // Turn right
            }
        } else {
            set_motor_speeds(50, 50); // Move forward
        }
    } else {
        set_motor_speeds(50, 50); // Move forward
    }

    // Display the binary image for debugging purposes
    cv::imshow("binary", binary);

    // Exit if the 'q' key is pressed
    if (cv::waitKey(1) == 'q') {
        break;
    }
}

// Release GPIO pins
pinMode(left_motor_pin1, INPUT);
pinMode(left_motor_pin2, INPUT);
pinMode(right_motor_pin1, INPUT);
pinMode(right_motor_pin2, INPUT);

// Close all windows
cv::destroyAllWindows();

return 0;
}



            
