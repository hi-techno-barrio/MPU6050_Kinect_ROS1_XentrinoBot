/*


*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <libfreenect.h>

// Define the target color in HSV space
cv::Scalar target_color(60, 255, 255);  // green

// Define the desired linear and angular speeds
double linear_speed = 0.2;  // meters per second
double angular_speed = 0.5;  // radians per second

// Set the intrinsic parameters for the Kinect camera
// These values are specific to the Kinect for Xbox 360
double fx = 594.21;
double fy = 591.04;
double cx = 339.5;
double cy = 242.7;

// Main function
int main(int argc, char** argv) {
    // Initialize the Kinect sensor
    freenect_context* context;
    freenect_device* device;
    freenect_init(&context, NULL);
    freenect_open_device(context, &device, 0);
    freenect_set_video_mode(device, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
    freenect_set_depth_mode(device, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    freenect_set_depth_callback(device, [](freenect_device* dev, void* depth, uint32_t timestamp) {});
    freenect_start_video(device);

    // Create OpenCV windows for displaying the image and mask
    cv::namedWindow("Image");
    cv::namedWindow("Mask");

    // Main loop
    while (true) {
        // Capture an image from the Kinect sensor
        uint8_t* kinect_data = NULL;
        freenect_sync_get_video((void**)&kinect_data, NULL, 0, FREENECT_VIDEO_RGB);
        cv::Mat kinect_bgr(480, 640, CV_8UC3, kinect_data);
        cv::cvtColor(kinect_bgr, kinect_bgr, cv::COLOR_RGB2BGR);
        cv::Mat kinect_hsv;
        cv::cvtColor(kinect_bgr, kinect_hsv, cv::COLOR_BGR2HSV);

        // Define a binary mask for the target color
        cv::Mat mask;
        cv::inRange(kinect_hsv, target_color, target_color, mask);

        // Find the contours in the binary mask
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // If no contours are found, stop the robot
        if (contours.empty()) {
            std::cout << "No target found, stopping the robot" << std::endl;
            // TODO: Stop the robot
            continue;
        }

        // Find the contour with the largest area
        size_t max_area_index = 0;
        double max_area = cv::contourArea(contours[0]);
        for (size_t i = 1; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_area_index = i;
            }
        }

        // Find the centroid of the largest contour
        cv::Moments moments = cv::moments(contours[max_area_index]);
        cv::Point2f centroid(m

double cx = moments.m10 / moments.m00;
    double cy = moments.m01 / moments.m00;

    // Calculate the depth at the centroid of the largest contour
    uint16_t* depth_data = NULL;
    freenect_sync_get_depth((void**)&depth_data, NULL, 0, FREENECT_DEPTH_11BIT);
    double depth = double(depth_data[int(cy) * 640 + int(cx)]) / 2047 * 10;  // convert depth to meters

    // Calculate the position of the centroid in 3D space
    double x = (cx - cx) * depth / fx;
    double y = (cy - cy) * depth / fy;

    // Calculate the error between the target position and the current position
    double error = x;

    // Calculate the angular velocity based on the error
    double angular_velocity = angular_speed * error / cx;

    // Calculate the linear velocity based on the depth
    double linear_velocity = linear_speed * depth;

    // TODO: Send the motor commands to control the robot based on the angular and linear velocities
    // For example, you could use a PWM motor controller to adjust the motor speed and direction

    // Display the image and mask for visualization
    cv::circle(kinect_bgr, cv::Point(cx, cy), 10, cv::Scalar(0, 0, 255), -1);
    cv::imshow("Image", kinect_bgr);
    cv::imshow("Mask", mask);
    cv::waitKey(1);
}

// Release the Kinect sensor
freenect_stop_video(device);
freenect_close_device(device);
freenect_shutdown(context);

return 0;
}
