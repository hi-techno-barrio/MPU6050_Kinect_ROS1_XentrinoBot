/*
The code assumes that the RealSense T265 is connected to the Raspberry Pi via USB and that the 2WD motor is connected to the GPIO pins of the Raspberry Pi.

In the set_motor_speeds() function, the left and right motor speeds are specified as input parameters, which can range from -100 to 100. A positive speed 
value indicates forward motion, while a negative value indicates backward motion. The function sets the appropriate GPIO pins to control the direction and 
speed of the motors.

In the main loop, the RealSense T265 is set up with a pipeline and a configuration that enables streaming of pose data in 6 degrees of freedom (6DOF). 
The loop waits for a new pose sample, extracts the position and orientation data from the pose data, and uses these values to control the motors.
The forward speed is set based on the z position, while the turn speed is set based on the roll orientation. The set_motor_speeds() function is 
called to control the motors to move the robot in the desired direction.

Finally, the code displays the position and orientation data for debugging purposes, and exits when the 'q' key is pressed. The GPIO pins are released 
before the code exits.

*/

#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
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

int main(int argc, char** argv)
{
    // Set up wiringPi library for GPIO control
    wiringPiSetup();
    pinMode(left_motor_pin1, OUTPUT);
    pinMode(left_motor_pin2, OUTPUT);
    pinMode(right_motor_pin1, OUTPUT);
    pinMode(right_motor_pin2, OUTPUT);

    // Set up RealSense T265
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    pipe.start(cfg);

    while (true) {
        // Wait for a new pose sample
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::pose_frame pose_frame = frames.first_or_default(RS2_STREAM_POSE);

        // Extract the pose data
        rs2_pose pose_data = pose_frame.get_pose_data();

        // Get position and orientation from the pose data
        float x = pose_data.translation.x;
        float z = pose_data.translation.z;
        float roll = pose_data.rotation.z;
        float pitch = pose_data.rotation.x;

        // Control the motors based on the position and orientation
        int turn_speed = (int)(roll * 100);
        int forward_speed = (int)(z * 100);
        set_motor_speeds(forward_speed + turn_speed, forward_speed - turn_speed);

        // Display the position and orientation for debugging purposes
        cout << "x: " << x << " z: " << z << " roll: " << roll << " pitch: " << pitch << endl;

        // Exit if the 'q' key is pressed
        if (waitKey(1) == 'q') {
            break;
        }
    }

    // Release GPIO pins
    pinMode(left_motor_pin1, INPUT);
    pinMode(left_motor_pin2, INPUT);
    pinMode(right_motor_pin1, INPUT);
    pinMode(right_motor_pin2, INPUT);

    return 0;
}
