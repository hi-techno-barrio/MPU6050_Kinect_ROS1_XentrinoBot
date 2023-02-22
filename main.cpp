#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ros::Subscriber kinect_sub, imu_sub;
ros::Publisher motor_pub;

double linear_vel = 0.0;
double angular_vel = 0.0;
double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;

void kinectCallback(const sensor_msgs::Image& msg) {
  // Convert image data to OpenCV format for processing
  cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

  // Calculate depth data from image
  cv::Mat depth;
  cv::cvtColor(image, depth, CV_BGR2GRAY);
  depth.convertTo(depth, CV_32F, 1.0/255.0);
  cv::GaussianBlur(depth, depth, cv::Size(7, 7), 1.5, 1.5);

  // Calculate centroid of depth data
  cv::Moments m = cv::moments(depth, true);
  double cx = m.m10 / m.m00;
  double cy = m.m01 / m.m00;

  // Calculate linear velocity based on depth data
  linear_vel = (cx - image.cols / 2) / (image.cols / 2);

  // Limit the linear velocity to prevent excessive motor speed
  if (linear_vel > 1.0) {
    linear_vel = 1.0;
  } else if (linear_vel < -1.0) {
    linear_vel = -1.0;
  }
  
  // Calculate angular velocity based on linear velocity and roll angle
  angular_vel = -0.5 * linear_vel - 0.1 * roll;

  // Limit the angular velocity to prevent excessive motor speed
  if (angular_vel > 1.0) {
    angular_vel = 1.0;
  } else if (angular_vel < -1.0) {
    angular_vel = -1.0;
  }

  // Publish motor command
  geometry_msgs::Twist motor_msg;
  motor_msg.linear.x = linear_vel;
  motor_msg.angular.z = angular_vel;
  motor_pub.publish(motor_msg);
}

void imuCallback(const sensor_msgs::Imu& msg) {
  // Calculate roll and pitch based on IMU data
  tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  // Calculate angular velocity based on roll angle and linear velocity
  angular_vel = -0.5 * linear_vel - 0.1 * roll;

  // Limit the angular velocity to prevent excessive motor speed
  if (angular_vel > 1.0) {
    angular_vel = 1.0;
  } else if (angular_vel < -1.0) {
    angular_vel = -1.0;
  }

  // Publish motor command
  geometry_msgs::Twist motor_msg;
  motor_msg.linear.x = linear_vel;
  motor_msg.angular.z = angular_vel;
  motor_pub.publish(motor_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_control");
  ros::NodeHandle nh;

  kinect_sub = nh
  kinect_sub = nh.subscribe("/kinect/image", 1, kinectCallback);
  imu_sub = nh.subscribe("/imu/data", 1, imuCallback);
  motor_pub = nh.advertise<geometry_msgs::Twist>("/motor/cmd_vel", 1);

  // Initialize MPU6050
  MPU6050 mpu;
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);

  // Calibrate MPU6050
  int num_samples = 100;
  Vector3D accel_bias(0.0, 0.0, 0.0);
  Vector3D gyro_bias(0.0, 0.0, 0.0);
  for (int i = 0; i < num_samples; i++) {
    Vector3D accel_data, gyro_data;
    mpu.getMotion6(&accel_data.x, &accel_data.y, &accel_data.z, &gyro_data.x, &gyro_data.y, &gyro_data.z);
    accel_bias += accel_data;
    gyro_bias += gyro_data;
    delay(10);
  }
  accel_bias /= num_samples;
  gyro_bias /= num_samples;
  mpu.setAccelBias(accel_bias.x, accel_bias.y, accel_bias.z);
  mpu.setGyroBias(gyro_bias.x, gyro_bias.y, gyro_bias.z);

  // Main loop
  ros::Rate rate(50);
  while (ros::ok()) {
    // Read MPU6050 data
    Vector3D accel_data, gyro_data;
    mpu.getMotion6(&accel_data.x, &accel_data.y, &accel_data.z, &gyro_data.x, &gyro_data.y, &gyro_data.z);
    roll = atan2(accel_data.y, accel_data.z);
    pitch = atan2(-accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));
    double gyro_rate = gyro_data.x;

    // Calculate linear velocity based on depth data and MPU6050 data
    linear_vel = (linear_vel + 0.02 * (gyro_rate + 0.1 * pitch)) * 0.98;

    // Limit the linear velocity to prevent excessive motor speed
    if (linear_vel > 1.0) {
      linear_vel = 1.0;
    } else if (linear_vel < -1.0) {
      linear_vel = -1.0;
    }

    // Calculate angular velocity based on linear velocity and roll angle
    angular_vel = -0.5 * linear_vel - 0.1 * roll;

    // Limit the angular velocity to prevent excessive motor speed
    if (angular_vel > 1.0) {
      angular_vel = 1.0;
    } else if (angular_vel < -1.0) {
      angular_vel = -1.0;
    }

    // Publish motor command
    geometry_msgs::Twist motor_msg;
    motor_msg.linear.x = linear_vel;
    motor_msg.angular.z = angular_vel;
    motor_pub.publish(motor_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
