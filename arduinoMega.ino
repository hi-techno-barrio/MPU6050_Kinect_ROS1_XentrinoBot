#include <Wire.h>
#include <MPU6050.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

MPU6050 mpu;
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> motor_sub("/motor/cmd_vel", &motorCallback);
ros::Publisher left_encoder_pub("left_encoder", &left_encoder_msg);
ros::Publisher right_encoder_pub("right_encoder", &right_encoder_msg);
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;
long left_encoder_last = 0;
long right_encoder_last = 0;
geometry_msgs::Twist left_encoder_msg;
geometry_msgs::Twist right_encoder_msg;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  nh.initNode();
  nh.subscribe(motor_sub);
  nh.advertise(left_encoder_pub);
  nh.advertise(right_encoder_pub);
  attachInterrupt(0, leftEncoderISR, RISING);
  attachInterrupt(1, rightEncoderISR, RISING);
}

void loop() {
  nh.spinOnce();
  updateEncoderCount();
  delay(10);
}

void motorCallback(const geometry_msgs::Twist& msg) {
  // Convert linear and angular velocity to motor speeds
  double linear_vel = msg.linear.x;
  double angular_vel = msg.angular.z;
  double left_speed = linear_vel - angular_vel;
  double right_speed = linear_vel + angular_vel;

  // Limit motor speeds to prevent excessive speed
  if (left_speed > 255.0) {
    left_speed = 255.0;
  } else if (left_speed < -255.0) {
    left_speed = -255.0;
  }
  if (right_speed > 255.0) {
    right_speed = 255.0;
  } else if (right_speed < -255.0) {
    right_speed = -255.0;
  }

  // Set motor speeds
  analogWrite(6, abs(left_speed));
  analogWrite(9, abs(right_speed));
  if (left_speed >= 0) {
    digitalWrite(7, LOW);
  } else {
    digitalWrite(7, HIGH);
  }
  if (right_speed >= 0) {
    digitalWrite(8, LOW);
  } else {
    digitalWrite(8, HIGH);
  }
}

void leftEncoderISR() {
  left_encoder_count++;
}

void rightEncoderISR() {
  right_encoder_count++;
}

void updateEncoderCount() {
  long left_encoder_current = left_encoder_count;
  long right_encoder_current = right_encoder_count;
  long left_encoder_diff = left_encoder_current - left_encoder_last;
  long right_encoder_diff = right_encoder_current - right_encoder_last;
  left_encoder_last = left_encoder_current;
  right_encoder_last = right_encoder_current;
  double left_velocity = (double)left_encoder_diff / 20.0; // Encoder count to velocity in m/s
  double right_velocity = (double)right_encoder_diff / 20.0;
  left_encoder_msg.linear.x = left_velocity;
  right_encoder_msg.linear.x = right_velocity;
  left_encoder_pub.publish(&left_encoder_msg);
  right_encoder_pub.publish(&right_encoder_msg);
}
