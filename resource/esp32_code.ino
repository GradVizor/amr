#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <PID_v1.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3_stamped.h>
#include <nav_msgs/msg/odometry.h>
// #include <tf2/LinearMath/Quaternion.h>
#include <cmath>

#define LOOPTIME 10
#define LED_PIN 23
#define ENA 2     //left motor - pwm
#define IN_1 26   //left motor - backward
#define IN_2 25   //left motor - forward
#define IN_3 23   //right motor - backward
#define IN_4 21   //right motor - forward
#define ENB 18    //right motor - pwm

#define EncLA 27  // encoder 1
#define EncLB 14
#define EncRA 13  // encoder 2
#define EncRB 12

// Define robot properties
#define WHEEL_RADIUS 0.07   // 3 cm radius
#define WHEEL_BASE 0.31      // Distance between wheels (30 cm)

double yaw = 0;
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_publisher_t odom_publisher;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

float linear, angular;
unsigned long currentMillis, prevMillis;
volatile long encoder0Pos = 0, encoder1Pos = 0;
double demand_speed_left, demand_speed_right;
float encoder0Diff, encoder1Diff, encoder0Prev, encoder1Prev;
double speed_act_left = 0, speed_act_right = 0;
double x = 0.0, y = 0.0, theta = 0.0; // Robot pose
double vx, vy = 0.0, vth; // Linear and angular velocities
unsigned long prevOdomTime;

nav_msgs__msg__Odometry odom_msg;

// Encoder interrupt service routines (as in your original code)
// ************** Encoder 1 *********************
void change_left_a() {
  if (digitalRead(EncLA) == HIGH) {
    if (digitalRead(EncLB) == LOW) {
      encoder0Pos = encoder0Pos + 1;  // CW
    } else {
      encoder0Pos = encoder0Pos - 1;  // CCW
    }
  } else {
    if (digitalRead(EncLB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;  // CW
    } else {
      encoder0Pos = encoder0Pos - 1;  // CCW
    }
  }
}
void change_left_b() {
  if (digitalRead(EncLB) == HIGH) {
    if (digitalRead(EncLA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;  // CW
    } else {
      encoder0Pos = encoder0Pos - 1;  // CCW
    }
  } else {
    if (digitalRead(EncLA) == LOW) {
      encoder0Pos = encoder0Pos + 1;  // CW
    } else {
      encoder0Pos = encoder0Pos - 1;  // CCW
    }
  }
}
// ************** Encoder 2 *********************
void change_right_a() {
  if (digitalRead(EncRA) == HIGH) {
    if (digitalRead(EncRB) == LOW) {
      encoder1Pos = encoder1Pos - 1;  // CW
    } else {
      encoder1Pos = encoder1Pos + 1;  // CCW
    }
  } else {
    if (digitalRead(EncRB) == HIGH) {
      encoder1Pos = encoder1Pos - 1;  // CW
    } else {
      encoder1Pos = encoder1Pos + 1;  // CCW
    }
  }
}
void change_right_b() {
  if (digitalRead(EncRB) == HIGH) {
    if (digitalRead(EncRA) == HIGH) {
      encoder1Pos = encoder1Pos - 1;  // CW
    } else {
      encoder1Pos = encoder1Pos + 1;  // CCW
    }
  } else {
    if (digitalRead(EncRA) == LOW) {
      encoder1Pos = encoder1Pos - 1;  // CW
    } else {
      encoder1Pos = encoder1Pos + 1;  // CCW
    }
  }
}
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Motor control functions (as in your original code)
void single_forward(int MotorPWM, int MotorA, int MotorB, int value) {
  digitalWrite(MotorA, LOW);
  digitalWrite(MotorB, HIGH);
  analogWrite(MotorPWM, value);
}
void single_backward(int MotorPWM, int MotorA, int MotorB, int value) {
  digitalWrite(MotorA, HIGH);
  digitalWrite(MotorB, LOW);
  analogWrite(MotorPWM, value);
}
void rotate(std::string direction, int value) {
  if (direction == "left") {
    if (value >= 0) {
      int val = map(value, 0, 100, 0, 255);
      single_forward(ENA, IN_1, IN_2, val);
    } else {
      int val = map(value, 0, -100, 0, 255);
      single_backward(ENA, IN_1, IN_2, val);
    }
  } else if (direction == "right") {
    if (value >= 0) {
      int val = map(value, 0, 100, 0, 255);
      single_forward(ENB, IN_3, IN_4, val);
    } else {
      int val = map(value, 0, -100, 0, 255);
      single_backward(ENB, IN_3, IN_4, val);
    }
  }
}

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  linear = msg->linear.x;
  angular = msg->angular.z;
}

void calculate_odometry() {
  // Time difference in seconds
  unsigned long current_time = millis();
  double dt = (current_time - prevOdomTime) / 1000.0;
  prevOdomTime = current_time;

  // Compute linear and angular velocities
  double v_left = speed_act_left * WHEEL_RADIUS;
  double v_right = speed_act_right * WHEEL_RADIUS;

  vx = (v_left + v_right) / 2.0; // Linear velocity
  vth = (v_right - v_left) / WHEEL_BASE; // Angular velocity

  // Update robot's position and orientation
  double delta_x = vx * cos(theta) * dt;
  double delta_y = vx * sin(theta) * dt;
  double delta_theta = vth * dt;

  x += delta_x;
  y += delta_y;
  theta += delta_theta;
}

void publish_odometry() {
  odom_msg.header.stamp.sec = millis() / 1000;
  odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.child_frame_id.data = (char*)"base_link";

  // Set the position
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  // Quaternion for orientation
  // tf2::Quaternion q;
  // Calculate angular velocity (omega) and update yaw
  double omega = (speed_act_right - speed_act_left) / WHEEL_BASE;
  yaw += omega * (LOOPTIME / 1000.0);  // Update yaw (radians)
  double q_x = 0;
  double q_y = 0;
  double q_z = sin(yaw / 2.0);
  double q_w = cos(yaw / 2.0);

  odom_msg.pose.pose.orientation.x = q_x;
  odom_msg.pose.pose.orientation.y = q_y;
  odom_msg.pose.pose.orientation.z = q_z;
  odom_msg.pose.pose.orientation.w = q_w;

  // Set the velocity
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.angular.z = vth;

  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void setup() {
  set_microros_transports();
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(EncLA, INPUT_PULLUP);
  pinMode(EncLB, INPUT_PULLUP);
  pinMode(EncRA, INPUT_PULLUP);
  pinMode(EncRB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EncLA), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncLB), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncRA), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncRB), change_right_b, CHANGE);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Subscriber for cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // Publisher for odom
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  prevOdomTime = millis();
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Update motor speeds
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME) {
    prevMillis = currentMillis;

    encoder0Diff = encoder0Pos - encoder0Prev;
    encoder1Diff = encoder1Pos - encoder1Prev;

    speed_act_left = (encoder0Diff / 39.65); // Calculate actual speed (ticks to m/s)
    speed_act_right = (encoder1Diff / 39.65);

    encoder0Prev = encoder0Pos;
    encoder1Prev = encoder1Pos;

    calculate_odometry();
    publish_odometry();
  }

  // Set motor speeds based on received cmd_vel
  float speed_left = linear - (angular * (WHEEL_BASE / 2));
  float speed_right = linear + (angular * (WHEEL_BASE / 2));

  rotate("left", 200 * speed_left);
  rotate("right", 200 * speed_right);
}

