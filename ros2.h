#ifndef ROS2_H
#define ROS2_H

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/int32.h>

// Publishers
extern rcl_publisher_t imu_publisher;
extern sensor_msgs__msg__Imu imu_msg;

extern rcl_publisher_t laser_publisher;
extern sensor_msgs__msg__Range laser_msg;

extern rcl_publisher_t encoder_publisher;
extern std_msgs__msg__Int32 encoder_msg;

// Subscriber
extern rcl_subscription_t rotation_subscriber;

// Setup function for ROS2 communication
void setupROS2();

// Debug help
void printError(const char* fn, rcl_ret_t temp_rc);
void error_loop(const char* fn, rcl_ret_t temp_rc);

// Functions to publish data
void publishIMUData();
void publishRangeData(sensor_msgs__msg__Range &range_msg);
void publishEncoderData(std_msgs__msg__Int32 &encoder_msg);

// Callback function for the subscriber
void rotation_callback(const void * msgin);

// Debugging
void printIMUMessage(const sensor_msgs__msg__Imu &imu_msg);

#endif