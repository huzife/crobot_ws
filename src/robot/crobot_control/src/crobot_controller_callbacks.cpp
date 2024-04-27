#include "crobot_control/crobot_controller_callbacks.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace crobot;

namespace crobot_ros {

Crobot_Control_Callbacks::Crobot_Control_Callbacks(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private) {
    init();
}

void Crobot_Control_Callbacks::set_pid_interval_callback() {}
void Crobot_Control_Callbacks::set_velocity_callback() {}
void Crobot_Control_Callbacks::set_count_per_rev_callback() {}
void Crobot_Control_Callbacks::set_correction_factor_callback() {}

void Crobot_Control_Callbacks::get_odometry_callback(const Get_Odometry_Resp& resp) {
    ros::Time current_time = ros::Time::now();

    // odom topic
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = resp.position_x;
    odom.pose.pose.position.y = resp.position_y;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, resp.direction);
    odom.pose.pose.orientation = tf2::toMsg(orientation);

    odom.twist.twist.linear.x = resp.linear_x;
    odom.twist.twist.linear.y = resp.linear_y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = resp.angular_z;

    odom_pub_.publish(odom);

    // tf
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = current_time;
    tfs.header.frame_id = "odom";
    tfs.child_frame_id = "base_footprint";

    tfs.transform.translation.x = resp.position_x;
    tfs.transform.translation.y = resp.position_y;
    tfs.transform.translation.z = 0.0;
    tfs.transform.rotation = odom.pose.pose.orientation;

    odom_base_tb_.sendTransform(tfs);
}

void Crobot_Control_Callbacks::reset_odometry_callback() {}

void Crobot_Control_Callbacks::get_imu_temperature_callback(const Get_IMU_Temperature_Resp& resp) {
    std_msgs::Float32 temperature;
    temperature.data = resp.temperature;

    imu_temperature_pub_.publish(temperature);
}

void Crobot_Control_Callbacks::get_imu_data_callback(const Get_IMU_Data_Resp& resp) {
    sensor_msgs::Imu imu_raw_data;
    imu_raw_data.header.stamp = ros::Time::now();
    imu_raw_data.header.frame_id = "imu_link";

    imu_raw_data.linear_acceleration.x = resp.accel_x * 9.7833;
    imu_raw_data.linear_acceleration.y = resp.accel_y * 9.7833;
    imu_raw_data.linear_acceleration.z = resp.accel_z * 9.7833;
    imu_raw_data.angular_velocity.x = resp.angular_x * M_PI / 180.0;
    imu_raw_data.angular_velocity.y = resp.angular_y * M_PI / 180.0;
    imu_raw_data.angular_velocity.z = resp.angular_z * M_PI / 180.0;

    imu_raw_data_pub_.publish(imu_raw_data);
}

void Crobot_Control_Callbacks::get_ultrasonic_range_callback(const Get_Ultrasonic_Range_Resp& resp) {
    std_msgs::UInt16 range;
    range.data = resp.range;

    ultrasonic_range_pub_.publish(range);
}

void Crobot_Control_Callbacks::get_battery_voltage_callback(const Get_Battery_Voltage_Resp& resp) {
    std_msgs::Float32 voltage;
    voltage.data = resp.voltage;

    battery_voltage_pub_.publish(voltage);
}

void Crobot_Control_Callbacks::init() {
    odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odom", 10);
    imu_temperature_pub_ = nh_private_.advertise<std_msgs::Float32>("imu_temperature", 10);
    imu_raw_data_pub_ = nh_private_.advertise<sensor_msgs::Imu>("imu_raw_data", 10);
    ultrasonic_range_pub_ = nh_private_.advertise<std_msgs::UInt16>("ultrasonic_range", 10);
    battery_voltage_pub_ = nh_private_.advertise<std_msgs::Float32>("battery_voltage", 10);
}

} // namespace crobot_ros
