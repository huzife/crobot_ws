#ifndef CROBOT_CONTROLLER_CALLBACKS_H
#define CROBOT_CONTROLLER_CALLBACKS_H

#include "ros/ros.h"
#include "crobot/controller_callbacks.h"
#include "tf2_ros/transform_broadcaster.h"

namespace crobot_ros {

class Crobot_Control_Callbacks: public crobot::Controller_Callbacks {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher odom_pub_;
    ros::Publisher imu_temperature_pub_;
    ros::Publisher imu_raw_data_pub_;
    ros::Publisher ultrasonic_range_pub_;
    ros::Publisher battery_voltage_pub_;
    tf2_ros::TransformBroadcaster odom_base_tb_;

public:
    Crobot_Control_Callbacks(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    void set_pid_interval_callback() override;

    void set_count_per_rev_callback() override;

    void set_correction_factor_callback() override;

    void set_velocity_callback() override;

    void get_odometry_callback(const crobot::Get_Odometry_Resp& resp) override;

    void reset_odometry_callback() override;

    void get_imu_temperature_callback(const crobot::Get_IMU_Temperature_Resp& resp) override;

    void get_imu_data_callback(const crobot::Get_IMU_Data_Resp& resp) override;

    void get_ultrasonic_range_callback(const crobot::Get_Ultrasonic_Range_Resp& resp) override;

    void get_battery_voltage_callback(const crobot::Get_Battery_Voltage_Resp& resp) override;

private:
    void init();
};

} // namespace crobot_ros

#endif // CROBOT_CONTROLLER_CALLBACKS_H
