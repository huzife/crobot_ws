#ifndef CROBOT_CONTROL_H
#define CROBOT_CONTROL_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "crobot/controller.h"
#include "crobot/controller_callbacks.h"
#include "crobot_control/CorrectionFactor.h"
#include "crobot_control/CountPerRev.h"
#include "crobot_control/PidInterval.h"
#include "std_srvs/Trigger.h"
#include <boost/thread.hpp>

namespace crobot_ros {

class Crobot_Control {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::ServiceServer set_pid_interval_server_;
    ros::ServiceServer set_count_per_rev_server_;
    ros::ServiceServer set_correction_factor_server_;
    ros::ServiceServer reset_odometry_server_;
    ros::Subscriber cmd_vel_sub_;

    crobot::Controller controller_;

    bool thread_end_ = false;
    boost::thread get_odometry_thread_;
    boost::thread get_imu_temperature_thread_;
    boost::thread get_imu_data_thread_;
    boost::thread get_ultrasonic_range_thread_;
    boost::thread get_battery_voltage_thread_;

public:
    Crobot_Control(ros::NodeHandle nh,
                   ros::NodeHandle nh_private,
                   crobot::Controller_Callbacks& cbs);
    ~Crobot_Control();

    void init();
    bool start();

private:
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    bool set_pid_interval_func(
        crobot_control::PidInterval::Request& req,
        crobot_control::PidInterval::Response& resp);
    bool set_count_per_rev_func(
        crobot_control::CountPerRev::Request& req,
        crobot_control::CountPerRev::Response& resp);
    bool set_correction_factor_func(
        crobot_control::CorrectionFactor::Request& req,
        crobot_control::CorrectionFactor::Response& resp);
    bool reset_odometry_func(
        std_srvs::Trigger::Request& req,
        std_srvs::Trigger::Response& resp);
    void get_odometry_func();
    void get_imu_temperature_func();
    void get_imu_data_func();
    void get_ultrasonic_range_func();
    void get_battery_voltage_func();
};

} // namespace crobot_ros

#endif // CROBOT_CONTROL_H
