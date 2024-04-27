#include "crobot_control/crobot_control.h"

using namespace crobot;
using namespace crobot_control;

namespace crobot_ros {

Crobot_Control::Crobot_Control(ros::NodeHandle nh,
                               ros::NodeHandle nh_private,
                               Controller_Callbacks& cbs)
    : nh_(nh),
      nh_private_(nh_private),
      controller_(cbs) {}

Crobot_Control::~Crobot_Control() {
    thread_end_ = true;
    get_odometry_thread_.join();
    get_imu_temperature_thread_.join();
    get_imu_data_thread_.join();
    get_battery_voltage_thread_.join();
}

void Crobot_Control::init() {
    std::string port_name = "/dev/smart_car";
    nh_private_.getParam("port_name", port_name);

    set_pid_interval_server_ = nh_private_.advertiseService(
        "set_pid_interval", &Crobot_Control::set_pid_interval_func, this);
    set_count_per_rev_server_ = nh_private_.advertiseService(
        "set_count_per_rev", &Crobot_Control::set_count_per_rev_func, this);
    set_correction_factor_server_ = nh_private_.advertiseService(
        "set_correction_factor", &Crobot_Control::set_correction_factor_func, this);
    reset_odometry_server_ = nh_private_.advertiseService(
        "reset_odometry", &Crobot_Control::reset_odometry_func, this);
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 10, &Crobot_Control::cmd_vel_callback, this);

    controller_.init(port_name.c_str(),
                    itas109::BaudRate115200,
                    itas109::ParityNone,
                    itas109::DataBits8,
                    itas109::StopOne,
                    itas109::FlowNone);
}

bool Crobot_Control::start() {
    init();

    if (!controller_.open()) {
        ROS_ERROR("Failed to start crobot control\n");
        return false;
    }

    // Reset odometry
    controller_.send_request(Reset_Odometry_Req{});

    get_odometry_thread_ = boost::thread{
        boost::bind(&Crobot_Control::get_odometry_func, this)};
    get_imu_temperature_thread_ = boost::thread{
        boost::bind(&Crobot_Control::get_imu_temperature_func, this)};
    get_imu_data_thread_ = boost::thread{
        boost::bind(&Crobot_Control::get_imu_data_func, this)};
    get_ultrasonic_range_thread_ = boost::thread{
        boost::bind(&Crobot_Control::get_ultrasonic_range_func, this)};
    get_battery_voltage_thread_ = boost::thread{
        boost::bind(&Crobot_Control::get_battery_voltage_func, this)};

    return true;
}

void Crobot_Control::cmd_vel_callback(
    const geometry_msgs::Twist::ConstPtr& msg) {
    controller_.send_request(
        Set_Velocity_Req{static_cast<float>(msg->linear.x),
                         static_cast<float>(msg->linear.y),
                         static_cast<float>(msg->angular.z)});
}

bool Crobot_Control::set_pid_interval_func(
    PidInterval::Request& req, PidInterval::Response& resp) {
    controller_.send_request(Set_PID_Interval_Req{req.pid_interval});
    resp.success = true;
    return true;
}

bool Crobot_Control::set_count_per_rev_func(
    CountPerRev::Request& req, CountPerRev::Response& resp) {
    controller_.send_request(Set_Count_Per_Rev_Req{req.cpr});
    resp.success = true;
    return true;
}


bool Crobot_Control::set_correction_factor_func(
    CorrectionFactor::Request& req,
    CorrectionFactor::Response& resp) {
    controller_.send_request(Set_Correction_Factor_Req{req.linear,
                                                       req.angular});
    resp.success = true;
    return true;
}

bool Crobot_Control::reset_odometry_func(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& resp) {
    controller_.send_request(Reset_Odometry_Req{});
    resp.success = true;
    return true;
}

void Crobot_Control::get_odometry_func() {
    ros::Rate rate(20);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(Get_Odometry_Req{});
    }
}

void Crobot_Control::get_imu_temperature_func() {
    ros::Rate rate(1);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(Get_IMU_Temperature_Req{});
    }
}

void Crobot_Control::get_imu_data_func() {
    ros::Rate rate(100);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(Get_IMU_Data_Req{});
    }
}

void Crobot_Control::get_ultrasonic_range_func() {
    ros::Rate rate(20);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(Get_Ultrasonic_Range_Req{});
    }
}

void Crobot_Control::get_battery_voltage_func() {
    ros::Rate rate(1);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(Get_Battery_Voltage_Req{});
    }
}

} // namespace crobot_ros
