#include "crobot_control/crobot_control.h"
#include "crobot/robot_base/robot_base.h"

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

    // Set motor param and robot base
    if (!set_motor_param() || !set_robot_base())
        return false;

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

bool Crobot_Control::set_motor_param() {
    int pid_interval;
    int count_per_rev;
    bool reverse;
    if (!nh_private_.getParam("motor/pid_interval", pid_interval) ||
        !nh_private_.getParam("motor/count_per_rev", count_per_rev) ||
        !nh_private_.getParam("motor/reverse", reverse)) {
        ROS_ERROR("Failed to load motor parameters, please check config file\n");
        return false;
    }

    controller_.send_request(Set_PID_Interval_Req{static_cast<uint16_t>(pid_interval)});
    controller_.send_request(Set_Motor_Param_Req{static_cast<uint32_t>(count_per_rev), reverse});

    return true;
}

bool Crobot_Control::set_robot_base() {
    int type;
    if (!nh_private_.getParam("robot_base/type", type)) {
        ROS_ERROR("Failed to load robot base type\n");
        return false;
    }

    switch (static_cast<Robot_Base_Type>(type)) {
        case Robot_Base_Type::ROBOT_BASE_2WD: {
            Robot_Base_2WD_Param param;
            if (!nh_private_.getParam("robot_base/radius", param.radius) ||
                !nh_private_.getParam("robot_base/separation", param.separation)) {
                ROS_ERROR("Failed to load 2wd parameters, please check config file\n");
                return false;
            }
            controller_.send_request(Set_Robot_Base_2WD_Req{param});
            break;
        }
        case Robot_Base_Type::ROBOT_BASE_3WO: {
            Robot_Base_3WO_Param param;
            if (!nh_private_.getParam("robot_base/radius", param.radius) ||
                !nh_private_.getParam("robot_base/distance", param.distance)) {
                ROS_ERROR("Failed to load 3wo parameters, please check config file\n");
                return false;
            }
            controller_.send_request(Set_Robot_Base_3WO_Req{param});
            break;
        }
        case Robot_Base_Type::ROBOT_BASE_4WD: {
            Robot_Base_4WD_Param param;
            if (!nh_private_.getParam("robot_base/radius", param.radius) ||
                !nh_private_.getParam("robot_base/separation", param.separation)) {
                ROS_ERROR("Failed to load 4wd parameters, please check config file\n");
                return false;
            }
            controller_.send_request(Set_Robot_Base_4WD_Req{param});
            break;
        }
        case Robot_Base_Type::ROBOT_BASE_4MEC: {
            Robot_Base_4MEC_Param param;
            if (!nh_private_.getParam("robot_base/radius", param.radius) ||
                !nh_private_.getParam("robot_base/distance_x", param.distance_x) ||
                !nh_private_.getParam("robot_base/distance_y", param.distance_y)) {
                ROS_ERROR("Failed to load 4mec parameters, please check config file\n");
                return false;
            }
            controller_.send_request(Set_Robot_Base_4MEC_Req{param});
            break;
        }
    }

    return true;
}

void Crobot_Control::cmd_vel_callback(
    const geometry_msgs::Twist::ConstPtr& msg) {
    controller_.send_request(
        Set_Velocity_Req{static_cast<float>(msg->linear.x),
                         static_cast<float>(msg->linear.y),
                         static_cast<float>(msg->angular.z)});
}

bool Crobot_Control::set_correction_factor_func(
    CorrectionFactor::Request& req,
    CorrectionFactor::Response& resp) {
    controller_.send_request(Set_Correction_Factor_Req{req.linear_x,
                                                       req.linear_y,
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
