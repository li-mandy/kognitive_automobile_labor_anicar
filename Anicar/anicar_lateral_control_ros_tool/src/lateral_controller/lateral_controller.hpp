#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

#include <Eigen/Dense>

#include "anicar_lateral_control_ros_tool/LateralControllerParameters.h"
#include <motor_interface_ros_tool/MotorCommand.h>

namespace anicar_lateral_control_ros_tool {

    class LateralController {

        using Parameters = LateralControllerParameters;
        using Config = LateralControllerConfig;
        using ReconfigureServer = dynamic_reconfigure::Server<Config>;

    public:
        LateralController(ros::NodeHandle, ros::NodeHandle);

    private:
        void pathCallback(const nav_msgs::Path::ConstPtr& msg_p);
        void motorCommandCallback(const motor_interface_ros_tool::MotorCommand::ConstPtr& msg_mc);
        void controlLoopCallback(const ros::TimerEvent& timer_event);
        void reconfigureRequest(const Config&, uint32_t);

        double vel;
        std::vector<Eigen::Affine3d> path_;
        int path_entries_to_end;

        ros::Publisher servo_command_publisher_;
        ros::Publisher path_curv_publisher_;
        ros::Publisher path_progress_publisher_;
        ros::Publisher distance_to_path_end_publisher_;
        ros::Subscriber path_subscriber_;
        ros::Subscriber motor_command_subscriber_;
        ros::Timer control_loop_timer_;
        Parameters params_;
        ReconfigureServer reconfigureServer_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
        tf2_ros::TransformBroadcaster tfBroadcaster_;
    };
} // namespace anicar_lateral_control_ros_tool
