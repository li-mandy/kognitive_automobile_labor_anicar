#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>

#include "anicar_longitudinal_control_ros_tool/LongitudinalControllerParameters.h"

namespace anicar_longitudinal_control_ros_tool {

    class LongitudinalController {

        using Parameters = LongitudinalControllerParameters;
        using Config = LongitudinalControllerConfig;
        using ReconfigureServer = dynamic_reconfigure::Server<Config>;

    public:
        LongitudinalController(ros::NodeHandle, ros::NodeHandle);

    private:
        void pathCurvatureCallback(const std_msgs::Float64 msg_pc);
        void pathProgressCallback(const std_msgs::Int16 msg_pp);
        void drivingStateCallback(const std_msgs::Int8 msg_ds);
        void controlLoopCallback(const ros::TimerEvent& timer_event);
        void reconfigureRequest(const Config&, uint32_t);

        double curv;
        int path_entries_to_end;
        int driving_state;

        ros::Publisher motor_command_publisher_;
        ros::Subscriber path_curv_subscriber_;
        ros::Subscriber path_progress_subscriber_;
        ros::Subscriber driving_state_subscriber_;
        ros::Timer control_loop_timer_;
        Parameters params_;
        ReconfigureServer reconfigureServer_;
    };
} // namespace anicar_longitudinal_control_ros_tool
