#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Point.h>
#include <tinyxml.h>
#include <geometry_msgs/PointStamped.h>

#include "path_planner_ros_tool/StateMachineInterface.h"

namespace path_planner_ros_tool {

class StateMachine {

    using Interface = StateMachineInterface;

    using Msg = std_msgs::Header;

public:
    StateMachine(ros::NodeHandle, ros::NodeHandle);

    static const int INTERSECTION;
    static const int UNKNOWN_ENVIRONMENT;
    static const int ENTERING_INTERSECTION;
    static const int DRIVE;
    static const int STARTING;

    static const int GOAL_UP;
    static const int GOAL_DOWN;
    static const int GOAL_LEFT;
    static const int GOAL_RIGHT;
    static const int GOAL_BEFORE_UE;
    static const int GOAL_UE;
    static const int GOAL_ZEBRA;



private:
    void callbackStopSign(const std_msgs::Int8);
    void callbackDirectionSign(const std_msgs::Int8);
    void callbackZebra(const std_msgs::Int8 );
    void reconfigureRequest(const Interface::Config&, uint32_t);
    bool inRange(geometry_msgs::Point, double);
    bool importGoals(geometry_msgs::Point*&, std::string);
    void handleIntersection();
    void handleEnteringIntersection();
    void handleDrive();
    void publishCurrentGoal();
    void updateState(int);

    geometry_msgs::Point *goalCoordinates;
    geometry_msgs::Point *beforeGoalCoordinates;

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    ros::Subscriber sub_stop;
    ros::Subscriber sub_direct;
    ros::Subscriber sub_zebra;
    ros::Publisher pub_goal;
    ros::Publisher pub_state;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    struct sign {
        ros::Time time;
        int direction;
        static const int LEFT = 2;
        static const int RIGHT = 1;
        static const int FORWARD =0;
        bool hasToStop;
        bool obsolete;
    } lastDirectionSign_;

    int state_;
    int goal_;
    int lastGoal_;
};
} // namespace path_planner_ros_tool
