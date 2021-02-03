#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include "Dsl.h"
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int8.h>
#include <tinyxml.h>

#include "path_planner_ros_tool/PathPlannerInterface.h"

namespace path_planner_ros_tool {

class PathPlanner {

    using Interface = PathPlannerInterface;

    using Msg = std_msgs::Header;

public:
    PathPlanner(ros::NodeHandle, ros::NodeHandle);

    static const int GOAL_UP;
    static const int GOAL_DOWN;
    static const int GOAL_LEFT;
    static const int GOAL_RIGHT;
    static const int GOAL_BEFORE_UE;
    static const int GOAL_UE;
    static const int GOAL_ZEBRA;

    static const int GRID_WIDTH;
    static const int GRID_HEIGHT;

    static const bool WITH_INTERPOLATION;

    static const int PLANNEDTOLEAVE;
    static const int NOTPLANNED;
    static const int PLANNEDTOGOAL;

private:
    void callbackObstacle(const nav_msgs::OccupancyGrid);
    void callbackGoal(const std_msgs::Int8);
    void callbackMap(const nav_msgs::OccupancyGrid);
    void reconfigureRequest(const Interface::Config&, uint32_t);

    bool inRange(geometry_msgs::Point, double);
    bool importGoals(geometry_msgs::Point*& , std::string );
    bool planTo(nav_msgs::OccupancyGrid, geometry_msgs::Point);
    std::vector<int8_t> mergeGrids(std::vector<int8_t> , std::vector<int8_t> );
    std::vector<int8_t> inflate(std::vector<int8_t> ,int );
    std::vector<int8_t> importXML(std::string);

    Interface interface_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_obstacle_;
    ros::Publisher pub_path_;
    ros::Publisher pub_map_;
    ros::Publisher pub_obstmap_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    geometry_msgs::Point *goalCoordinates_;
    geometry_msgs::Point *leaveIntersectionCoordinates_;


    Dsl* pathPlanner_;
    int goal_;
    int state_;
    bool mapReceived_;

    nav_msgs::OccupancyGrid origMap_;
    std::vector<int8_t>* obstacles_;
    std::vector<int8_t>* costMaps_;
    std::vector<int8_t>* blockages_;

    int seq;
    ros::Time time;


    //Path interpolation
    void interpolatePath(const nav_msgs::Path&, nav_msgs::Path&);
    void interpolatePath(const std::vector<geometry_msgs::PoseStamped>&, std::vector<geometry_msgs::PoseStamped>&);
    void interpolatePoint(const std::vector<geometry_msgs::PoseStamped>&, const std::vector<double>&,geometry_msgs::PoseStamped&,double);
    void calcCummulativeDistances(const std::vector<geometry_msgs::PoseStamped>, std::vector<double>&);
    double calcTotalDistance(const std::vector<geometry_msgs::PoseStamped>&);
    double calcDistance(const std::vector<geometry_msgs::PoseStamped>&,unsigned int);
    double calcAlphaCoeff(const std::vector<geometry_msgs::PoseStamped>, const std::vector<double>, unsigned int,double );
    double calcBetaCoeff(const std::vector<geometry_msgs::PoseStamped>,const std::vector<double>, unsigned int,double);
    double calcGammaCoeff(const std::vector<geometry_msgs::PoseStamped>, const std::vector<double>, unsigned int, double);
    double calcDeltaCoeff(const std::vector<geometry_msgs::PoseStamped>, const std::vector<double>, unsigned int, double);
    double calcRelativeDistance(const std::vector<double>&, const unsigned int, const double);
    void calcPointGradient(const std::vector<geometry_msgs::PoseStamped>&, const std::vector<double>&, unsigned int,std::vector<double>&);
    unsigned int findGroup(const std::vector<double>&, double);

    double pointsPerUnit_ = 5.0;
    unsigned int skipPoints_ = 0;
    bool useEndConditions_ = true;
    bool useMiddleConditions_ = false;


};
} // namespace path_planner_ros_tool
