#include "path_planner.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "path_planner_node");

    path_planner_ros_tool::PathPlanner path_planner(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
