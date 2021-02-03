#include "obstacle_detection.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "obstacle_detection_node");

    obstacle_detection_ros_tool::ObstacleDetection obstacle_detection(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
