#include "robot_detector.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "robot_detector_node");

    additional_task_ros_tool::RobotDetector robot_detector(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
