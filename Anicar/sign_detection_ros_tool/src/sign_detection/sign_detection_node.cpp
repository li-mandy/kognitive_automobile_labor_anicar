#include "sign_detection.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "sign_detection_node");

    sign_detection_ros_tool::SignDetection sign_detection(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
