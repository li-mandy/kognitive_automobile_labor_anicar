#include "state_machine.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "state_machine_node");

    path_planner_ros_tool::StateMachine state_machine(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
