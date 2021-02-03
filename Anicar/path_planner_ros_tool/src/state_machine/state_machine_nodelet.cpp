#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "state_machine.hpp"

namespace path_planner_ros_tool {

class StateMachineNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<StateMachine>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<StateMachine> impl_;
};
} // namespace path_planner_ros_tool

PLUGINLIB_EXPORT_CLASS(path_planner_ros_tool::StateMachineNodelet, nodelet::Nodelet);
