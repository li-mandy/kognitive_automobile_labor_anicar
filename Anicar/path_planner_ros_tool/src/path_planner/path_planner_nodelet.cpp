#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "path_planner.hpp"

namespace path_planner_ros_tool {

class PathPlannerNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<PathPlanner>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<PathPlanner> impl_;
};
} // namespace path_planner_ros_tool

PLUGINLIB_EXPORT_CLASS(path_planner_ros_tool::PathPlannerNodelet, nodelet::Nodelet);
