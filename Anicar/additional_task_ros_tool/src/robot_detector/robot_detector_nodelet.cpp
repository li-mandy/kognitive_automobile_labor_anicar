#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "robot_detector.hpp"

namespace additional_task_ros_tool {

class RobotDetectorNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<RobotDetector>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<RobotDetector> impl_;
};
} // namespace additional_task_ros_tool

PLUGINLIB_EXPORT_CLASS(additional_task_ros_tool::RobotDetectorNodelet, nodelet::Nodelet);
