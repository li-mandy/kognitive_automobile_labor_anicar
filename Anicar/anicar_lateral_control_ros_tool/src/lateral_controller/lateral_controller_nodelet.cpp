#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "lateral_controller.hpp"

namespace anicar_lateral_control_ros_tool {

class LateralControllerNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<LateralController>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<LateralController> impl_;
};
} // namespace anicar_lateral_control_ros_tool

PLUGINLIB_EXPORT_CLASS(anicar_lateral_control_ros_tool::LateralControllerNodelet, nodelet::Nodelet);
