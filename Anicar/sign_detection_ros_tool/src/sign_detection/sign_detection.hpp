#pragma once

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>


#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/dnn.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <ros/time.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include "sign_detection_ros_tool/SignDetectionInterface.h"

#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>


namespace sign_detection_ros_tool {

    using namespace std;

    class SignDetection {

        using Interface = SignDetectionInterface;
        using Msg = std_msgs::Header;

    public:
        SignDetection(ros::NodeHandle, ros::NodeHandle);
        ros::Subscriber image_subscriber;
        ros::Subscriber subscriber_camera_time_;
        ros::Subscriber depth_image_subscriber;
        ros::Subscriber camera_factor_subscriber;
        ros::Publisher pub_signs;
        ros::Publisher pub_stop;
        ros::Publisher pub_go_direction;

        void postprocess(cv::Mat& frame,const vector<cv::Mat>& out);
        void drawPred(int classId,float conf,int left,int top,int right,int bottom,cv::Mat& frame);
        //Get names of detected signs
        vector<cv::String> getOutputNames(const cv::dnn::Net& net);
    private:
        
        void callbackSubscriber(const Msg::ConstPtr& msg);
        void reconfigureRequest(const Interface::Config&, uint32_t);
        void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
        void imageCb(const sensor_msgs::ImageConstPtr& msg);
        void setupIntrinsics(const sensor_msgs::CameraInfo::ConstPtr& camera_info);

        sensor_msgs::CameraInfo camera_time_globa;
        Interface interface_;
        dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    };

} // namespace sign_detection_ros_tool
