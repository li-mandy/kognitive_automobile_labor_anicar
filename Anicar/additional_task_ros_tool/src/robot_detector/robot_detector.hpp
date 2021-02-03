#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>


#include <mutex>
#include <iostream>
#include <chrono>

#include <cv_bridge/cv_bridge.h>

#include <shape_msgs/Mesh.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/io.h>

//Filters and Downsampling
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//Clustering
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stdlib.h>
#include <std_msgs/Int8.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/AccelWithCovariance.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/common/angles.h"
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <visualization_msgs/Marker.h>

#include <automated_driving_msgs/MotionState.h>
#include <automated_driving_msgs/ObjectState.h>
#include <automated_driving_msgs/ObjectStateArray.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include "additional_task_ros_tool/RobotDetectorInterface.h"

namespace additional_task_ros_tool {

    using Msg = std_msgs::Header;
    using Img = sensor_msgs::Image;
    using Info = sensor_msgs::CameraInfo;
    using PCL2 = sensor_msgs::PointCloud2;
    using Point = geometry_msgs::Point;
    using PoseWithCovariance = geometry_msgs::PoseWithCovariance;
    using TwistWithCovariance = geometry_msgs::TwistWithCovariance;
    using AccelWithCovariance = geometry_msgs::AccelWithCovariance;
    using Marker = visualization_msgs::Marker;
    using MotionState = automated_driving_msgs::MotionState;
    using Hull = shape_msgs::Mesh;
    using ObjectState = automated_driving_msgs::ObjectState;
    using ObjectStateArray = automated_driving_msgs::ObjectStateArray;
    using SynchronizedKinectImages = message_filters::sync_policies::ApproximateTime<Img, Img>;


    struct hesse_normal_form {
        cv::Point2d normal_vector;
        double distance_from_origin;

        std::pair<Point, Point> boundaries;
        Point end_of_line;
        double length;

        pcl::PointCloud<pcl::PointXYZI>::Ptr points{new pcl::PointCloud<pcl::PointXYZI>};

        bool fit = false;
    };

    struct line_set {
        hesse_normal_form lines[2];

        Point intersection;
    };

    struct box_object {
        MotionState motion_state;
        Hull hull;

        double length;
        double width;
        double height;

        cv::Point2d orientation;
    };
class RobotDetector {

    using Interface = RobotDetectorInterface;

    using Msg = std_msgs::Header;

//    using namespace std;

public:
    RobotDetector(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackSubscriber(const Msg::ConstPtr& msg);
    void reconfigureRequest(const Interface::Config&, uint32_t);
    void callback_depthimage(const sensor_msgs::PointCloud2::ConstPtr& msg2);
    const pcl::PointCloud<pcl::PointXYZ> & transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);
    void setupSubscriber(ros::NodeHandle&);
    void setupPublisher(ros::NodeHandle&);
    void setupIntrinsics(const sensor_msgs::CameraInfo::ConstPtr&);
    void mapImport(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);

    void makeDecision( pcl::PointCloud<pcl::PointXYZ>::Ptr &input, ros::Time);
    void prediction(double, double, ros::Time);
    void update(Eigen::Vector2d&, Eigen::Vector4d& );
//    Eigen::Vector4d KalmanFiltering::transformMeasurement(Eigen::Vector4d);
//        void prediction(double, double, ros::Time);
     void init(const Eigen::VectorXd& , ros::Time);

        bool measurement_data_;
        bool time_initialized_;
    bool transform_set_ ;
        Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

        sensor_msgs::CameraInfo camera_info_globa;
        nav_msgs::OccupancyGrid grid_pub;
        nav_msgs::MapMetaData info;
        geometry_msgs::Pose pose;
        ros::Subscriber subscriber_camera_info_;

    ros::Publisher pub_;
        ros::Publisher pub_obstacles;
        ros::Publisher pub_ground_plane;


    std::unique_ptr<message_filters::Subscriber<Img>> subscriber_image_;
    std::unique_ptr<message_filters::Subscriber<Img>> subscriber_depth_;
    std::unique_ptr<message_filters::Subscriber<PCL2>> subscriber_pcl_;

    std::unique_ptr<message_filters::Synchronizer<SynchronizedKinectImages>> kinect_image_synchronizer_;
    std::mutex m_;
        double leaf_size=0.05;

        double focal_length_;
        double u0_, v0_;
        double dt_;
        ros::Time old_time;
        ros::Time new_time;

//        Eigen::Matrix4d P_;
        Eigen::Vector4d x_, x_hat, x_new;

            // int n=4; int m=2;
        Eigen::MatrixXd A_; // System dynamics matrix
        Eigen::MatrixXd H_; // Output matrix
        Eigen::MatrixXd Q_; // Process noise covariance
        Eigen::MatrixXd R_; // Measurement noise covariance
        Eigen::MatrixXd P_; // Estimate error covariance
        Eigen::MatrixXd K_;

        Eigen::Vector2d measurement;

        Eigen::MatrixXd M_;

        int dec_state;
        std_msgs::Int8 msg2;

        Eigen::Affine3d kinect_ir_2_camera_top_tranform_;
        Eigen::Affine3d vehicle_2_camera_top_transform_;
        Eigen::Affine3d stargazer_2_world_transform_;

/*     cv::Mat structuring_element_0_;
        cv::Mat structuring_element_1_;

        cv::Mat_<uchar> conv_horizontal_;
        cv::Mat_<uchar> conv_vertical_;
        cv::Mat_<uchar> conv_diagonal_lr_;
        cv::Mat_<uchar> conv_diagonal_rl_;
        cv::Mat_<uchar> conv_diagonal_lro_;
        cv::Mat_<uchar> conv_diagonal_rlo_;
        cv::Mat_<uchar> conv_diagonal_lru_;
        cv::Mat_<uchar> conv_diagonal_rlu_;

        std::queue<box_object> object_queue_;
*/
        tf2_ros::Buffer tf_buffer_kinect_ir_2_vehicle_;
        tf2_ros::TransformListener transform_listener_camera_extrinsics_a_{tf_buffer_kinect_ir_2_vehicle_};
        tf2_ros::Buffer tf_buffer_vehicle_2_camera_top_;
        tf2_ros::TransformListener transform_listener_vehicle_2_camera_top_{tf_buffer_vehicle_2_camera_top_};
        tf2_ros::Buffer tf_buffer_camera_top_2_stargazer_;
        tf2_ros::TransformListener transform_listener_camera_top_2_stargazer_{tf_buffer_camera_top_2_stargazer_};
        tf2_ros::Buffer tf_buffer_stargazer_2_world_;
        tf2_ros::TransformListener transform_listener_stargazer_2_world_{tf_buffer_stargazer_2_world_};
        tf2_ros::TransformBroadcaster tfBroadcaster_;
        tf2_ros::Buffer tfBuffer_;
    //tf2_ros::TransformListener tfListener_{tfBuffer_};
    //tf2_ros::TransformBroadcaster tfBroadcaster_;
    ros::Publisher cloud_pub;
    ros::Publisher chatter_pub;
    ros::Publisher chatter_pub1;
    ros::Publisher pub_decision;
};
} // namespace additional_task_ros_tool
