#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

//for transformation
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Core>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

#include <iostream>
#include <boost/shared_ptr.hpp>

//PCL specific inludes
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
#include <pcl/features/normal_3d.h>

//Filters and Downsampling
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/radius_outlier_removal.h>

//Clustering
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

//occupancy grid map
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose.h>

//filter the noise for Map
#include <algorithm>
#include <vector>

//for time
#include <pcl/console/time.h>

#include "obstacle_detection_ros_tool/ObstacleDetectionInterface.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;


namespace obstacle_detection_ros_tool {

    class ObstacleDetection {

        using Interface = ObstacleDetectionInterface;
        using Msg = std_msgs::Header;

    public:
        ObstacleDetection(ros::NodeHandle, ros::NodeHandle);


    private:
        void callbackSubscriber(const sensor_msgs::PointCloud2ConstPtr& input);
        void reconfigureRequest(const Interface::Config&, uint32_t);
        Interface interface_;
        dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

        //my function
        void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
        const pcl::PointCloud<pcl::PointXYZ> & transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);
        void setupIntrinsics(const sensor_msgs::CameraInfo::ConstPtr&);
        void mapImport(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, nav_msgs::OccupancyGrid& output);
        std::vector<int8_t> inflate(std::vector<int8_t>,int rad);
        sensor_msgs::PointCloud2ConstPtr cloud_msg;

        void process();

        double downsample_leaf_size = 0.07;
        bool transform_set_;
        //for grid map
        nav_msgs::OccupancyGrid grid_pub;
        //nav_msgs::OccupancyGrid grid_pub_temp;

        nav_msgs::MapMetaData info;
        geometry_msgs::Pose pose;

        //for coordinate transformation
        sensor_msgs::CameraInfo camera_info_globa;
        Eigen::Affine3d kinect_ir_2_camera_top_tranform_;
        Eigen::Affine3d vehicle_2_camera_top_transform_;
        Eigen::Affine3d stargazer_2_world_transform_;


        //publisher and subscriber
        ros::Subscriber subscriber_input;
        ros::Subscriber subscriber_camera_info_;

        ros::Publisher pub_obstacles;
        ros::Publisher pub_ground_plane;
        ros::Publisher pub_obstacles_proj;
        ros::Publisher pub_occupancy_grid_map;


        //for tf2
        tf2_ros::Buffer tf_buffer_kinect_ir_2_vehicle_;
        tf2_ros::TransformListener transform_listener_camera_extrinsics_a_{tf_buffer_kinect_ir_2_vehicle_};
        tf2_ros::Buffer tf_buffer_vehicle_2_camera_top_;
        tf2_ros::TransformListener transform_listener_vehicle_2_camera_top_{tf_buffer_vehicle_2_camera_top_};
        tf2_ros::Buffer tf_buffer_camera_top_2_stargazer_;
        tf2_ros::TransformListener transform_listener_camera_top_2_stargazer_{tf_buffer_camera_top_2_stargazer_};
        tf2_ros::Buffer tf_buffer_stargazer_2_world_;
        tf2_ros::TransformListener transform_listener_stargazer_2_world_{tf_buffer_stargazer_2_world_};
        tf2_ros::TransformBroadcaster tfBroadcaster_;

    };

} // namespace obstacle_detection_ros_tool
