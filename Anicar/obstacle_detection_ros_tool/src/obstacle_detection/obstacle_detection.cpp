#include "obstacle_detection.hpp"

namespace obstacle_detection_ros_tool {

    ObstacleDetection::ObstacleDetection(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
            : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

        /**
         * Initialization
         */
        interface_.fromParamServer();
        pub_ground_plane = nhPublic.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/Map_plane",
                                                                        interface_.queue_size);
        pub_obstacles = nhPublic.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/Map_obstacle",
                                                                     interface_.queue_size);
        pub_obstacles_proj = nhPublic.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/Map_obstacleproj",
                                                                          interface_.queue_size);
        pub_occupancy_grid_map = nhPublic.advertise<nav_msgs::OccupancyGrid>("/obstacle_detection/Occupancy_grid_map",
                                                                             interface_.queue_size);

        subscriber_input = nhPublic.subscribe("/clean_points/output", interface_.queue_size,
                                              &ObstacleDetection::cloud_cb, this);
        subscriber_camera_info_ = nhPublic.subscribe("/kinect2/sd/camera_info", interface_.queue_size,
                                                     &ObstacleDetection::setupIntrinsics, this,
                                                     ros::TransportHints().tcpNoDelay());
        /**
         * Set up callbacks for subscribers and reconfigure.
         *
         * New subscribers can be created with "add_subscriber" in "cfg/ObstacleDetection.if file.
         * Don't forget to register your callbacks here!
         */

        reconfigureServer_.setCallback(boost::bind(&ObstacleDetection::reconfigureRequest, this, _1, _2));


        rosinterface_handler::showNodeInfo();
        //for grid map
        info.width = 340;
        info.height = 131;

        grid_pub.data.assign((info.width * info.height), 0);

        grid_pub.header.frame_id = "world";


        pose.position.x = 0;
        pose.position.y = -0.5;
        pose.position.z = 0;
        info.origin = pose;
        info.resolution = 0.05;
        grid_pub.info = info;
        cloud_msg =NULL;
        ros::Rate loop_rate(3);
        while(ros::ok()){
            if(cloud_msg!=NULL){
                process();
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void ObstacleDetection::callbackSubscriber(const sensor_msgs::PointCloud2ConstPtr &input) {

        cloud_cb(input);
    }

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
    void ObstacleDetection::reconfigureRequest(const Interface::Config &config, uint32_t level) {
        interface_.fromConfig(config);
    }

    void ObstacleDetection::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
        this->cloud_msg = cloud_msg;
    }

    void ObstacleDetection::process() {
        //container for original & filtered data
        if(transform_set_) {
            pcl::console::TicToc tt;
            tt.tic();

            pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());

            //conversion
            pcl_conversions::toPCL(*cloud_msg, *cloud);

            pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

            //Passthrough filter : cut off values that are either inside or outside a given user range.
            pcl::PassThrough <pcl::PCLPointCloud2> pass;
            pass.setInputCloud(cloudPtr);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(0, 0.6);      // the accepted interval values are set to (-0.5,0,6).
            pass.filter(*cloud);

            pcl::PCLPointCloud2ConstPtr cloudPtr1(cloud);

            //Voxel Grid Downsample: reduce the number of points – a point cloud dataset, using a voxelized grid approach.
            pcl::VoxelGrid <pcl::PCLPointCloud2> sor;
            sor.setInputCloud(cloudPtr1);
            sor.setLeafSize(downsample_leaf_size, downsample_leaf_size,downsample_leaf_size);//a pcl::VoxelGrid filter is created with a leaf size of 1cm, the input data is passed
            sor.filter(*cloud);

            //Ground Extracion
            //Ground Point Pointer Definition
            pcl::PointCloud <pcl::PointXYZ> groundcloud;
            pcl::fromPCLPointCloud2(*cloud, groundcloud);


            pcl::PointCloud<pcl::PointXYZ>::Ptr groundcloudPtrtemp(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr groundcloudPtrtemp_const(groundcloudPtrtemp);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));

            //Filter out the noise
            pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor2;
            sor2.setInputCloud(groundcloudPtrtemp_const);
            sor2.setMeanK(100);
            //The number of neighbors to analyze for each point is set to 50, and the standard deviation multiplier to 1.
            // What this means is that all points who have a distance larger than 1 standard deviation of the mean distance to the query point will be marked as outliers and removed.
            sor2.setStddevMulThresh(1.0);
            sor2.filter(*cloud_filtered);

            pcl::PointCloud<pcl::PointXYZ>::ConstPtr groundcloudPtr(cloud_filtered);

            //Plane Coefficients and Indices
            pcl::ModelCoefficients::Ptr groundcoeffPtr(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr groundindicPtr(new pcl::PointIndices);
            pcl::PointIndices::Ptr objectindicPtr(new pcl::PointIndices);


            //Output Definition
            pcl::PointCloud<pcl::PointXYZ>::Ptr groundpointsPtr(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));
            pcl::PointCloud<pcl::PointXYZ>::Ptr objectpointsPtr(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudprojPtr(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));
            pcl::PointCloud<pcl::PointXYZ>::Ptr groundhullPtr(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));


            //Find the ground Plane using RANSAC
            //pcl::fromPCLPointCloud2(*temp, *groundcloudPtr);  //from PointCloud2 to PointCloud<PointXYZ>
            pcl::SACSegmentation <pcl::PointXYZ> seg; //Create the segmentation object

            //Mandatory
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.05);//smaller means more noise
            seg.setMaxIterations(150);
            seg.setInputCloud(groundcloudPtr);
            seg.segment(*groundindicPtr, *groundcoeffPtr);

            //Extract ground plane inliers
            pcl::ExtractIndices <pcl::PointXYZ> extractor;
            extractor.setInputCloud(groundcloudPtr);
            extractor.setIndices(groundindicPtr);
            extractor.filter(*groundpointsPtr);


            //Extract ground plane outliers
            pcl::ExtractIndices <pcl::PointXYZ> outlier_extractor;
            outlier_extractor.setInputCloud(groundcloudPtr);
            outlier_extractor.setIndices(groundindicPtr);
            outlier_extractor.setNegative(true);
            outlier_extractor.filter(*objectpointsPtr);


            //Project the ground inliers
            pcl::ProjectInliers <pcl::PointXYZ> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(groundpointsPtr);
            proj.setModelCoefficients(groundcoeffPtr);
            proj.filter(*cloudprojPtr);


            //Creat Convex Hull of projected inliers
            pcl::ConvexHull <pcl::PointXYZ> chull;
            chull.setInputCloud(cloudprojPtr);
            chull.reconstruct(*groundhullPtr);//bis hier ok

            //Extract outliers above convex hull
            pcl::ExtractPolygonalPrismData <pcl::PointXYZ> hull_limiter;
            hull_limiter.setInputCloud(objectpointsPtr);
            hull_limiter.setInputPlanarHull(groundhullPtr);
            hull_limiter.setHeightLimits(0.05,0.3);    //All points having distances to the model outside this interval will be discarded.
            hull_limiter.segment(*objectindicPtr);


            pcl::ExtractIndices <pcl::PointXYZ> object_extractor;
            object_extractor.setInputCloud(objectpointsPtr);
            object_extractor.setIndices(objectindicPtr);

            pcl::PointCloud<pcl::PointXYZ>::Ptr objectpointsPtr_out(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));
            object_extractor.filter(*objectpointsPtr_out);

            pcl::PointCloud<pcl::PointXYZ>::Ptr objectpointsPtr_out_filtered(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));

            //Filter out the noise
            if ((objectpointsPtr_out->height * objectpointsPtr_out->width) > 0) {
                pcl::RadiusOutlierRemoval <pcl::PointXYZ> ror;
                ror.setInputCloud(objectpointsPtr_out);
                ror.setMinNeighborsInRadius(25);
                ror.setRadiusSearch(0.2);
                ror.filter(*objectpointsPtr_out_filtered);
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectprojPtr(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));

            //Project the obstacle in ground
            pcl::ProjectInliers <pcl::PointXYZ> proj2;
            proj2.setModelType(pcl::SACMODEL_PLANE);
            proj2.setInputCloud(objectpointsPtr_out_filtered);
            proj2.setModelCoefficients(groundcoeffPtr);
            proj2.filter(*ObjectprojPtr);


            //Transform the coordinate
            pcl::PointCloud<pcl::PointXYZ>::Ptr objectpointsPtr_out_filtered_transform(new pcl::PointCloud<pcl::PointXYZ>(transform(objectpointsPtr_out_filtered)));
            pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectProjPtr_transform(new pcl::PointCloud<pcl::PointXYZ>(transform(ObjectprojPtr)));

            //Change it to Pointcloud2
            //Convert pcl::CloudT back to PointCloud2 for last filter step
            pcl::PCLPointCloud2 *cloud_out = new pcl::PCLPointCloud2();
            pcl::PCLPointCloud2 *cloud_out_ground = new pcl::PCLPointCloud2();
            pcl::PCLPointCloud2 *cloud_out_ObjectProj = new pcl::PCLPointCloud2();

            pcl::toPCLPointCloud2(*objectpointsPtr_out_filtered_transform, *cloud_out);
            pcl::toPCLPointCloud2(*cloudprojPtr, *cloud_out_ground);
            pcl::toPCLPointCloud2(*ObjectProjPtr_transform, *cloud_out_ObjectProj);

            //Conversion out
            sensor_msgs::PointCloud2 output_Object, output_ground_plane, output_ObjectProj;
            pcl_conversions::moveFromPCL(*cloud_out, output_Object);
            pcl_conversions::moveFromPCL(*cloud_out_ground, output_ground_plane);
            pcl_conversions::moveFromPCL(*cloud_out_ObjectProj, output_ObjectProj);

            //Transform to occupancy grid map
            mapImport(ObjectProjPtr_transform, grid_pub);

            //Filter the noise better not to use it.
            std::vector<int8_t> grid_temp;
            //Openning
            grid_temp = inflate(grid_pub.data, 0);//erosion
            grid_temp = inflate(grid_temp, 100);//dilation

            //Closing
            grid_temp = inflate(grid_pub.data, 100);
            grid_temp = inflate(grid_temp, 0);

            grid_pub.data = grid_temp;

            //Publish
            pub_obstacles.publish(output_Object);
            pub_ground_plane.publish(output_ground_plane);
            pub_obstacles_proj.publish(output_ObjectProj);
            pub_occupancy_grid_map.publish(grid_pub);

            //Running time
            std::cout<<"time ="<<tt.toc()<<"ms"<<std::endl;
        }


    }

    void ObstacleDetection::setupIntrinsics(const sensor_msgs::CameraInfo::ConstPtr &camera_info) {

        try {

            Eigen::Affine3d kinect_ir_2_vehicle_transform{tf2::transformToEigen(
                    tf_buffer_kinect_ir_2_vehicle_.lookupTransform("vehicle_front_axle", "kinect2_ir_optical_frame",
                                                                   ros::Time(),
                                                                   ros::Duration(10e-3)))};

            Eigen::Affine3d vehicle_2_camera_top_transform{tf2::transformToEigen(
                    tf_buffer_vehicle_2_camera_top_.lookupTransform("camera_top", "vehicle_front_axle",
                                                                    ros::Time(),
                                                                    ros::Duration(10e-3)))};

            stargazer_2_world_transform_ = Eigen::Affine3d{tf2::transformToEigen(
                    tf_buffer_stargazer_2_world_.lookupTransform("world", "stargazer",
                                                                 ros::Time(),
                                                                 ros::Duration(10e-3)))};

            kinect_ir_2_camera_top_tranform_.matrix() =
                    vehicle_2_camera_top_transform.matrix() * kinect_ir_2_vehicle_transform.matrix();

            transform_set_ = true;
            subscriber_camera_info_.shutdown();

        }

    }
    const pcl::PointCloud <pcl::PointXYZ> & ObstacleDetection::transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input) {
        //transform from kinect2_ir_optical_frame to world_frame
        try {

            Eigen::Affine3d kinect_ir_2_world_transform{tf2::transformToEigen(
                    tf_buffer_camera_top_2_stargazer_.lookupTransform("stargazer", "camera_top",
                                                                      ros::Time(),
                                                                      ros::Duration(10e-1)))};

            kinect_ir_2_world_transform.matrix() =
                    stargazer_2_world_transform_.matrix() * kinect_ir_2_world_transform.matrix() *
                    kinect_ir_2_camera_top_tranform_.matrix();

            pcl::transformPointCloud(*input, *input, kinect_ir_2_world_transform);

            input->header.frame_id = "world";

        }

        return *input;
    }

    void ObstacleDetection::mapImport(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, nav_msgs::OccupancyGrid& output) {
        //Limit aus Andres Karte, maybe change the cell size
        output.data.assign((info.width * info.height), 0);
        int x_unten = 0;
        int x_oben = 340;
        int y_unten = 0;
        int y_oben = 131;
        int zaehler = 0;

        for (int i = 0; i < input->size(); ++i) {
            double x_pos_in = round(input->points[i].x / 0.05);
            double y_pos_in = round((input->points[i].y + 0.5) / 0.05);
            if (((x_pos_in >= x_unten) && (x_pos_in < x_oben)) && ((y_pos_in >= y_unten) && (y_pos_in < y_oben))) {
                double x_pos = x_pos_in;
                double y_pos = y_pos_in;
                output.data[(int) x_pos + (int) y_pos * 340] = 100;
            }

        }
        output.header.stamp = ros::Time();

    }

    std::vector<int8_t> ObstacleDetection::inflate(std::vector<int8_t> grid,int rad){

        //=width, adjust this with your grid size
        //for testen I need in this filed
        //rad = 0 for erosion and 100 for dilatation
        std::vector<int8_t> newgrid = grid ;
        for(int i = 0;i<131;i++){
            for(int j = 0;j<340;j++){
                if((int)grid[j+i*340] == rad){

                    for(int n = i-1; n <= i+1;n++){
                        for(int k = j-1; k <= j+1;k++){
                            if(n>=0&&n<131&&k>=0&&k<340){
                                newgrid[k+n*340] = rad;
                            }
                        }
                    }
                }
            }
        }
        return newgrid;
    }

} // namespace obstacle_detection_ros_tool
