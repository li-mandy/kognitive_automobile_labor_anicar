#include "robot_detector.hpp"
//using namespace pcl;
using namespace ros;

namespace additional_task_ros_tool {

RobotDetector::RobotDetector(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    /**
     * Initialization
     */
    interface_.fromParamServer();

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/RobotDetector.if file.
     * Don't forget to register your callbacks here!
     */
    this->setupSubscriber(nhPrivate);
    this->setupPublisher(nhPrivate);

    reconfigureServer_.setCallback(boost::bind(&RobotDetector::reconfigureRequest, this, _1, _2));
//    interface_.dummy_subscriber->registerCallback(&RobotDetector::callbackSubscriber, this);

    rosinterface_handler::showNodeInfo();

/*    grid_pub.data.assign(33274, 0);

    grid_pub.header.frame_id = "world";

    info.width = 254;
    info.height = 131;

    pose.position.x = 0;
    pose.position.y = -0.5;
    pose.position.z = 0;
    info.origin = pose;
    info.resolution = 0.05;
    grid_pub.info = info;
  */  int state = 4;
    int mes = 2;
    Eigen::MatrixXd A_(state, mes); // System dynamics matrix
    Eigen::MatrixXd H_(mes, state); // Output matrix
    Eigen::MatrixXd Q_(state, state); // Process noise covariance
    Eigen::MatrixXd R_(mes, mes); // Measurement noise covariance
    Eigen::MatrixXd P_(state, state);

    Eigen::MatrixXd M_(5,3);

    //x_ << x_x,x_y, 0, 0;

    P_ << interface_.ini_state_uncertainty, 0, 0, 0,
            0., interface_.ini_state_uncertainty, 0, 0,
            0, 0, interface_.ini_state_uncertainty, 0,
            0, 0, 0, interface_.ini_state_uncertainty;
    Q_ << .05, 0, .0, .0,
            0., .05, .0, .0,
            0., 0, .05, .0,
            .0, .0, .0, .05;
    H_ << 1., 0., 0., 0.,
            0., 1., 0., 0.;
    R_ << interface_.R_x * interface_.R_x, 0.,
            0., interface_.R_y * interface_.R_y;
    //time_initialized_ = false;
    old_time = new_time;
    new_time = ros::Time();
}
    void RobotDetector::setupSubscriber(ros::NodeHandle& nh) {

        subscriber_pcl_=std::make_unique<message_filters::Subscriber<PCL2>>(
                nh, interface_.input_pcl, interface_.queue_size);

        //kinect_image_synchronizer_ = std::make_unique<message_filters::Synchronizer<SynchronizedKinectImages>>(
          //      SynchronizedKinectImages(interface_.queue_size), *subscriber_image_, *subscriber_depth_);

        interface_.depth_subscriber->registerCallback(&RobotDetector::callback_depthimage, this);
        subscriber_camera_info_ = nh.subscribe("/kinect2/sd/camera_info", interface_.queue_size, &RobotDetector::setupIntrinsics, this, ros::TransportHints().tcpNoDelay());
    }

    void RobotDetector::setupPublisher(ros::NodeHandle& nh) {
        pub_ground_plane = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/Map_plane",interface_.queue_size);
        pub_obstacles = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/Map_obstacle",interface_.queue_size);
        pub_decision = nh.advertise<sensor_msgs::PointCloud2>("/robot_detector/pcl_decision",interface_.queue_size);

        chatter_pub= nh.advertise<std_msgs::Int8 >("/robot_detector/state_publisher",interface_.queue_size);
        //chatter_pub1= nh.advertise<std_msgs::Int8 >("/robot_detector/state_publisher1",interface_.queue_size);

    }

    void RobotDetector::setupIntrinsics(const sensor_msgs::CameraInfo::ConstPtr& camera_info) {
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
            //std::cout << "Kinect IR 2 Camera Top transform: " << kinect_ir_2_camera_top_tranform_.matrix() << std::endl;

       /*   Eigen::Affine3d kinect_ir_2_world_transform{tf2::transformToEigen(
                    tf_buffer_camera_top_2_stargazer_.lookupTransform("stargazer", "camera_top",
                                                                      ros::Time(),
                                                                      ros::Duration(10e-1)))};
                      kinect_ir_2_world_transform.matrix() =
                    stargazer_2_world_transform_.matrix() * kinect_ir_2_world_transform.matrix() *
                    kinect_ir_2_camera_top_tranform_.matrix();
*/
            transform_set_ = true;
    //       camera_info_globa = *camera_info;
            subscriber_camera_info_.shutdown();

        }
        catch (const tf2::TransformException& e) {
            /*
            ROS_WARN_STREAM(e.what() << std::endl
                                     << "Reference: "
                                     << camera_info->header.frame_id
                                     << ", sensor: "
                                     << interface_.frame_id_reference);
                                     */
        };
    }

void RobotDetector::callback_depthimage(const sensor_msgs::PointCloud2::ConstPtr&  input) {
    if(transform_set_)
    {
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*input, *cloud);


     /* Eigen::Affine3d kinect_ir_2_world_transform{tf2::transformToEigen(
                tf_buffer_camera_top_2_stargazer_.lookupTransform("stargazer","camera_top",
                                                                  input->header.stamp,
                                                                  ros::Duration(10e-1)))};
      */// we want just 2d points
        /*
        kinect_ir_2_world_transform.matrix()(2,0) = 0;
        kinect_ir_2_world_transform.matrix()(2,1) = 0;
        kinect_ir_2_world_transform.matrix()(2,2) = 0;
*/
        //kinect_ir_2_world_transform.matrix()(2,3) = 0;
//        kinect_ir_2_world_transform.matrix() = stargazer_2_world_transform_.matrix() * kinect_ir_2_world_transform.matrix() * kinect_ir_2_camera_top_tranform_.matrix();


//    std::cout << "Kinect IR 2 Camera Top transform: " << kinect_ir_2_world_transform.matrix() << std::endl;

        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PCLPointCloud2ConstPtr cloudPtr1(cloud);
        pcl::PCLPointCloud2ConstPtr cloudPtr2(cloud);
        pcl::PCLPointCloud2ConstPtr cloudPtr3(cloud);


        pcl::PassThrough <pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloudPtr);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.05, 0.3);      //rectification
        pass.filter(*cloud);

        pcl::PassThrough <pcl::PCLPointCloud2> pass1;
        pass1.setInputCloud(cloudPtr1);
        pass1.setFilterFieldName("z"); //entspricht hinten/vorne abschneiden
        pass1.setFilterLimits(0.5, 2.8);      //rectification
        pass1.filter(*cloud);

        pcl::PassThrough <pcl::PCLPointCloud2> pass2;
        pass2.setInputCloud(cloudPtr2);
        pass2.setFilterFieldName("x"); //entspricht links/rechts abschneiden
        pass2.setFilterLimits(-3, 2);      //rectification
        pass2.filter(*cloud);



        pcl::VoxelGrid <pcl::PCLPointCloud2> vox;
        vox.setInputCloud(cloudPtr3);
        vox.setLeafSize(leaf_size, leaf_size,
                        leaf_size); //leaf size in [meter] je kleiner f Wert, desto mehr Punkte bleiben über
        vox.filter(*cloud);


        //pcl::PCLPointCloud2ConstPtr tempPtr(cloud);
        pcl::PointCloud <pcl::PointXYZ> groundcloud;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(*cloud, groundcloud);
        //pcl::fromPCLPointCloud2(*cloud, *pcl_cloud);

        for (int i = 0; i < groundcloud.width; i = i + 50) {
            //std::cout<<"index["<<i<<"]"<<" x ="<<groundcloud.points[i].x<<" y = "<<groundcloud.points[i].y<<", z = "<<groundcloud.points[i].z<<std::endl;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr groundcloudPtrtemp(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr groundcloudPtrtemp_const(groundcloudPtrtemp);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));

        //Filter out the noise
        pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor2;
        sor2.setInputCloud(groundcloudPtrtemp_const);
        sor2.setMeanK(100);
        sor2.setStddevMulThresh(1.0);
        sor2.filter(*cloud_filtered); //problem here
      /*  for (int i = 0; i < cloud_filtered->width; i = i + 200) {
            std::cout<<"Ohne    index["<<i<<"]"<<" x ="<<cloud_filtered->points[i].x<<" y = "<<cloud_filtered->points[i].y<<", z = "<<cloud_filtered->points[i].z<<std::endl;
        }
       */// pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, kinect_ir_2_world_transform);

/*        for (int i = 0; i < cloud_filtered->width; i = i + 200) {
            std::cout<<"Mit     index["<<i<<"]"<<" x ="<<cloud_filtered->points[i].x<<" y = "<<cloud_filtered->points[i].y<<", z = "<<cloud_filtered->points[i].z<<std::endl;
        }
*/

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
        seg.setDistanceThreshold(0.1);
        seg.setMaxIterations(200);  //bis hier 19 07 07
        seg.setInputCloud(groundcloudPtr);
        seg.segment(*groundindicPtr, *groundcoeffPtr);

        //Extract ground plane inliers
        pcl::ExtractIndices <pcl::PointXYZ> extractor;
        extractor.setInputCloud(groundcloudPtr);
        extractor.setIndices(groundindicPtr);
        //cout<<"this is for debug"<<endl;
        extractor.filter(*groundpointsPtr);


        //Extract ground plane outliers
        pcl::ExtractIndices <pcl::PointXYZ> outlier_extractor;
        outlier_extractor.setInputCloud(groundcloudPtr);
        outlier_extractor.setIndices(groundindicPtr);
        outlier_extractor.setNegative(true);
        outlier_extractor.filter(*objectpointsPtr);


        //project the ground inliers
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
        hull_limiter.setHeightLimits(0.075, 5);    //All points having distances to the model outside this interval will be discarded.
        hull_limiter.segment(*objectindicPtr); //bie hier ok


        pcl::ExtractIndices <pcl::PointXYZ> object_extractor;
        object_extractor.setInputCloud(objectpointsPtr);
        object_extractor.setIndices(objectindicPtr);

        pcl::PointCloud<pcl::PointXYZ>::Ptr objectpointsPtr_out(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));
        object_extractor.filter(*objectpointsPtr_out);//problem hier

        pcl::PointCloud<pcl::PointXYZ>::Ptr objectpointsPtr_out_filtered(
                new pcl::PointCloud<pcl::PointXYZ>(groundcloud));

        //filter out the noise
        if ((objectpointsPtr_out->height * objectpointsPtr_out->width) > 0) {
            pcl::RadiusOutlierRemoval <pcl::PointXYZ> ror;
            ror.setInputCloud(objectpointsPtr_out);
            ror.setMinNeighborsInRadius(20);
            ror.setRadiusSearch(0.1);
            ror.filter(*objectpointsPtr_out_filtered);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectprojPtr(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));
        //project the obstacle in ground
        pcl::ProjectInliers <pcl::PointXYZ> proj2;
        proj2.setModelType(pcl::SACMODEL_PLANE);
        proj2.setInputCloud(objectpointsPtr_out_filtered);
        proj2.setModelCoefficients(groundcoeffPtr);
        proj2.filter(*ObjectprojPtr);

        //convert pcl::CloudT back to PointCloud2 for last filter step
        pcl::PCLPointCloud2 *cloud_out = new pcl::PCLPointCloud2();
        pcl::PCLPointCloud2 *cloud_out_ground = new pcl::PCLPointCloud2();
        pcl::PCLPointCloud2 *cloud_out_ObjectProj = new pcl::PCLPointCloud2();

        pcl::PCLPointCloud2ConstPtr cloudOutPtr(cloud_out);

        //transform the coordinate
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectpointsPtr_out_filtered_transform(
                new pcl::PointCloud<pcl::PointXYZ>(transform(objectpointsPtr_out_filtered)));
        pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectProjPtr_transform(
                new pcl::PointCloud<pcl::PointXYZ>(transform(ObjectprojPtr)));

/*if(ObjectProjPtr_transform->width>1) {
    for (int i = 0; i <= ObjectProjPtr_transform->width; i = i + 1) {
        std::cout << "x=" << ObjectProjPtr_transform->points[i].x << std::endl;
        std::cout << "y=" << ObjectProjPtr_transform->points[i].y << std::endl;
        std::cout << "z=" << ObjectProjPtr_transform->points[i].z << std::endl;
    }
}
 */

        //change it to Pointcloud2
        pcl::toPCLPointCloud2(*objectpointsPtr_out_filtered_transform, *cloud_out);
        pcl::toPCLPointCloud2(*cloudprojPtr, *cloud_out_ground);
        pcl::toPCLPointCloud2(*ObjectProjPtr_transform, *cloud_out_ObjectProj);
/*
        //for rectification
        pcl::toPCLPointCloud2(*objectpointsPtr_out_filtered,*cloud_out);
        pcl::toPCLPointCloud2(*cloudprojPtr,*cloud_out_ground);
        pcl::toPCLPointCloud2(*ObjectprojPtr,*cloud_out_ObjectProj);
*/
        //conversion out
        sensor_msgs::PointCloud2 output_Object, output_ground_plane, output_ObjectProj;
        pcl_conversions::moveFromPCL(*cloud_out, output_Object);
        pcl_conversions::moveFromPCL(*cloud_out_ground, output_ground_plane);
        pcl_conversions::moveFromPCL(*cloud_out_ObjectProj, output_ObjectProj);


        interface_.obstacle_publisher.publish(output_Object);
        interface_.ground_publisher.publish(output_ground_plane);
        interface_.pcl_publisher.publish(output_ObjectProj);

        std::cout<<"width= "<<ObjectProjPtr_transform->width<<std::endl;
        if (ObjectProjPtr_transform->width >= 1) {
            makeDecision(ObjectProjPtr_transform, ros::Time());
           // std_msgs::Int8 msg;
            //msg.data = 17;
            //ROS_INFO("%d", msg.data);
            //chatter_pub.publish(msg);

        }

    }


}

const pcl::PointCloud<pcl::PointXYZ> & RobotDetector::transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input) {

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
            //std::cout<<"frame_ID: "<<input->header.frame_id<<std::endl;
            //std::cout << "Kinect IR 2 Camera Top transform: " << kinect_ir_2_world_transform.matrix() << std::endl;
        }
        catch (const tf2::TransformException &e) {
            ROS_WARN_STREAM(e.what() << std::endl
                                     << "Reference: "
                                     << camera_info_globa.header.frame_id
                                     << ", sensor: ");
        };
    return *input;
    }

void RobotDetector::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}
void RobotDetector::mapImport(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input){
    //Limit aus Andres Karte
    /*
     * Oben Tisch x=10.55 y=6.87
     * Vorne Auf der Linie x=8.2151 y=6.1847
     * Zebbrastreifen Oben x=9.655 y= 6.183
     * zebra unten x=9.169 y=5.081
     *
     */
    int x_unten=0;
    int x_oben=254;
    int y_unten=0;
    int y_oben=131;
    int zaehler=0;

    for (int i = 0; i < input->height; ++i) {
        double x_pos_in = round(input->points[i].x/0.05);
        double y_pos_in = round((input->points[i].y+0.5)/0.05);
        if (((x_pos_in >= x_unten) && (x_pos_in < x_oben)) && ((y_pos_in >= y_unten) && (y_pos_in < y_oben))){
            double x_pos =x_pos_in;
            double y_pos =y_pos_in;
            grid_pub.data[(int)x_pos+(int)y_pos*254] = 100;
        }
    }
    grid_pub.header.stamp = ros::Time();
}

void RobotDetector::makeDecision(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, ros::Time stamp) {
    //Limit aus Andrés Karte
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*input, *cloud_in);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudPTR1(cloud_in);
*/
    //pcl::PointCloud<pcl::PointXYZ>::Ptr groundcloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud <pcl::PointXYZ> groundcloud;
    pcl::copyPointCloud(*input, groundcloud);


    pcl::PointCloud<pcl::PointXYZ>::Ptr groundcloudPtrtemp(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));
    //pcl::PointCloud<pcl::PointXYZ>::ConstPtr groundcloudPtrtemp_const(groundcloudPtrtemp);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr groundcloudPtrtemp_const(groundcloudPtrtemp);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr groundcloudPtrtemp_const1(groundcloudPtrtemp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond1(new pcl::ConditionAnd<pcl::PointXYZ>());

    for (int i = 0; i < groundcloudPtrtemp->width; ++i) {
        std::cout << i << "<= i,    ground  x= " << groundcloudPtrtemp->points[i].x << " y= "
                  << groundcloudPtrtemp->points[i].y << "z=" << groundcloudPtrtemp->points[i].z << std::endl;
    }
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, interface_.x_min)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, interface_.x_max)));
    range_cond1->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, interface_.y_min)));
    range_cond1->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, interface_.y_max)));
    pcl::ConditionalRemoval <pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(groundcloudPtrtemp);
    //condrem.setKeepOrganized (true);
    condrem.filter(*cloud_filtered);
    std::cout << "filtered_width= " << cloud_filtered->width << std::endl;
  /*  for (int i = 0; i < cloud_filtered->width; ++i) {
        std::cout << i << "<= i,    filter  x= " << cloud_filtered->points[i].x << " y= " << cloud_filtered->points[i].y
                  << "z=" << cloud_filtered->points[i].z << std::endl;
    }
*/

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr groundcloudPtr(cloud_filtered);
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundpointsPtr(new pcl::PointCloud<pcl::PointXYZ>(groundcloud));

    /*
 * Oben Tisch x=10.55 y=6.87
 * Vorne Auf der Linie x=8.2151 y=6.1847
 * Zebbrastreifen Oben x=9.655 y= 6.183
 * zebra unten x=9.169 y=5.081
 *
 */
    pcl::ConditionalRemoval <pcl::PointXYZ> condrem1;
    condrem1.setCondition(range_cond1);
    condrem1.setInputCloud(groundcloudPtr);
    condrem1.filter(*groundpointsPtr);

    std::cout << "filtered1_width= " << groundpointsPtr->width << std::endl;
    for (int i = 0; i < groundpointsPtr->width; ++i) {
        std::cout << "x= " << groundpointsPtr->points[i].x << " y= " << groundpointsPtr->points[i].y << "z="
                  << groundpointsPtr->points[i].z << std::endl;
    }
    if (groundpointsPtr->width > 0) {
        sensor_msgs::PointCloud2 pcl_decision;
        pcl::PCLPointCloud2 *cloud_out = new pcl::PCLPointCloud2();
        pcl::toPCLPointCloud2(*groundpointsPtr, *cloud_out);
        pcl_conversions::moveFromPCL(*cloud_out, pcl_decision);

        pub_decision.publish(pcl_decision);
        double x_mean = 0;//(9.655 + 9.169) / 2;
        double y_mean = 0;//(5.081 + 6.183) / 2;

        for (int i = 0; i < groundpointsPtr->width; ++i) {
            x_mean = x_mean + groundpointsPtr->points[i].x;
            y_mean = y_mean + groundpointsPtr->points[i].y;

        }
        x_mean = x_mean / groundpointsPtr->width;
        y_mean = y_mean / groundpointsPtr->width;
        std::cout << "x_mean=       " << x_mean << " , y_mean=      " << y_mean << std::endl;

        for (int j = 4; j > 0; --j) {
            M_.row(j)=M_.row(j-1);
        }
        M_.row(0)<<x_mean,y_mean, ros::Time::now();
        dec_state = 0; //rand() % 2;
        measurement << x_mean, y_mean;
        //MISSING_Initialise
       // if (!time_initialized_) { init(x_, stamp); }
        /*
     * Oben Tisch x=10.55 y=6.87
     * Vorne Auf der Linie x=8.2151 y=6.1847
     * Zebbrastreifen Oben x=9.655 y= 6.183
     * zebra unten x=9.169 y=5.081
     *
     */
           //update(measurement, x_);
        std::cout << "x_mean_kalman = " << measurement[0] << " , y_mean_kalman=     " << measurement[1] << std::endl;
           if (measurement[1] <= 6.1 && measurement[1] >= 5.08) //
           {
               dec_state = 1; //Stopp
           }
           else if (measurement[0] >= 9.17 && measurement[0] <= 9.65)
           {
               dec_state = 1;
           }
           //else if (x_[3]>x_[2]&&x_[3]>=0.05)
           // {
           // dec_state=1;
           // }
           else dec_state = 0;
        msg2.data = dec_state;
       // msg2.header.stamp = ros::Time::now();
        chatter_pub.publish(msg2);
    }


    }



void RobotDetector::update(Eigen::Vector2d& measurement, Eigen::Vector4d& x_state){
    if(!time_initialized_){
        x_ << interface_.ini_x, interface_.ini_y, interface_.ini_v_x, interface_.ini_v_y;
        dt_ = 1. / interface_.timer_rate;
        A_ << 1., 0., dt_ ,0.,
                0., 1.,0., dt_ ,
                0., 0., 1., 0.,
                0., 0., 0., 1.;
        x_=A_*x_;

        P_ = A_ * P_ * A_.transpose() + Q_;


        K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
        x_ = x_ + K_ * (measurement - H_ * x_);
        P_ = (Eigen::Matrix4d::Identity(4, 4) - K_ * H_) * P_;

        ///Initialise Kalman Filter;
    }
    if(time_initialized_) {
        ros::Duration dt_dur_;
        dt_dur_ = new_time - old_time;

        // to double
        dt_ = dt_dur_.toSec();


        x_ = A_ * x_state;
        P_ = A_ * P_ * A_.transpose() + Q_;
        //if (!((x_state[0] == measurement[0]) && (x_state[1] == measurement[1])))
        {
            K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
            x_ = x_ + K_ * (measurement - H_ * x_);
            P_ = (Eigen::Matrix4d::Identity(4, 4) - K_ * H_) * P_;
            //x_hat = x_hat_new;
        }
        //old_time=new_time;

    }
    time_initialized_=true;
    }

   void RobotDetector::init(const Eigen::VectorXd& x0, ros::Time new_time )
    {

        //A_predict
        //Q_predict


        /*
* Oben Tisch x=10.55 y=6.87
* Vorne Auf der Linie x=8.2151 y=6.1847
* Zebbrastreifen Oben x=9.655 y= 6.183
* zebra unten x=9.169 y=5.081
*
*/

}

    /*
     1. Objekte in vordefiniertem Raum rausfiltern
     2. Schritt Mittelpunkt der Messung bestimmen
     3. Kalman Filter
     4. Entscheidungsfindung
     4.1 Roboter auf der Straße => Stopp an André
     4.2 Roboter in Straßennähe
     4.2.1 geringes v_y=> will passieren
     4.2.1 großes v_y=> patrullieren
     4.3. Roboter nicht in der Nähe des Zebrastreifens => weiterfahren


     */



} // namespace additional_task_ros_tool
