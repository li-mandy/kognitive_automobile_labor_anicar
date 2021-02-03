#include "sign_detection.hpp"


namespace sign_detection_ros_tool {

    using namespace std;

    vector<string> classes;//Container for storing names
    float confThreshold = 0.9;//Confidence threshold
    float nmsThreshold = 0.4;//Non-maximum suppression threshold
    int inpWidth = 218;//Input image width
    int inpHeight = 218;//Input image height
    int count = 0;

    void SignDetection::postprocess(cv::Mat& frame,const vector<cv::Mat>& outs,cv::Mat& depthimage,int camerafactor) {

        vector<int> classIds;//Index of storage identification class
        vector<float> confidences;//Store confidence
        vector<cv::Rect> boxes;//Save border


        for (size_t i = 0; i < outs.size(); i++) {
            //Scan all bounding boxes from the network output
            //Keep high confidence check box
            //Target data data: x, y, w, h are percentages, x, y are the coordinates of the target center point
            float frame_z;
            float *data = (float *) outs[i].data;
            frame_z = depthimage.at<short>((int) (data[0] * frame.cols),(int) (data[1] * frame.rows);)/camerafactor;
            for (int j = 0; j < outs[i].rows; j++, data += outs[i].cols) {
                cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                cv::Point classIdPoint;
                double confidence;
                //Get the maximum score and index
                cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if (confidence > confThreshold && frame_z<2.0f) {
                    int centerX = (int) (data[0] * frame.cols);
                    int centerY = (int) (data[1] * frame.rows);
                    int width = (int) (data[2] * frame.cols);
                    int height = (int) (data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float) confidence);
                    boxes.push_back(cv::Rect(left, top, width, height));
                }

            }

        }
        //Low confidence
        vector<int> indices;//Save index without overlapping borders
        //This function is used to suppress overlapping borders
        cv::dnn::NMSBoxes(boxes,confidences,confThreshold,nmsThreshold,indices);
        for(size_t i=0;i<indices.size();i++){
            int idx = indices[i];
            cv::Rect box = boxes[idx];
            drawPred(classIds[idx],confidences[idx],box.x,box.y,
                     box.x+box.width,box.y+box.height,frame);
        }
    }
    //Draw prediction bounding box and publish messages of detected results
    void SignDetection::drawPred(int classId,float conf,int left,int top,int right,int bottom,cv::Mat& frame) {
        cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);

        string label = cv::format("%.2f", conf);
        if (!classes.empty()) {
            CV_Assert(classId < (int) classes.size());
            label = classes[classId] + ":" + label;//Category label and confidence on the border

            if(label.size()==9){
               std::cout<<"the sign is stop"<<std::endl;
               //std::cout<<label<<std::endl;
               pub_stop.publish(4);
            }
            else if (label.size()==16){
               std::cout<<"the sign is go_straight"<<std::endl;
               //std::cout<<label<<std::endl;
               pub_go_direction.publish(0);
            }
            else if (label.size()==14){
               std::cout<<"the sign is turn_left"<<std::endl;
               //std::cout<<label<<std::endl;
               pub_go_direction.publish(2);
            }
            else if (label.size()==15){
               std::cout<<"the sign is turn_right"<<std::endl;
               //std::cout<<label<<std::endl;
               pub_go_direction.publish(1);
            }
            else{
               std::cout<<"no signs detected"<<std::endl;
            }
          }



        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = max(top, labelSize.height);
        cv::rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)),
                      cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255),
                      cv::FILLED);
        cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
    }


    //Output names of detected traffic signs
    vector<cv::String> SignDetection:: getOutputNames(const cv::dnn::Net& net){
        static vector<cv::String> names;
        if(names.empty()) {
            vector<int> outLayers = net.getUnconnectedOutLayers();
            vector <cv::String> layersNames = net.getLayerNames();
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); i++) {
                names[i] = layersNames[outLayers[i] - 1];
            }
        }
        return names;

    }

    SignDetection::SignDetection(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
              : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

       /**
        * Initialization
        */
       interface_.fromParamServer();
       image_subscriber = nhPublic.subscribe("/kinect2/qhd/image_color_rect", interface_.queue_size, &SignDetection::callbackImage, this);
       camera_factor_subscriber = nhPublic.subscribe("/kinect2/qhd/camera_info", interface_.queue_size, &SignDetection::callbackImage, this);
       depth_image_subscriber = nhPublic.subscribe("/kinect2/qhd/image_depth_rect", interface_.queue_size, &SignDetection::callbackImage, this);
       subscriber_camera_time_ = nhPublic.subscribe("/kinect2/qhd/camera_info", interface_.queue_size, &SignDetection::setupIntrinsics, this, ros::TransportHints().tcpNoDelay());
       pub_signs =nhPublic.advertise<sensor_msgs::Image>("/sign_detection/detected_signs",interface_.queue_size);
       pub_stop =nhPublic.advertise<std_msgs::Int8>("/stop/detected_signs",interface_.queue_size);
       pub_go_direction =nhPublic.advertise<std_msgs::Int8>("/go_direction/detected_signs",interface_.queue_size);


       /**
        * Set up callbacks for subscribers and reconfigure.
        *
        * New subscribers can be created with "add_subscriber" in "cfg/SignDetection.if file.
        * Don't forget to register your callbacks here!
        */
       reconfigureServer_.setCallback(boost::bind(&SignDetection::reconfigureRequest, this, _1, _2));
       interface_.dummy_subscriber->registerCallback(&SignDetection::callbackSubscriber, this);

       rosinterface_handler::showNodeInfo();
    }


    void SignDetection::callbackSubscriber(const Msg::ConstPtr& msg) {

        Msg::Ptr newMsg = boost::make_shared<Msg>(*msg);
        interface_.dummy_publisher.publish(newMsg);
    }

    void SignDetection::setupIntrinsics(const sensor_msgs::CameraInfo::ConstPtr& camera_info) {
        camera_time_globa = *camera_info;
    }

    void SignDetection::callbackImage(const sensor_msgs::Image::ConstPtr& msg, const sensor_msgs::ImageConstPtr& msg2, const sensor_msgs::CameraInfo::ConstPtr& msg3){

        string classesFile = "/home/kal5/docu/voc.names";
        ifstream ifs(classesFile.c_str());
        string line;
        while(getline(ifs,line))classes.push_back(line);

        // Load weights and network.
        cv::String modelConfiguration = "/home/kal5/docu/yolov3.cfg";
        cv::String modelWeights = "/home/kal5/ducu/yolov3.weights";
        cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration,modelWeights);
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableBackend(cv::dnn::DNN_TARGET_CPU);


        //Open video file or graphics file or camera data stream
        string str, outputFile;
        cv::Mat frame,blob,depthimage;
        int camerafactor = msg3.camera_factor;

        //Convert ros depth image to opencv depth image
        cv_bridge::CvImagePtr cv_ptr2;
        cv_ptr2=cv_bridge::yoCvCopy(msg2,sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptr2->image.copyTo(depthimage);

        //Convert ros color image to opencv color image
        cv_bridge::CvImagePtr cv_ptr1;
        cv_ptr1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        frame=cv_ptr1->image;


        cv::dnn::blobFromImage(frame, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0,0,0), true, false);
        //Set input network
        net.setInput(blob);
        //Set the output layer
        vector <cv::Mat> outs;//Save recognition results
        net.forward(outs, getOutputNames(net));

        //Remove low confidence bounding box
        postprocess(frame, outs, depthimage,camerafactor);

        //Display s delay information and draw
        vector<double> layersTimes;
        double freq = cv::getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        string label = cv::format("Infercence time for a frame:%.2f ms", t);
        cv::putText(frame, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));


        //Draw recognition box
        cv::Mat detecteFrame;
        //Publish detected signs
        frame.convertTo(detecteFrame, CV_8U);
        sensor_msgs::ImagePtr ros_image=cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
        pub_signs.publish(ros_image);
    }

    /**
     * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
     **/
    void SignDetection::reconfigureRequest(const Interface::Config& config, uint32_t level) {
            interface_.fromConfig(config);
        }

} // namespace sign_detection_ros_tool
