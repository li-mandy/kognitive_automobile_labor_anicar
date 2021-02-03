#include "path_planner.hpp"

namespace path_planner_ros_tool {
    
    const int PathPlanner::GOAL_UP = 1;
    const int PathPlanner::GOAL_DOWN = 2;
    const int PathPlanner::GOAL_LEFT = 3;
    const int PathPlanner::GOAL_RIGHT = 4;
    const int PathPlanner::GOAL_BEFORE_UE = 5;
    const int PathPlanner::GOAL_ZEBRA = 6;
    const int PathPlanner::GOAL_UE = 0;

    const int PathPlanner::GRID_WIDTH = 254;
    const int PathPlanner::GRID_HEIGHT = 131;

    const int PathPlanner::PLANNEDTOLEAVE = 1;
    const int PathPlanner::NOTPLANNED = 0;
    const int PathPlanner::PLANNEDTOGOAL = 2;

    const bool PathPlanner::WITH_INTERPOLATION = true;

PathPlanner::PathPlanner(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, tfListener_{tfBuffer_}  {


    goal_ = 99;
    state_= 99;
    if(!importGoals(goalCoordinates_, "goals.xml")){
        return;
    }
    if(!importGoals(leaveIntersectionCoordinates_, "leave_intersection.xml")){
        return;
    }
    
    costMaps_ = new std::vector<int8_t>[7];
    costMaps_[GOAL_UP] = importXML("costmap right out.osm");
    costMaps_[GOAL_DOWN] = importXML("costmap left out.osm");
    costMaps_[GOAL_BEFORE_UE] =costMaps_[GOAL_UP];
    costMaps_[GOAL_UE].assign(GRID_HEIGHT*GRID_WIDTH,0);
    costMaps_[GOAL_RIGHT].assign(GRID_HEIGHT*GRID_WIDTH,0);//todo
    costMaps_[GOAL_LEFT].assign(GRID_HEIGHT*GRID_WIDTH,0);//todo
    costMaps_[GOAL_ZEBRA]=costMaps_[GOAL_UP];

    blockages_ = new std::vector<int8_t>[7];
    blockages_[GOAL_UP] = importXML("block up.osm");
    blockages_[GOAL_DOWN] = importXML("block down.osm");
    blockages_[GOAL_BEFORE_UE].assign(GRID_HEIGHT*GRID_WIDTH,0);
    blockages_[GOAL_UE].assign(GRID_HEIGHT*GRID_WIDTH,0);
    blockages_[GOAL_RIGHT] = importXML("block right.osm");
    blockages_[GOAL_LEFT] = importXML("block left.osm");
    blockages_[GOAL_ZEBRA] = importXML("block zebra.osm");

    obstacles_ = new std::vector<int8_t>[5];
    seq = 0;
    time = ros::Time::now();

    interface_.fromParamServer();
    sub_map_ = nhPrivate.subscribe("/map_provider/map",10,&PathPlanner::callbackMap, this);
    sub_goal_ = nhPrivate.subscribe("/state_machine/goal",10,&PathPlanner::callbackGoal,this);
    sub_obstacle_ = nhPrivate.subscribe("/obstacle_detection/Occupancy_grid_map",100,&PathPlanner::callbackObstacle,this);
    pub_path_ = nhPrivate.advertise<nav_msgs::Path>("path",10);
    pub_map_ = nhPrivate.advertise<nav_msgs::OccupancyGrid>("map",10);
    pub_obstmap_ = nhPrivate.advertise<nav_msgs::OccupancyGrid>("obstacle",10);


    reconfigureServer_.setCallback(boost::bind(&PathPlanner::reconfigureRequest, this, _1, _2));

    rosinterface_handler::showNodeInfo();


    while(ros::ok()){
        if(origMap_.header.seq > 0 && state_!=99){
            switch(state_){
                case NOTPLANNED:
                    if(goal_==GOAL_UP){
                        nav_msgs::OccupancyGrid map = origMap_;
                        map.data=mergeGrids(map.data,blockages_[goal_]);
                        map.data=mergeGrids(map.data,costMaps_[goal_]);
                        if(planTo(map ,goalCoordinates_[goal_])){
                            state_ = PLANNEDTOGOAL;
                            ROS_INFO_STREAM("PLANNED TO GOAL");
                        }
                    }else{
                        if(planTo(origMap_ ,leaveIntersectionCoordinates_[goal_])){
                            state_ = PLANNEDTOLEAVE;
                            ROS_INFO_STREAM("PLANNED TO LEAVE");
                        }
                    }


                    break;
                case PLANNEDTOLEAVE:
                    if(inRange(leaveIntersectionCoordinates_[goal_],0.7)){
                        nav_msgs::OccupancyGrid map = origMap_;

                        map.data = mergeGrids(map.data,blockages_[goal_]);
                        map.data = mergeGrids(map.data,costMaps_[goal_]);

                        planTo(map,goalCoordinates_[goal_]);
                        state_ = PLANNEDTOGOAL;
                        ROS_INFO_STREAM("PLANNED TO GOAL");
                    }
                    break;
                case PLANNEDTOGOAL:

                    //wait till new goal
                    break;

            }

        }
        ros::spinOnce();

    }
}

bool PathPlanner::planTo(nav_msgs::OccupancyGrid map, geometry_msgs::Point goal){
    bool result = false;
    geometry_msgs::Point position;
    try {
        const geometry_msgs::TransformStamped tf_ros = tfBuffer_.lookupTransform("world", "vehicle_front_axle", ros::Time(0));
        position.x = tf_ros.transform.translation.x;
        position.y = tf_ros.transform.translation.y;
    } catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM(e.what());
        ROS_INFO_STREAM("no frame found");
        return false;
    }


    pathPlanner_ = new Dsl(map, position, goal);

    if(pathPlanner_->replan()){

        nav_msgs::Path path = pathPlanner_->path();


        if(WITH_INTERPOLATION){
            nav_msgs::Path smoothedPath;
            interpolatePath(pathPlanner_->path(),smoothedPath);
            path = smoothedPath;
        }

        pub_map_.publish(map);

        pub_path_.publish(path);


        ROS_INFO_STREAM("path published");
        result = true;
    } else {
        ROS_INFO_STREAM("no path found");
    }
    return result;

}

void PathPlanner::callbackGoal(const std_msgs::Int8 newGoal){
    goal_ = (int)newGoal.data;
    ROS_INFO_STREAM("received new goal");
    state_ = NOTPLANNED;


}

void PathPlanner::callbackObstacle(const nav_msgs::OccupancyGrid map) {

    ros::Duration diff =  ros::Time::now() - time;
    if(state_ == PLANNEDTOGOAL && diff.toNSec()>500000000){
        obstacles_[seq] = map.data;
        seq = (seq + 1) % 5;

        std::vector<int8_t> myobstacles = obstacles_[0];
        for(int i = 1;i<5;i++){
            myobstacles = mergeGrids(myobstacles,obstacles_[i]);

        }
        myobstacles = inflate(myobstacles,4);

        map.data = myobstacles;
        pub_obstmap_.publish(map);


        uint32_t cols = map.info.width;
        uint32_t rows = map.info.height;

        for(int i = 0; i<rows;i++){
            for(int j = 0;j<cols;j++){
                geometry_msgs::Point point;
                point.x = (j * map.info.resolution) + map.info.origin.position.x;
                point.y = (i * map.info.resolution) + map.info.origin.position.y;
                if(myobstacles[j+i*GRID_WIDTH]>0){
                    pathPlanner_->update(point,Cell::COST_UNWALKABLE);
                }else{
                    pathPlanner_->reset(point);
                }

            }

        }


        if(pathPlanner_->replan()){
            nav_msgs::Path path = pathPlanner_->path();


            if(WITH_INTERPOLATION){
                nav_msgs::Path smoothedPath;
                interpolatePath(pathPlanner_->path(),smoothedPath);
                path = smoothedPath;
            }

            pub_path_.publish(path);
            ROS_INFO_STREAM("New Path published");
        } else{
            ROS_INFO_STREAM("no Path found");
        }


    }

}

void PathPlanner::callbackMap(const nav_msgs::OccupancyGrid map) {
    origMap_ = map;

}




/**
 * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
 */
void PathPlanner::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}




std::vector<int8_t> PathPlanner::mergeGrids(std::vector<int8_t> grid1, std::vector<int8_t> grid2){
    std::vector<int8_t> result;//[33275];
    result.assign(GRID_HEIGHT*GRID_WIDTH,-1);
    for(int i = 0;i<GRID_HEIGHT;i++){
        for(int j = 0;j<GRID_WIDTH;j++){
            result[j+i*GRID_WIDTH] = std::max((int)grid1[j+i*GRID_WIDTH],(int)grid2[j+i*GRID_WIDTH]);
        }
    }
    return result;
}

std::vector<int8_t> PathPlanner::inflate(std::vector<int8_t> grid,int rad){
    std::vector<int8_t> newgrid = grid ;
    for(int i = 0;i<GRID_HEIGHT;i++){
        for(int j = 0;j<GRID_WIDTH;j++){
            if((int)grid[j+i*GRID_WIDTH] == 100){

                for(int n = i-rad; n <= i+rad;n++){
                    for(int k = j-rad; k <= j+rad;k++){
                        if(n>=0&&n<GRID_HEIGHT&&k>=0&&k<GRID_WIDTH){
                            int dist_i = i-n;
                            int dist_j = j-k;
                            int dist = std::sqrt(dist_i*dist_i + dist_j*dist_j);
                            if(dist<=rad){
                                newgrid[k+n*GRID_WIDTH] = std::max((int)grid[k+n*GRID_WIDTH],80);
                            }
                        }
                    }
                }
            }
        }
    }
    return newgrid;
}

std::vector<int8_t> PathPlanner::importXML(std::string filename){
    //todo use meta data
    std::vector<int8_t> grid;//[33275];
    grid.assign(GRID_HEIGHT*GRID_WIDTH,0);
    TiXmlDocument costs( ros::package::getPath("path_planner_ros_tool") + "/res/" +filename);
    if(costs.LoadFile()){
        ROS_INFO_STREAM("Found xml");
        TiXmlElement *root, *node;
        root = costs.FirstChildElement("osm");
        if(root){
            node = root->FirstChildElement("node");
            while(node){
                double x = std::stod(node->Attribute("lon"));
                double y = std::stod(node->Attribute("lat"));
                double x_pos = round(x/0.05);
                double y_pos = round((y+0.5)/0.05);

                TiXmlElement *tag = node->FirstChildElement("tag");
                std::string attribute = tag->Attribute("k");
                if(!(attribute.compare("colour")==0)){
                    tag = tag->NextSiblingElement("tag");

                }
                std::string color = "";
                if(tag) {
                    color = tag->Attribute("v");
                }

                if(tag && (color.compare("white")==0 || color.compare("red")==0)){
                    //ROS_INFO_STREAM("occupied");
                    grid[(int)x_pos+(int)y_pos*GRID_WIDTH] = 100;

                }else if(tag && color.compare("blue")==0){
                    //ROS_INFO_STREAM("not occupied");
                    grid[(int)x_pos+(int)y_pos*GRID_WIDTH] = 50;
                    //ROS_INFO_STREAM("something blue was found");

                }else{
                    grid[(int)x_pos+(int)y_pos*GRID_WIDTH] = 0;
                }
                node = node->NextSiblingElement("node");
            }
        }


    }else{
        ROS_INFO_STREAM("Couldn't find xml " + filename);
    }
    return grid;
}
bool PathPlanner::importGoals(geometry_msgs::Point*& array, std::string filename){
    array = new geometry_msgs::Point[7];

    TiXmlDocument costs( ros::package::getPath("path_planner_ros_tool") + "/res/" + filename);
    if(costs.LoadFile()){
        ROS_INFO_STREAM("Found xml");
        TiXmlElement *root, *node;
        root = costs.FirstChildElement("GOALS");
        if(root){
            node = root->FirstChildElement("goal");
            while(node){
                TiXmlElement *name = node->FirstChildElement( "name" );
                int goalnumber = 99;
                if ( NULL != name )
                {

                    if(std::strcmp(name->GetText(),"GOAL_UP")==0){
                        goalnumber = GOAL_UP;
                    }else if(std::strcmp(name->GetText(),"GOAL_DOWN")==0){
                        goalnumber = GOAL_DOWN;
                    }else if(std::strcmp(name->GetText(),"GOAL_LEFT")==0){
                        goalnumber = GOAL_LEFT;
                    }else if(std::strcmp(name->GetText(),"GOAL_RIGHT")==0){
                        goalnumber = GOAL_RIGHT;
                    }else if(std::strcmp(name->GetText(),"GOAL_BEFORE_UE")==0){
                        goalnumber = GOAL_BEFORE_UE;
                    }else if(std::strcmp(name->GetText(),"GOAL_UE")==0){
                        goalnumber = GOAL_UE;
                    }else if(std::strcmp(name->GetText(),"GOAL_ZEBRA")==0){
                        goalnumber = GOAL_ZEBRA;
                    }

                    if(goalnumber!=99){
                        TiXmlElement *x = node->FirstChildElement( "x" );
                        if ( NULL != x )
                        {
                            array[goalnumber].x = std::stod(x->GetText());
                        }
                        TiXmlElement *y = node->FirstChildElement( "y" );
                        if ( NULL != y )
                        {
                            array[goalnumber].y = std::stod(y->GetText());
                        }
                    }



                }

                node = node->NextSiblingElement("goal");
            }
        }


    }else{
        ROS_INFO_STREAM("Couldn't find xml " + filename);
        return false;
    }

    return true;

}



void PathPlanner::interpolatePath(
        const nav_msgs::Path& path,
        nav_msgs::Path& smoothedPath)
{
    smoothedPath.header = path.header;
    interpolatePath(path.poses, smoothedPath.poses);
}


void PathPlanner::interpolatePath(
        const std::vector<geometry_msgs::PoseStamped>& path,
        std::vector<geometry_msgs::PoseStamped>& smoothedPath)
{
    // clear new smoothed path vector in case it's not empty
    smoothedPath.clear();

    // set skipPoints_ to 0 if the path contains has too few points
    unsigned int oldSkipPoints = skipPoints_;
    skipPoints_ = std::min<int>(path.size() - 2, skipPoints_);

    // create cummulative distances vector
    std::vector<double> cummulativeDistances;
    calcCummulativeDistances(path, cummulativeDistances);

    // create temp pose
    geometry_msgs::PoseStamped pose;
    pose.header = path[0].header;

    unsigned int numPoints = pointsPerUnit_ * calcTotalDistance(path);

    smoothedPath.resize(numPoints);

    // interpolate points on the smoothed path using the points in the original path
    for (unsigned int i = 0; i < numPoints; i++)
    {
        double u = static_cast<double>(i) / (numPoints-1);
        interpolatePoint(path, cummulativeDistances, pose, u);

        if (isnan(pose.pose.position.x) || isnan(pose.pose.position.y))
            pose.pose = smoothedPath[std::max(static_cast<int>(i)-1, 0)].pose;
        smoothedPath[i] = pose;
    }

    // copy start and goal orientations to smoothed path
    //smoothedPath.front().pose.orientation = path.front().pose.orientation;
    //smoothedPath.back().pose.orientation = path.back().pose.orientation;

    // interpolate orientations of intermediate poses
    for (unsigned int i = 1; i < smoothedPath.size()-1; i++)
    {
        double dx = smoothedPath[i+1].pose.position.x - smoothedPath[i].pose.position.x;
        double dy = smoothedPath[i+1].pose.position.y - smoothedPath[i].pose.position.y;
        double th = atan2(dy, dx);
        //smoothedPath[i].pose.orientation = tf::createQuaternionMsgFromYaw(th);
    }

    // revert skipPoints to original value
    skipPoints_ = oldSkipPoints;
}


void PathPlanner::interpolatePoint(
        const std::vector<geometry_msgs::PoseStamped>& path,
        const std::vector<double>& cummulativeDistances,
        geometry_msgs::PoseStamped& point,
        double pointCummDist)
{
    unsigned int group = findGroup(cummulativeDistances, pointCummDist);
    // ROS_INFO("u: %f, idx: %u", pointCummDist, group);

    double a = calcAlphaCoeff(path, cummulativeDistances, group, pointCummDist);
    double b = calcBetaCoeff(path, cummulativeDistances, group, pointCummDist);
    double c = calcGammaCoeff(path, cummulativeDistances, group, pointCummDist);
    double d = calcDeltaCoeff(path, cummulativeDistances, group, pointCummDist);

    std::vector<double> grad, nextGrad;
    calcPointGradient(path, cummulativeDistances, group, grad);
    calcPointGradient(path, cummulativeDistances, group+1, nextGrad);

    point.pose.position.x =
            + a * path[group*(skipPoints_+1)].pose.position.x
            + b * path[(group+1)*(skipPoints_+1)].pose.position.x
            + c * grad[0]
            + d * nextGrad[0];

    point.pose.position.y =
            + a * path[group*(skipPoints_+1)].pose.position.y
            + b * path[(group+1)*(skipPoints_+1)].pose.position.y
            + c * grad[1]
            + d * nextGrad[1];
}


void PathPlanner::calcCummulativeDistances(
        const std::vector<geometry_msgs::PoseStamped> path,
        std::vector<double>& cummulativeDistances)
{
    cummulativeDistances.clear();
    cummulativeDistances.push_back(0);

    for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1)
        cummulativeDistances.push_back(
                cummulativeDistances.back()
                + calcDistance(path, i) / calcTotalDistance(path));
}


double PathPlanner::calcTotalDistance(
        const std::vector<geometry_msgs::PoseStamped>& path)
{
    double totalDist = 0;

    for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1)
        totalDist += calcDistance(path, i);

    return totalDist;
}


double PathPlanner::calcDistance(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int idx)
{
    if (idx <= 0 || idx >=path.size())
        return 0;

    double dist =
            hypot(
                    path[idx].pose.position.x - path[idx-skipPoints_-1].pose.position.x,
                    path[idx].pose.position.y - path[idx-skipPoints_-1].pose.position.y);

    return dist;
}


double PathPlanner::calcAlphaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input)
{
    double alpha =
            + 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
            - 3 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2)
            + 1;

    return alpha;
}


double PathPlanner::calcBetaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input)
{
    double beta =
            - 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
            + 3 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2);

    return beta;
}


double PathPlanner::calcGammaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input)
{
    double gamma =
            (pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
             - 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2))
            * (cummulativeDistances[idx+1] - cummulativeDistances[idx])
            + input
            - cummulativeDistances[idx];

    return gamma;
}


double PathPlanner::calcDeltaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input)
{
    double delta =
            (pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
             - pow(calcRelativeDistance(cummulativeDistances, idx, input), 2))
            * (cummulativeDistances[idx+1] - cummulativeDistances[idx]);

    return delta;
}


double PathPlanner::calcRelativeDistance(
        const std::vector<double>& cummulativeDistances,
        const unsigned int idx,
        const double input)
{
    double relDist =
            (input - cummulativeDistances[idx])
            / (cummulativeDistances[idx+1] - cummulativeDistances[idx]);
    return relDist;
}


void PathPlanner::calcPointGradient(
        const std::vector<geometry_msgs::PoseStamped>& path,
        const std::vector<double>& cummulativeDistances,
        unsigned int idx,
        std::vector<double>& gradient)
{
    double dx, dy, du;
    gradient.assign(2, 0);

    // use either pose.yaw or interpolation to find gradient of points
//    if ((useEndConditions_ && (idx == 0 || idx == cummulativeDistances.size()-1))
//        || useMiddleConditions_)
//    {
//        double th = tf::getYaw(path[idx*(skipPoints_+1)].pose.orientation);
//        int sign = (fabs(th) < M_PI / 2) ? 1 : -1;
//
//        gradient[0] = sign * calcTotalDistance(path)
//                      * sqrt(1 + pow(tan(th),2)) / (1 + pow(tan(th), 2));
//        gradient[1] = tan(th) * gradient[0];
//    }
//    else  // gradient interpolation using original points
//    {
        if (idx == 0 || idx == cummulativeDistances.size()-1)
            return;

        dx = path[(idx)*(skipPoints_+1)].pose.position.x - path[(idx-1)*(skipPoints_+1)].pose.position.x;
        dy = path[(idx)*(skipPoints_+1)].pose.position.y - path[(idx-1)*(skipPoints_+1)].pose.position.y;
        du = cummulativeDistances[idx] - cummulativeDistances[idx-1];

        gradient[0] =  dx / du;
        gradient[1] =  dy / du;
//    }
}


unsigned int PathPlanner::findGroup(
        const std::vector<double>& cummulativeDistances,
        double pointCummDist)
{
    unsigned int i;
    for (i = 0; i < cummulativeDistances.size()-1; i++)
    {
        if (pointCummDist <= cummulativeDistances[i+1])
            return i;
    }
    return i;
}

bool PathPlanner::inRange(geometry_msgs::Point point, double range){
    bool result = false;
    geometry_msgs::Point position;
    try {
        const geometry_msgs::TransformStamped tf_ros = tfBuffer_.lookupTransform("world", "vehicle_front_axle", ros::Time(0));
        position.x = tf_ros.transform.translation.x;
        position.y = tf_ros.transform.translation.y;
    } catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM(e.what());
        return false;
    }



    double dx=position.x-point.x;
    double dy=position.y-point.y;
    if(sqrt(dx*dx + dy*dy)<=range){
        result = true;
    }
    return result;
}


} // namespace path_planner_ros_tool
