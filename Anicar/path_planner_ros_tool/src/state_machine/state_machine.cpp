#include "state_machine.hpp"

namespace path_planner_ros_tool {

    const int StateMachine::INTERSECTION = 1;
    const int StateMachine::UNKNOWN_ENVIRONMENT = 3;
    const int StateMachine::ENTERING_INTERSECTION = 2;
    const int StateMachine::DRIVE = 0;
    const int StateMachine::STARTING = 99;

    const int StateMachine::GOAL_UP = 1;
    const int StateMachine::GOAL_DOWN = 2;
    const int StateMachine::GOAL_LEFT = 3;
    const int StateMachine::GOAL_RIGHT = 4;
    const int StateMachine::GOAL_BEFORE_UE = 5;
    const int StateMachine::GOAL_UE = 0;
    const int StateMachine::GOAL_ZEBRA = 6;

StateMachine::StateMachine(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */


    state_ = STARTING;

    if(!importGoals(goalCoordinates, "goals.xml")){
        return;
    }
    if(!importGoals(beforeGoalCoordinates, "before_goals.xml")){
        return;
    }


    interface_.fromParamServer();
    lastDirectionSign_ = {ros::Time::now(),99,false,true};

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/StateMachine.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&StateMachine::reconfigureRequest, this, _1, _2));
    sub_stop = nhPrivate.subscribe("/stop/detected_signs",1000,&StateMachine::callbackStopSign, this); //subscriber for stop sign
    sub_direct = nhPrivate.subscribe("/go_direction/detected_signs",1000,&StateMachine::callbackDirectionSign, this); //subscriber for direction sign
    sub_zebra = nhPrivate.subscribe("/robot_detector/state_publisher",1000,&StateMachine::callbackZebra,this);
    pub_goal = nhPrivate.advertise<std_msgs::Int8>("goal",10);
    pub_state = nhPrivate.advertise<std_msgs::Int8>("state",10);

    rosinterface_handler::showNodeInfo();
    bool zebra = false;
    if(!nhPublic.getParam("with_zebra",zebra)){
        ROS_INFO_STREAM("didnt found Param");
    }
    if(zebra){
        ROS_INFO_STREAM("Additionaltask activated");
    }

    ros::Duration(10).sleep();


    while(ros::ok()){
        switch(state_){
            case STARTING:
                lastGoal_ = 0;
                if(zebra){
                    goal_ = GOAL_ZEBRA;
                }else{
                    goal_ = GOAL_UP;
                }
                publishCurrentGoal();
                updateState(DRIVE);
                break;
            case INTERSECTION:
                //Wait if stop and give next goal
                //change to DRIVE;
                handleIntersection();
                break;
            case ENTERING_INTERSECTION:
                //Activate Sign recognition, if in Range of Goal change to INTERSECTION
                handleEnteringIntersection();
                break;
            case UNKNOWN_ENVIRONMENT:
                //basicly do nothing, deactivate sign recognition
                break;
            case DRIVE:
                //if pass beforeGoal Coordinate it will change to ENTERING_INTERSECTION
                handleDrive();
                break;
        }

        ros::spinOnce();
        //ROS_INFO_STREAM("New Map Published");

    }

}

void StateMachine::callbackZebra(const std_msgs::Int8 msg){
    if((int)msg.data ==0){
        lastDirectionSign_.hasToStop = false;
    }else if ((int)msg.data ==1){
        lastDirectionSign_.hasToStop = true;
    }
}

void StateMachine::callbackStopSign(const std_msgs::Int8 msg){
    if(state_==ENTERING_INTERSECTION){
        lastDirectionSign_.hasToStop = true;
    }

}

void StateMachine::callbackDirectionSign(const std_msgs::Int8 msg){
    if(state_==ENTERING_INTERSECTION){

        lastDirectionSign_.time = ros::Time::now();
        lastDirectionSign_.direction =(int)msg.data;
    }

}

/**
 * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
 */
void StateMachine::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}

void StateMachine::updateState(int state){
    std_msgs::Int8 myState;
    myState.data = state;
    state_ = state;
    ROS_INFO_STREAM("change state to: "<<state_);
    pub_state.publish(myState);

}

bool StateMachine::inRange(geometry_msgs::Point point, double range){
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

bool StateMachine::importGoals(geometry_msgs::Point*& array, std::string filename){
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


void StateMachine::handleIntersection(){
    if(goal_==GOAL_ZEBRA && lastDirectionSign_.hasToStop){
        return;
    }

    if(lastDirectionSign_.hasToStop){
        ros::Duration(3).sleep();
    }
    //calculate next Goal
    lastGoal_ = goal_;
    int direction = lastDirectionSign_.direction;
    switch(lastGoal_){
        case GOAL_UP:
            switch(direction){
                case lastDirectionSign_.LEFT:
                    goal_ = GOAL_BEFORE_UE;
                    break;
                case lastDirectionSign_.RIGHT:
                    goal_ = GOAL_DOWN;
                    break;
                case lastDirectionSign_.FORWARD:
                    goal_ = GOAL_LEFT;
                    break;
                default:
                    goal_ = GOAL_LEFT;
                    break;
            }
            break;
        case GOAL_DOWN:
            switch(direction){
                case lastDirectionSign_.LEFT:
                    goal_ = GOAL_DOWN;
                    break;
                case lastDirectionSign_.RIGHT:
                    goal_ = GOAL_BEFORE_UE;
                    break;
                case lastDirectionSign_.FORWARD:
                    goal_ = GOAL_RIGHT;
                    break;
                default:
                    goal_ = GOAL_RIGHT;
                    break;
            }
            break;
        case GOAL_LEFT:
            switch(direction){
                case lastDirectionSign_.LEFT:
                    goal_ = GOAL_RIGHT;
                    break;
                case lastDirectionSign_.RIGHT:
                    goal_ = GOAL_LEFT;
                    break;
                case lastDirectionSign_.FORWARD:
                    goal_ = GOAL_BEFORE_UE;
                    break;
                default:
                    goal_ = GOAL_BEFORE_UE;
                    break;
            }
            break;
        case GOAL_RIGHT:
            switch(direction){
                case lastDirectionSign_.LEFT:
                    goal_ = GOAL_LEFT;
                    break;
                case lastDirectionSign_.RIGHT:
                    goal_ = GOAL_RIGHT;
                    break;
                case lastDirectionSign_.FORWARD:
                    goal_ = GOAL_DOWN;
                    break;
                default:
                    goal_ = GOAL_DOWN;
                    break;
            }
            break;
        case GOAL_BEFORE_UE:
            switch(direction){
                case lastDirectionSign_.LEFT:
                    goal_ = GOAL_UP;
                    break;
                case lastDirectionSign_.RIGHT:
                    goal_ = GOAL_UE;
                    break;
                case lastDirectionSign_.FORWARD:
                    goal_ = GOAL_UP;
                    break;
                default:
                    goal_ = GOAL_UP;
                    break;
            }
            break;
        case GOAL_ZEBRA:
            break;
    }

    publishCurrentGoal();
    if(goal_ == GOAL_UE){
        updateState(UNKNOWN_ENVIRONMENT);
    }else{
        updateState(DRIVE);
    }
}

void StateMachine::handleEnteringIntersection(){
    if(inRange(goalCoordinates[goal_],0.5)){
        updateState(INTERSECTION);
    }
}

void StateMachine::handleDrive(){
    if(inRange(beforeGoalCoordinates[goal_],0.5)){
        updateState(ENTERING_INTERSECTION);
        lastDirectionSign_ = {ros::Time::now(),99,false,true};
    }


}


void StateMachine::publishCurrentGoal(){
    std_msgs::Int8 goal;
    goal.data = goal_;
    pub_goal.publish(goal);
    ROS_INFO_STREAM("Published new goal");
}


} // namespace path_planner_ros_tool
