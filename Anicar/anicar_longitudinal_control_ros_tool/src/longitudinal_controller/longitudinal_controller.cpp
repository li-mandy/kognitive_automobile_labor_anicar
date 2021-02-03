#include "longitudinal_controller.hpp"
#include <utils_ros/ros_console.hpp>

#include "motor_interface_ros_tool/MotorCommand.h"
#include <cmath>

namespace anicar_longitudinal_control_ros_tool {

    using MotorCommand = motor_interface_ros_tool::MotorCommand;

    LongitudinalController::LongitudinalController(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
            : params_{nh_private}, reconfigureServer_{nh_private} {

        /**
         * Initialization
         */
        utils_ros::setLoggerLevel(nh_private);
        params_.fromParamServer();

        /**
         * Publishers & subscriber
         */

        motor_command_publisher_ = nh_private.advertise<MotorCommand>(params_.motor_command_topic, params_.msg_queue_size);

        // Instantiate subscriber last, to assure all objects are initialised when first message is received.

        path_curv_subscriber_ = nh_private.subscribe("/lateral_controller/path_curvature",
                                                     5,
                                                     &LongitudinalController::pathCurvatureCallback,
                                                     this,
                                                     ros::TransportHints().tcpNoDelay());

        path_progress_subscriber_ = nh_private.subscribe("/lateral_controller/path_progress",
                                                     5,
                                                     &LongitudinalController::pathProgressCallback,
                                                     this,
                                                     ros::TransportHints().tcpNoDelay());

        driving_state_subscriber_ = nh_private.subscribe("/state_machine/state",
                                                         5,
                                                         &LongitudinalController::pathProgressCallback,
                                                         this,
                                                         ros::TransportHints().tcpNoDelay());

        control_loop_timer_ =
                nh_private.createTimer(ros::Rate(params_.control_loop_rate), &LongitudinalController::controlLoopCallback, this);

        /**
         * Set up dynamic reconfiguration
         */
        reconfigureServer_.setCallback(boost::bind(&LongitudinalController::reconfigureRequest, this, _1, _2));

        utils_ros::showNodeInfo();
    }

    void LongitudinalController::pathCurvatureCallback(const std_msgs::Float64 msg_pc) {
        curv = msg_pc.data;
    }

    void LongitudinalController::pathProgressCallback(const std_msgs::Int16 msg_pp) {
        path_entries_to_end = msg_pp.data;
        ROS_INFO_STREAM("Path entries to end: " << path_entries_to_end);
    }

    void LongitudinalController::drivingStateCallback(const std_msgs::Int8 msg_ds) {
        driving_state = msg_ds.data;
    }

    void LongitudinalController::controlLoopCallback(const ros::TimerEvent& timer_event) {
        double vel_calc;
        double vel_publish;
        switch (params_.long_contr_type) {
            // Constant velocity
            case 1: {
                vel_publish = params_.v_const;
                break;
            }

            // Velocity depending on the curvature
            case 2: {
                vel_calc = params_.v_max - (params_.v_max - params_.v_min) / params_.curv_max * curv;

                // Check if vel_calc is not a number (happens when approaching the end of the path due to curvature calculation)
                if (std::isnan(vel_calc)) {
                    vel_calc = params_.v_min;
                }

                // Make sure that the velocity limits are respected
                if (vel_calc > params_.v_max) {
                    vel_publish = params_.v_max;
                } else if (vel_calc < params_.v_min) {
                    vel_publish = params_.v_min;
                } else {
                    vel_publish = vel_calc;
                }
                //vel_publish = boost::algorithm::clamp(vel_calc, params_.v_min, params_.v_max)
                break;
            }

            // Velocity fast - slow - fast
            //case 3: { ... }
        }

        MotorCommand motor_command;
        motor_command.header.stamp = timer_event.current_expected;

        //ROS_INFO_STREAM("Path entries to end: " << path_entries_to_end);

        // Check if path is not over, otherwise wait
        /*if (path_entries_to_end > 10) {
            motor_command.velocity = params_.v_fast;
        } else if ((path_entries_to_end <= 10) && (path_entries_to_end > 1)) {
            motor_command.velocity = params_.v_slow;
        }
        else {
            motor_command.velocity = 0.0;
        }*/

        // Velocity depending on the state
        // driving_state =  0 --> DRIVE
        // driving_state =  1 --> INTERSECTION
        // driving_state =  2 --> ENTERING_INTERSECTION
        // driving_state =  3 --> UNKNOWN_ENVIRONMENT
        // driving_state = 99 --> STARTING

        if (path_entries_to_end < 2) {
            driving_state = 5; // --> STOP
        }
        else{
            driving_state = driving_state;
        }

        ROS_INFO_STREAM("Driving Staaate: " << driving_state);

        switch (driving_state) {
            case 0: {
                motor_command.velocity = 0.6;
                break;
            }
            case 1: {
                if (path_entries_to_end > 1) {
                    motor_command.velocity = 0.2;
                }
                else {
                    motor_command.velocity = 0.0;
                }
                break;
            }
            case 2: {
                motor_command.velocity = 0.4;
                break;
            }
            case 3: {
                motor_command.velocity = 0.3;
                break;
            }
            case 99: {
                motor_command.velocity = 0.6;
                break;
            }
        }

        motor_command_publisher_.publish(motor_command);
    }

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
    void LongitudinalController::reconfigureRequest(const Config& config, uint32_t level) {
        params_.fromConfig(config);
    }


} // namespace anicar_longitudinal_control_ros_tool
