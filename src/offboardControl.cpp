#include "offboardControl.hpp"
using std::placeholders::_1;

offboardControl::offboardControl() : Node("offboard_control_node"){
	
	// Publisher
	global_pose_publisher = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/astro_sim/setpoint_position/global", 10);

	tol_client = this->create_client<mavros_msgs::srv::CommandTOL>("/astro_sim/cmd/takeoff");
	arm_client = this->create_client<mavros_msgs::srv::CommandBool>("/astro_sim/cmd/arming");
	set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("/astro_sim/set_mode");


	offboard_str_ = std::string("OFFBOARD");
	
	//Get controller parameters
    this->declare_parameter("x_axis", -1);
    this->declare_parameter("y_axis", -1);
    this->declare_parameter("z_axis", -1);
    this->declare_parameter("yaw_axis", -1);

    this->declare_parameter("x_vel_max", 1.0);
    this->declare_parameter("y_vel_max", 1.0);
    this->declare_parameter("z_vel_max", 1.0);
    this->declare_parameter("yaw_vel_max", 1.0);

    this->declare_parameter("arm_button", -1);
    this->declare_parameter("disarm_button", -1);
    this->declare_parameter("takeoff_button", -1);
    this->declare_parameter("land_button", -1);

    this->get_parameter("x_axis", axes_.x.axis);
    this->get_parameter("y_axis", axes_.y.axis);
    this->get_parameter("z_axis", axes_.z.axis);
    this->get_parameter("yaw_axis", axes_.yaw.axis);

    this->get_parameter("x_vel_max", axes_.x.factor);
    this->get_parameter("y_vel_max", axes_.y.factor);
    this->get_parameter("z_vel_max", axes_.z.factor);
    this->get_parameter("yaw_vel_max", axes_.yaw.factor);

    this->get_parameter("arm_button", buttons_.arm.button);
    this->get_parameter("disarm_button", buttons_.disarm.button);
    this->get_parameter("takeoff_button", buttons_.takeoff.button);
    this->get_parameter("land_button", buttons_.land.button);

    RCLCPP_INFO(this->get_logger(), "Loaded controller parameters:\nX: %d, Y: %d, Z: %d, Yaw: %d, Arm: %d, Disarm: %d, Takeoff: %d, Land: %d", 
        axes_.x.axis, axes_.y.axis, axes_.z.axis, axes_.yaw.axis,buttons_.arm.button, buttons_.disarm.button, buttons_.takeoff.button, buttons_.land.button);

    // NOT SURE WHAT SUB_QOS IS USED FOR, USED IN WILLIE'S CODE
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
    sub_qos.best_effort();
    sub_qos.durability_volatile();
    //==========//

    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&offboardControl::joy_callback, this, _1));
    state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>("/astro_sim/state", sub_qos, std::bind(&offboardControl::state_callback, this, _1));
    ext_state_subscriber_ = this->create_subscription<mavros_msgs::msg::ExtendedState>("extended_state", sub_qos, std::bind(&offboardControl::ext_state_callback, this, _1));
    global_gpos_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/astro_sim/global_position/global", sub_qos, std::bind(&offboardControl::pose_callback, this, _1));
	global_lpos_sub = this->create_subscription<nav_msgs::msg::Odometry>("/astro__sim/global_position/local", 10, std::bind(&offboardControl::lpos_callback, this, _1));

}

void offboardControl::send_arming_request(bool arm) {

    if (arm) {
        if (!current_state.armed) {
            RCLCPP_WARN(this->get_logger(), "Sending arm request.");
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = arm;
            auto arm_result = arm_client->async_send_request(arm_request, std::bind(&offboardControl::arm_response_callback, this, _1));
         }
         else {
            RCLCPP_WARN(this->get_logger(), "Device already armed - arm request ignored.");
            return;
            } 
	}
	else {
        if (current_state.armed) {
            RCLCPP_WARN(this->get_logger(), "Sending disarm request.");
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = arm;
            auto arm_result = arm_client->async_send_request(arm_request, std::bind(&offboardControl::arm_response_callback, this, _1));
        } else {
            RCLCPP_WARN(this->get_logger(), "Device already disarmed - disarm request ignored.");
            return;
        }
    }
    
}
void offboardControl::setMode_request() {
	rclcpp::Rate rate(10);
	geographic_msgs::msg::GeoPoseStamped poseStamped;
	geographic_msgs::msg::GeoPoint position;

	position.latitude = latitude;
	position.longitude = longitude;
	position.altitude = altitude + 1.0f;

	poseStamped.pose.position = position;
	poseStamped.pose.orientation = orientation;

	for(int i = 0; i < 100; i++) {
		global_pose_publisher->publish(poseStamped);
		rate.sleep();
	}

	auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";
        auto set_mode_result = set_mode_client->async_send_request(request, std::bind(&offboardControl::setMode_response_callback, this, _1));
        global_pose_publisher->publish(poseStamped);
}
void offboardControl::send_takeoff_request() {
	auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

	//request->min_pitch = asin(-2.0*(orientation.x*orientation.z - orientation.w*orientation.y));
	request->yaw = quat_to_yaw(orientation);
	request->altitude = altitude + 50000.0f; 
	request->latitude = latitude + 5000.0f;
	request->longitude = longitude + 5000.0f;

	auto tol_result = tol_client->async_send_request(request, std::bind(&offboardControl::tol_response_callback, this, _1));
}

void offboardControl::tol_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
    auto response = future.get();

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "TOL request succeeded. Result=%d", response->result);
    } else {
        RCLCPP_ERROR(this->get_logger(), "TOL request failed!");
    }
}

void offboardControl::arm_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
    auto response = future.get();

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Arm/disarm request succeeded. Result=%d", response->result);

    } else {
        RCLCPP_ERROR(this->get_logger(), "Arm/disarm request failed!");
    }
}

void offboardControl::state_callback(const mavros_msgs::msg::State::SharedPtr state_msg) {
    
    // std::cout << current_state_.mode << std::endl;
    // RCLCPP_INFO(this->get_logger(), "Got state. %d", msg->armed);

    if (state_msg->armed && !current_state.armed) {
        RCLCPP_WARN(this->get_logger(), "Armed");
    } else if (!state_msg->armed && current_state.armed) {
        RCLCPP_WARN(this->get_logger(), "Disarmed");
    }

    if (state_msg->mode == offboard_str_ && current_state.mode != offboard_str_) {
        RCLCPP_WARN(this->get_logger(), "Offboard mode enabled.");
    } else if (state_msg->mode != offboard_str_ && current_state.mode == offboard_str_) {
        RCLCPP_WARN(this->get_logger(), "Offboard mode disabled.");
    }

    //If the device isn't in OFFBOARD mode, request it ONCE (if it's turned off during operation, assume manual takeover)
    // if (!offboard_requested_) {
    //     if (state_msg->mode != offboard_str_) {
    //         //Put in offboard mode at all times while this node is running
    //         RCLCPP_WARN(this->get_logger(), "Offboard mode requested.");
    //         auto offboard_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    //         offboard_request->custom_mode = "OFFBOARD";
    //         auto mode_result = set_mode_client_->async_send_request(offboard_request, std::bind(&AstroTeleop::mode_response_callback, this, _1));
    //     }
    //     //Set to true regardless
    //     offboard_requested_ = true;
    // }

    current_state = *state_msg;
}

void offboardControl::ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg) {
    // uint8 LANDED_STATE_UNDEFINED=0
    // uint8 LANDED_STATE_ON_GROUND=1
    // uint8 LANDED_STATE_IN_AIR=2
    // uint8 LANDED_STATE_TAKEOFF=3
    // uint8 LANDED_STATE_LANDING=4

    if ((LandedState)ext_state_msg->landed_state != landed_state_) {
        switch (ext_state_msg->landed_state) {
            case undefined: {
                RCLCPP_ERROR(this->get_logger(), "Undefined landed state!");
                break;
            } case on_ground: {
                RCLCPP_WARN(this->get_logger(), "Entered on ground state.");
                break;
            } case in_air: {
                RCLCPP_WARN(this->get_logger(), "Entered in air state.");

                //Put drone in offboard mode to hold position
                // RCLCPP_WARN(this->get_logger(), "Requesting offboard mode.");
                // auto offboard_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                // offboard_request->custom_mode = "OFFBOARD";
                // auto mode_result = set_mode_client_->async_send_request(offboard_request, std::bind(&AstroTeleop::mode_response_callback, this, _1));

                break;
            } case takeoff: {
                RCLCPP_WARN(this->get_logger(), "Entered takeoff state.");
                break;
            } case landing: {
                RCLCPP_WARN(this->get_logger(), "Entered landing state.");
                break;
            }
        }

        landed_state_ = (LandedState)ext_state_msg->landed_state;
    }
}

void offboardControl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    //Check for arm button press
    int arm_button_state = get_button(joy_msg, buttons_.arm);

     if (arm_button_state != button_state_.arm.state) {
        if (arm_button_state == 1) {
            //Arm or takeoff, depending on the state
            if (!current_state.armed) {
                //Arm
                send_arming_request(true);
            } 
        }
        button_state_.arm.state = arm_button_state;
    }

    // int takeoffButtonPress = get_button(joy_msg, buttons_.takeoff);
    // if(takeoffButtonPress == 1) {
    // 	RCLCPP_WARN(this->get_logger(), "Takeoff Request has been made");
    // 	send_takeoff_request();
    // }
    int setModeButton = get_button(joy_msg, buttons_.takeoff);
    if(setModeButton == 1) {
    	RCLCPP_WARN(this->get_logger(), "Offboard mode request has been made");
    	setMode_request();
    }


    //Check for disarm button press
    //int disarm_button_state = get_button(joy_msg, buttons_.disarm);
    //if (disarm_button_state != button_state_.disarm.state) {
      //  if (disarm_button_state == 1) {
            //Land or disarm, depending on the state
        //    if (landed_state_ != on_ground) {
                //Land
          //      send_tol_request(false);
            //} else {
                //Disarm
              //  send_arming_request(false);
            //}
        //}
       // button_state_.disarm.state = disarm_button_state;
    //}

    
    // RCLCPP_WARN(this->get_logger(), "Sending vel msg.");
    // int takeoff_button_state = get_button(joy_msg, buttons_.takeoff);
    // if (takeoff_button_state != button_state_.takeoff.state) {
    //     if (takeoff_button_state == 1) {
    //         RCLCPP_WARN(this->get_logger(), "Taking off");
    //         takeoff(true);
    //     }
    //     button_state_.takeoff.state = takeoff_button_state;
    // }

    // int land_button_state = get_button(joy_msg, buttons_.land);
    // if (land_button_state != button_state_.land.state) {
    //     if (land_button_state == 1) {
    //         RCLCPP_WARN(this->get_logger(), "Landing");
    //         takeoff(false);
    //     }
    //     button_state_.land.state = land_button_state;
    // }
}

void offboardControl::pose_callback(const sensor_msgs::msg::NavSatFix::SharedPtr pose_msg) {
	latitude = pose_msg->latitude;
	longitude = pose_msg->longitude;
	altitude = pose_msg->altitude;
}
void offboardControl::lpos_callback(const nav_msgs::msg::Odometry::SharedPtr lpos_msg) {
	orientation = ((lpos_msg->pose).pose).orientation;
}
void offboardControl::setMode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
	auto response = future.get();

	if(response->mode_sent)
		RCLCPP_WARN(this->get_logger(), "MODE SET TO OFFBOARD!");
	else {
		RCLCPP_WARN(this->get_logger(), "FAILED TO ENTER OFFBOARD MODE");
	}

}

double offboardControl::get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis) {
    if (axis.axis < 0 || std::abs(axis.axis) > (int)joy_msg->axes.size()-1) {
        RCLCPP_ERROR(this->get_logger(), "Axis %d out of range, joy has %d axes", axis.axis, joy_msg->axes.size());
        return -1;
    }

    double output = joy_msg->axes[std::abs(axis.axis)] * axis.factor + axis.offset;

    return output;
}

int offboardControl::get_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button) {
    if (button.button < 0 || button.button > (int)joy_msg->buttons.size()-1) {
        RCLCPP_ERROR(this->get_logger(), "Button %d out of range, joy has %d buttons", button.button, joy_msg->buttons.size());
        return -1;
    }
    return joy_msg->buttons[button.button];
}

double offboardControl::quat_to_yaw(geometry_msgs::msg::Quaternion q) {
	return atan2(2.0*(q.y*q.z + q.w*q.x),
	 q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
}