#include "offboardControl.hpp"

offboardControl::offboardControl() : Node("offboard_control_node"){}

void offboardControl::send_takeoff_request() {
	auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
	// TODO
	//global_position_sub = this->create_subscription<geographic_msgs::msg::GeoPoint>("extended_state", sub_qos, std::bind(&AstroTeleop::ext_state_callback, this, _1));

	//request->min_pitch = 
	//request->yaw = 
	//request->altitude = 
	//request->latitude = 
	//request->longitude = 
}