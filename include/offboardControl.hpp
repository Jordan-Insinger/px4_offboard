#ifndef OFFBOARD_CONTROL_HPP
#define OFFBOARD_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>

#include<geographic_msgs/msg/geo_point.hpp>
#include<mavros_msgs/srv/command_tol.hpp>
#include<mavros_msgs/srv/command_bool.hpp>
#include<mavros_msgs/srv/set_mode.hpp>

#include <geodesy/utm.h>
#include <GeographicLib/Geoid.hpp>

class offboardControl : public rclcpp::Node {
private:
	// publishers
	//rclcpp::Publisher<geometry_msgs::msg::TwistStamped> SharedPtr velocity_publisher;
	//rclcpp::Publisher<geographic_msgs::msgs::PoseStamped> SharedPtr global_pose_publisher;

	// subscribers
	rclcpp::Subscription<geographic_msgs::msg::GeoPoint>::SharedPtr global_position_sub;
	//rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

	// Service Clients for Mode, Arm/Disarm, Takeoff/Landing
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr tol_client;


    // Service request functions
    void send_takeoff_request();
    void send_land_request();
    void send_mode_request();

    // Service request callback functions
    void setMode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture ptr);
    void arm_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture ptr);
    void tol_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture ptr);

public:
	offboardControl();
};

#endif