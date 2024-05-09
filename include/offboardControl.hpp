#ifndef OFFBOARD_CONTROL_HPP
#define OFFBOARD_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geodesy/utm.h>
#include <string>

#include<geometry_msgs/msg/pose_stamped.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<geographic_msgs/msg/geo_point.hpp>
#include<geographic_msgs/msg/geo_pose_stamped.hpp>
#include<GeographicLib/Geoid.hpp>

#include<mavros_msgs/msg/state.hpp>
#include<mavros_msgs/msg/extended_state.hpp>
#include<mavros_msgs/srv/command_tol.hpp>
#include<mavros_msgs/srv/command_bool.hpp>
#include<mavros_msgs/srv/set_mode.hpp>

#include<sensor_msgs/msg/nav_sat_fix.hpp>
#include<sensor_msgs/msg/joy.hpp>
#include<nav_msgs/msg/odometry.hpp>

class offboardControl : public rclcpp::Node {
private:
	// publishers
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr global_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;


	// subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_gpos_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_lpos_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber;

	// Service Clients for Mode, Arm/Disarm, Takeoff/Landing
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr tol_client;

    // msg types
    geometry_msgs::msg::Pose current_pose_;
    geographic_msgs::msg::GeoPoseStamped global_setpoint_pose;
    mavros_msgs::msg::State current_state;
    geometry_msgs::msg::Quaternion orientation;
    geometry_msgs::msg::Twist setpoint_velocity;


    float latitude, longitude, altitude;
    std::string control_mode;
    std::string offboard_str_;

    struct Axis {
        Axis() : axis(0), factor(0.0), offset(0.0) {}

        int axis;
        double factor;
        double offset;
    };

    struct Button {
        Button() : button(0) {}
        int button;
    };

    struct ButtonState {
        ButtonState() : state(0) {}
        int state;
    };

    struct {
        Axis x;
        Axis y;
        Axis z;
        Axis yaw;
    } axes_;

    struct {
        Button arm;
        Button disarm;
        Button takeoff;
        Button land;
    } buttons_;

    //Button state for debouncing
    struct {
        ButtonState arm;
        ButtonState disarm;
        ButtonState takeoff;
        ButtonState land;
    } button_state_;
    enum LandedState {
        undefined = 0,
        on_ground,
        in_air,
        takeoff,
        landing
    };

    LandedState landed_state_;
    bool offboard_requested_;

    // Service request functions
    void send_takeoff_request();
    void send_land_request();
    void setMode_request();
    void send_arming_request(bool arm);

    // Service request callback functions
    void setMode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture ptr);
    void arm_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture ptr);
    void tol_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture ptr);
    void state_callback(const mavros_msgs::msg::State::SharedPtr state_msg);
    void ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void pose_callback(const sensor_msgs::msg::NavSatFix::SharedPtr pose_msg);
    void lpos_callback(const nav_msgs::msg::Odometry::SharedPtr lpos_msg);
    void setpoint_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr velocity_msg);

    double quat_to_yaw(geometry_msgs::msg::Quaternion q);

    //Joy functions
    int get_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button);
    double get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis);

    

public:
	offboardControl();
};

#endif