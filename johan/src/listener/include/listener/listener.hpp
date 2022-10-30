//
// Created by Johan on 30.10.2022.
//

#ifndef INTP_ROS_LISTENER_HPP
#define INTP_ROS_LISTENER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/client.hpp"

#include "turtlesim/srv/teleport_absolute.hpp"

#include "custom_msgs/msg/pose2_d_stamped.hpp"

class Listener : public rclcpp::Node {
public:
    Listener(const std::string& node_name, const std::string& sub_namespace, const rclcpp::NodeOptions&	options = rclcpp::NodeOptions());
private:
    std::string target_topic;
    std::string turtle_name;
    void init_parameters();
    void subscriber_cb(const custom_msgs::msg::Pose2DStamped & message) const;
    std::shared_ptr<rclcpp::Subscription<custom_msgs::msg::Pose2DStamped>> subscriber;
    std::shared_ptr<rclcpp::Client<turtlesim::srv::TeleportAbsolute>> client;
    double time_delta_ms(const builtin_interfaces::msg::Time &time_send) const;
};

#endif //INTP_ROS_LISTENER_HPP
