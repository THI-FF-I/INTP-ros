//
// Created by Johan on 30.10.2022.
//

#ifndef INTP_ROS_LISTENER_HPP
#define INTP_ROS_LISTENER_HPP

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/parameter.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node {
public:
    Listener(const std::string& node_name, const std::string& sub_namespace, const rclcpp::NodeOptions&	options = rclcpp::NodeOptions());
private:
    std::string target_topic;
    void subscriber_cb(const std_msgs::msg::String& message) const;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> subscriber;
};

#endif //INTP_ROS_LISTENER_HPP
