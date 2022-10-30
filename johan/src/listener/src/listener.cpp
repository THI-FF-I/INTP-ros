//
// Created by Johan on 30.10.2022.
//
#include "listener/listener.hpp"

Listener::Listener(const std::string &node_name, const std::string &node_namespace, const rclcpp::NodeOptions &options)
: rclcpp::Node(node_name, node_namespace, options) {
    this->declare_parameter<std::string>("target_topic");
    this->get_parameter_or<std::string>("target_topic", this->target_topic, "/INTP_ROS/talker_node/info");
    RCLCPP_INFO(this->get_logger(), "Got target topic: \"%s\"", this->target_topic.c_str());
    this->subscriber = this->create_subscription<std_msgs::msg::String>(this->target_topic, rclcpp::QoS(10),
                                                                        std::bind(&Listener::subscriber_cb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Node %s init successfull",this->get_fully_qualified_name());

}

void Listener::subscriber_cb(const std_msgs::msg::String &message) const {
    RCLCPP_INFO(this->get_logger(), "Got message: \"%s\"", message.data.c_str());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>("talker_node", "INTP_ROS"));
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
