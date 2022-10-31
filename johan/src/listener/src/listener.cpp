//
// Created by Johan on 30.10.2022.
//
#include "listener/listener.hpp"

Listener::Listener(const std::string &node_name, const std::string &node_namespace, const rclcpp::NodeOptions &options)
: rclcpp::Node(node_name, node_namespace, options) {
    this->init_parameters();
    this->subscriber = this->create_subscription<custom_msgs::msg::Pose2DStamped>(this->target_topic, 10,
                                                                        std::bind(&Listener::subscriber_cb, this, std::placeholders::_1));
    this->client = this->create_client<turtlesim::srv::TeleportAbsolute>(this->get_effective_namespace() + "/_private/" + this->turtle_name + "/teleport_absolute");
    RCLCPP_INFO(this->get_logger(), "Node %s init successfull",this->get_fully_qualified_name());
}

void Listener::init_parameters() {
    this->declare_parameter<std::string>("target_topic");
    this->get_parameter_or<std::string>("target_topic", this->target_topic, "/INTP_ROS/talker_node/info");
    RCLCPP_INFO(this->get_logger(), "Got target topic: \"%s\"", this->target_topic.c_str());
    this->declare_parameter<std::string>("turtle_name");
    this->get_parameter_or<std::string>("turtle_name", this->turtle_name, "_turtlesim_node");
    RCLCPP_INFO(this->get_logger(), "Got turtlesim node name: \"%s\"", this->turtle_name.c_str());
}

void Listener::subscriber_cb(const custom_msgs::msg::Pose2DStamped &message) const {
    RCLCPP_INFO(this->get_logger(), "Got new target_pose x: %f, y: %f, \u03B8: %f, delay: %fms",
                message.pose.x, message.pose.y, message.pose.theta, this->time_delta_ms(message.header.stamp));
    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = message.pose.x;
    request->y = message.pose.y;
    request->theta = message.pose.theta;
    this->client->async_send_request(request);
}

double Listener::time_delta_ms(const builtin_interfaces::msg::Time &time_send) const {
    const rclcpp::Time now = this->now();
    const rclcpp::Time then(time_send);
    const rclcpp::Duration delta = now - then;
    return delta.seconds()*1000;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.start_parameter_event_publisher(false);
    options.enable_rosout(false);
    options.start_parameter_services(false);
    rclcpp::spin(std::make_shared<Listener>("talker_node", "INTP_ROS", options));
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
