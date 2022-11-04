//
// Created by Johan on 04.11.2022.
//

#ifndef INTP_ROS_SERVER_NODE_HPP
#define INTP_ROS_SERVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"

namespace jps_maze_server {

    class Server : public rclcpp::Node {
        Server(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());
    };
}

#endif //INTP_ROS_SERVER_NODE_HPP
