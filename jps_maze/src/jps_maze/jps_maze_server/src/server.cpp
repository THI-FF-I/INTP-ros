//
// Created by Johan on 04.11.2022.
//

#include "jps_maze_server/server.hpp"

namespace jps_maze_server {
    Server::Server(rclcpp::NodeOptions node_options)
        : rclcpp::Node("server_node", node_options) {

    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
}
