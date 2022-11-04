//
// Created by Johan on 04.11.2022.
//

#include "jps_maze_client/client.hpp"

namespace jps_maze_client {
    Client::Client(rclcpp::NodeOptions node_options)
            : rclcpp::Node("client_node", node_options) {

    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
}