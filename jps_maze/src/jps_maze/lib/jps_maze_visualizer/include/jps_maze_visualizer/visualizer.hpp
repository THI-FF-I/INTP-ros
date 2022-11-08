//
// Created by Johan on 08.11.2022.
//

#ifndef INTP_ROS_VISUALIZER_HPP
#define INTP_ROS_VISUALIZER_HPP

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <netinet/in.h>

#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace jps_maze_visualizer {

    using block_t = uint8_t;

    class Visualizer {
    public:
        Visualizer(const std::string_view host_name, const uint16_t port, const uint8_t width, const uint8_t height, block_t ***frame_buffer, rclcpp::Logger logger);
        ~Visualizer();
        void re_draw();
    private:
        uint8_t width;
        uint8_t height;
        uint8_t **frame_buffer;
        rclcpp::Logger logger;
        int network_socket;
        struct sockaddr_in server_address;
    };

}

#endif //INTP_ROS_VISUALIZER_HPP
