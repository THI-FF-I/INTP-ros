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

/*#include <netinet/in.h>*/
#include <netdb.h>

#include <string>
#include <stdexcept>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace jps_maze_visualizer {

    using block_t = uint32_t;

    class Visualizer {
    public:
        Visualizer(const std::string_view host_name, const std::string_view port, const block_t width, const block_t height, block_t ***frame_buffer, rclcpp::Logger logger);
        Visualizer(rclcpp::Logger logger) : logger(logger), valid(false) {}
        Visualizer& operator=(Visualizer&& rhs);
        ~Visualizer();
        void re_draw();
    private:
        void send_dim();
        uint8_t width;
        uint8_t height;
        block_t **frame_buffer;
        rclcpp::Logger logger;
        bool valid;
        int network_socket;
        struct sockaddr server_address;
    };

}

#endif //INTP_ROS_VISUALIZER_HPP
