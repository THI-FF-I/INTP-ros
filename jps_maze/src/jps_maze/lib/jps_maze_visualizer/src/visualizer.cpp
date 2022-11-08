#include "jps_maze_visualizer/visualizer.hpp"

namespace jps_maze_visualizer{

    Visualizer::Visualizer(const std::string_view host_name, const uint16_t port, const uint8_t width,
                           const uint8_t height, block_t ***frame_buffer, rclcpp::Logger logger) : width(width), height(height), logger(logger) {
        this->frame_buffer = new block_t *[this->height];
        for(size_t i = 0; i < this->height; ++i) {
            this->frame_buffer[i] = new block_t[this->width];
        }
        *frame_buffer = this->frame_buffer;
        RCLCPP_INFO(this->logger, "Init of framebuffer done");

        // create a udp socket
        this->network_socket = socket(AF_INET, SOCK_DGRAM, 0);

        // address for socket
        this->server_address.sin_family = AF_INET;
        this->server_address.sin_port = htons(port);
        inet_aton(host_name.data(), &server_address.sin_addr);

        // bind address to socket
        //TODO: make some error handling, but screw that, I am a consultant
        bind(network_socket, (struct sockaddr *) &server_address, sizeof(server_address));

        RCLCPP_INFO(this->logger, "Init of visualizer done");
    }

    Visualizer::~Visualizer() {
        for(size_t i = 0; i < this->height; ++i) {
            delete[] this->frame_buffer[i];
        }
        delete[] this->frame_buffer;
    }

    void Visualizer::re_draw() {
        RCLCPP_INFO(this->logger, "Redrawing");
        for(size_t i = 0; i < this->height; ++i) {
            sendto(this->network_socket, this->frame_buffer[i], sizeof(block_t) * this->width, 0, (const struct sockaddr *) &server_address,
                   sizeof(server_address));
            RCLCPP_INFO(this->logger, "Send row: %zu", i);
        }
    }
}