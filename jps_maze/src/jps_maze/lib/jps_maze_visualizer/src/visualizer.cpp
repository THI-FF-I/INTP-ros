#include "jps_maze_visualizer/visualizer.hpp"

namespace jps_maze_visualizer {

    Visualizer::Visualizer(const std::string_view host_name, const std::string_view port, const block_t width,
                           const block_t height, block_t ***frame_buffer, rclcpp::Logger logger) : width(width), height(height), logger(logger), valid(true) {
        this->frame_buffer = new block_t *[this->height];
        for(size_t i = 0; i < this->height; ++i) {
            this->frame_buffer[i] = new block_t[this->width];
        }
        *frame_buffer = this->frame_buffer;
        RCLCPP_INFO(this->logger, "Init of framebuffer done");

        struct addrinfo hints = {};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_protocol = 0;
        hints.ai_flags = 0;
        struct addrinfo *result, *rp;
        RCLCPP_INFO(this->logger, "Trying to connect to %s:%s", host_name.data(), port.data());
        if(getaddrinfo(host_name.data(), port.data(), &hints, &result)!= 0) {
            RCLCPP_FATAL(this->logger, "Could resolve hostname");
            throw std::runtime_error("Could not resolve hostname error");
        }

        for (rp = result; rp != NULL; rp = rp->ai_next) {
            RCLCPP_DEBUG(this->logger, "Trying: %s", rp->ai_addr->sa_data);
            if((this->network_socket = socket(rp->ai_family, rp->ai_socktype,
                         rp->ai_protocol)) != -1) {
                break;
                RCLCPP_DEBUG(this->logger, "Could not create socket");
            }

            close(this->network_socket);
        }

        freeaddrinfo(result);

        if (rp == NULL) {
            RCLCPP_FATAL(this->logger, "Could not connect to visualisation server");
            throw std::runtime_error("Could not connect to visualisation server");
        }
        this->server_address = *(rp->ai_addr);

        RCLCPP_INFO(this->logger, "Sending dimensions width: %d, height: %d", this->width, this->height);
        block_t dim[2] = {this->width, this->height};

        sendto(this->network_socket, dim, sizeof(dim), 0, (const struct sockaddr *) &server_address,
               sizeof(server_address));
        RCLCPP_INFO(this->logger, "Init of visualizer done");
    }

    Visualizer &Visualizer::operator=(jps_maze_visualizer::Visualizer &&rhs) {
        this->logger = rhs.logger;
        this->frame_buffer = rhs.frame_buffer;
        this->height = rhs.height;
        this->width = rhs.width;
        this->server_address = rhs.server_address;
        this->network_socket = rhs.network_socket;
        this->valid = rhs.valid;
        rhs.valid = false;
    }

    Visualizer::~Visualizer() {
        if(valid) {
            RCLCPP_INFO(this->logger, "Closing network socket");
            close(this->network_socket);
            for (size_t i = 0; i < this->height; ++i) {
                delete[] this->frame_buffer[i];
            }
            delete[] this->frame_buffer;
        }
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