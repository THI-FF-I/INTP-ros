#include "jps_maze_visualizer/visualizer.hpp"

namespace jps_maze_visualizer {

    Visualizer::Visualizer(const std::string_view host_name, const std::string_view port, const block_t width,
                           const block_t height, block_t ***frame_buffer, rclcpp::Logger logger) : width(width), height(height), logger(logger) {
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

        if(getaddrinfo(host_name.data(), port.data(), &hints, &result)!= 0) {
            RCLCPP_FATAL(this->logger, "Could resolve hostname");
            throw "Could not resolve hostname error";
        }

        for (rp = result; rp != NULL; rp = rp->ai_next) {
            if((this->network_socket = socket(rp->ai_family, rp->ai_socktype,
                         rp->ai_protocol)) == 1)
                continue;


            if (bind(this->network_socket, rp->ai_addr, rp->ai_addrlen) != -1)
                break;                  /* Success */

            close(this->network_socket);
        }

        freeaddrinfo(result);

        if (rp == NULL) {
            RCLCPP_FATAL(this->logger, "Could not connect to visualisation server");
            throw "Could not connect to visualisation server";
        }
        this->server_address = *(rp->ai_addr);
        /*
        // create a udp socket
        this->network_socket = socket(AF_INET, SOCK_DGRAM, 0);

        // address for socket
        this->server_address.sin_family = AF_INET;
        this->server_address.sin_port = htons(port);
        //inet_aton(host_name.data(), &server_address.sin_addr);

        // bind address to socket
        //TODO: make some error handling, but screw that, I am a consultant
        bind(network_socket, (struct sockaddr *) &server_address, sizeof(server_address));
        */

        RCLCPP_INFO(this->logger, "Sending dimensions");
        block_t dim[2] = {this->width, this->height};

        sendto(this->network_socket, dim, sizeof(dim), 0, (const struct sockaddr *) &server_address,
               sizeof(server_address));
        RCLCPP_INFO(this->logger, "Init of visualizer done");
    }

    Visualizer::~Visualizer() {
        RCLCPP_INFO(this->logger, "Closing network socket");
        close(this->network_socket);
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