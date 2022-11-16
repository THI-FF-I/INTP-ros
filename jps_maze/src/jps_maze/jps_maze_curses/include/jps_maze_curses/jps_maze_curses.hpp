//
// Created by Johan on 14.11.2022.
//

#ifndef INTP_ROS_CURSES_HPP
#define INTP_ROS_CURSES_HPP

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <limits>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include <curses.h>

namespace jps_maze_curses {

    using block_t = uint32_t;

    enum BLOCK_TYPES : block_t {
        EMPTY = 0,
        WALL = 1,
        PORTAL = 2,
        FLAG_A = 3,
        FLAG_B = 4,
        BASE_A = 5,
        BASE_B = 6,
        NOT_MAPPED = 7,

        WIN_TEXT = 10,
        LOOSE_TEXT = 11,

        PLAYER = 20,
    };

    class CursesWindow {
    public:
        CursesWindow(const char *port, const bool is_server, const bool team_A);
        ~CursesWindow();
        void handle_update();
    private:

        //Networking stuff
        void setup_socket();
        void get_dim();
        void check_dim();
        void get_row();

        const char *port;
        block_t width;
        block_t height;
        const bool is_server;
        const bool team_A;
        size_t cur_player_index = PLAYER;
        block_t *row_buf;
        int network_socket;
        struct sockaddr server_address;
        size_t cur_row;

        //Curses stuff
        void init_curses_app();
        void print_row();
        void print_result(const bool team_A);
    };
}

#endif //INTP_ROS_CURSES_HPP