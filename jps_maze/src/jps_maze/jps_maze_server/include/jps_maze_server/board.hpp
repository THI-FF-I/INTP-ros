#ifndef INTP_ROS_SERVER_NODE_BOARD_HPP
#define INTP_ROS_SERVER_NODE_BOARD_HPP

#include <vector>
#include <inttypes.h>

#include "jps_maze_msgs/msg/position.hpp"

namespace jps_maze_server {

    using coord_t = jps_maze_msgs::msg::Position::_x_type;
    static_assert(sizeof(jps_maze_msgs::msg::Position::_x_type) == sizeof(jps_maze_msgs::msg::Position::_y_type), "Position types for x and y differ!");
    typedef enum game_block : int8_t {
        GAME_BLOCK_OUT_OF_BOARD = -1,
        GAME_BLOCK_EMPTY,
        GAME_BLOCK_WALL,
        GAME_BLOCK_PORTAL, // Player gets teleported to another random portal
        GAME_BLOCK_FLAG_A,
        GAME_BLOCK_FLAG_B,
        GAME_BLOCK_BASE_A,
        GAME_BLOCK_BASE_B,
    } game_block_t;

    class Board
    {
    private:
        std::vector<std::vector<game_block_t>> board;
        uint8_t width, height;

    public:
        Board(const coord_t width, const coord_t height);

        Board() {};

        game_block_t get_block_state(const coord_t coord_x, const coord_t coord_y) const;

        ~Board()
        {
        }
    };
}

#endif // INTP_ROS_SERVER_NODE_BOARD_HPP