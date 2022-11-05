#ifndef INTP_ROS_SERVER_NODE_BOARD_HPP
#define INTP_ROS_SERVER_NODE_BOARD_HPP

#include <vector>
#include <inttypes.h>

#define COORD_X 0
#define COORD_Y 1

namespace jps_maze_server
{
    typedef enum game_block
    {
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
        uint8_t dimensions[2];

    public:
        Board(uint8_t dim_x, uint8_t dim_y);

        game_block_t get_block_state(int16_t coord_x, int16_t coord_y) const;

        ~Board()
        {
        }
    };
}

#endif // INTP_ROS_SERVER_NODE_BOARD_HPP