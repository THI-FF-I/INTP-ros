#ifndef INTP_ROS_SERVER_NODE_PLAYER_HPP
#define INTP_ROS_SERVER_NODE_PLAYER_HPP

#include <vector>
#include <inttypes.h>

#include "board.hpp"

#define COORD_X 0
#define COORD_Y 1

namespace jps_maze_server
{
    typedef enum team
    {
        PLAYER_TEAM_A,
        PLAYER_TEAM_B,
    } team_t;

    typedef enum direction
    {
        PLAYER_DIR_UP,
        PLAYER_DIR_DOWN,
        PLAYER_DIR_LEFT,
        PLAYER_DIR_RIGHT,
    } direction_t;

    class Player
    {
    private:
        uint64_t playerID;
        team_t team;
        uint32_t color;
        bool has_flag = false;
        uint8_t location[2];

    public:
        Player(uint64_t playerID_p, team_t team_p, uint32_t color_p);

        bool move(direction_t dir, Board &board);

        ~Player()
        {
        }
    };
}

#endif // INTP_ROS_SERVER_NODE_PLAYER_HPP