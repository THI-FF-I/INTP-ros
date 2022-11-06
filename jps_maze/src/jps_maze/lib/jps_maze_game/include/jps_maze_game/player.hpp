#ifndef INTP_ROS_SERVER_NODE_PLAYER_HPP
#define INTP_ROS_SERVER_NODE_PLAYER_HPP

#include <vector>
#include <inttypes.h>
#include <string>

#include "jps_maze_msgs/msg/team.hpp"
#include "jps_maze_msgs/msg/player.hpp"
#include "jps_maze_msgs/srv/move_player.hpp"

namespace {
    class Player;
}
#include "jps_maze_game/board.hpp"
#include "jps_maze_game/types.hpp"

namespace jps_maze_game
{
    class Player
    {
    private:
        player_id_t player_id;
        team_t team;
        color_t color;
        bool has_flag = false;
        coord_t x, y;
        std::string player_name;

    public:
        Player(const player_id_t player_id, const team_t team_id, const std::string &player_name);

        bool move(const direction_t dir, Board &board);

        ~Player() = default;
        Player(const Player &other) = default;
    };
}

#endif // INTP_ROS_SERVER_NODE_PLAYER_HPP