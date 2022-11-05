#ifndef INTP_ROS_SERVER_NODE_GAME_HPP
#define INTP_ROS_SERVER_NODE_GAME_HPP

#include <vector>

#include "jps_maze_server/board.hpp"
#include "jps_maze_server/player.hpp"

namespace jps_maze_server
{
    class Game
    {
    private:
        Board board;
        std::vector<Player> players_team_a;
        std::vector<Player> players_team_b;

    public:
        Game(uint8_t dim_x, uint8_t dim_y) : board(dim_x, dim_y)
        {
        }

        ~Game()
        {
        }
    };
}

#endif //INTP_ROS_SERVER_NODE_GAME_HPP