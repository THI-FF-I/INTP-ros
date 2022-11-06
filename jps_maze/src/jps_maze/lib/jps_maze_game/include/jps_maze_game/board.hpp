#ifndef INTP_ROS_SERVER_NODE_BOARD_HPP
#define INTP_ROS_SERVER_NODE_BOARD_HPP

#include <vector>
#include <inttypes.h>

#include "jps_maze_msgs/msg/position.hpp"

#include "jps_maze_game/player.hpp"
#include "jps_maze_game/types.hpp"

namespace jps_maze_game
{
    class Board
    {
    private:
        std::vector<std::vector<game_block_t>> board;
        coord_t width, height;

    public:
        Board(const coord_t width, const coord_t height);

        Board(){};

        bool load_board_from_file(const std::string_view filename);

        game_block_type_t get_block_state(const coord_t coord_x, const coord_t coord_y) const;
        void map_area(const coord_t coord_x, const coord_t coord_y, team_t team);
        bool player_move(const direction_t dir, Player &player);
        std::vector<std::vector<game_block_type_t>> get_team_board(team_t team);

        ~Board()
        {
        }

        constexpr coord_t get_width() const
        {
            return this->width;
        }

        constexpr coord_t get_height() const
        {
            return this->height;
        }
    };
}

#endif // INTP_ROS_SERVER_NODE_BOARD_HPP