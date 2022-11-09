#ifndef INTP_ROS_SERVER_NODE_BOARD_HPP
#define INTP_ROS_SERVER_NODE_BOARD_HPP

#include <vector>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <stdexcept>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "jps_maze_msgs/msg/position.hpp"

namespace jps_maze_game
{
    class Board;
}

#include "jps_maze_game/player.hpp"
#include "jps_maze_game/types.hpp"

namespace jps_maze_game
{
    class Portal
    {
    private:
        coord_t x, y;

    public:
        Portal(const coord_t px, const coord_t py)
        {
            x = px;
            y = py;
        }

        constexpr coord_t get_x() const
        {
            return x;
        }
        constexpr coord_t get_y() const
        {
            return y;
        }
        std::vector<coord_t> get_position() const
        {
            return {x, y};
        }
    };

    class Board
    {
    private:
        std::vector<std::vector<game_block_t>> board;
        coord_t width, height;
        game_flag_state_t flag_a = GAME_FLAG_STATE_IN_BASE;
        game_flag_state_t flag_b = GAME_FLAG_STATE_IN_BASE;
        std::vector<Portal> portals;
        rclcpp::Logger logger;

    public:
        Board(const std::string_view filename, rclcpp::Logger logger);

        Board(rclcpp::Logger logger) : logger(logger){};

        bool load_board_from_file(const std::string_view filename);
        void print_board() const;

        game_block_type_t get_block_state(const coord_t coord_x, const coord_t coord_y) const;
        void map_area(const coord_t coord_x, const coord_t coord_y, team_t team);
        bool player_move(const direction_t dir, Player &player);
        std::vector<std::vector<game_block_type_t>> get_team_board(const team_t team) const;

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