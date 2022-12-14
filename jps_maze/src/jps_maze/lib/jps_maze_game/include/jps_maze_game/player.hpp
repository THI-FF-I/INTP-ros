#ifndef INTP_ROS_SERVER_NODE_PLAYER_HPP
#define INTP_ROS_SERVER_NODE_PLAYER_HPP

#include <vector>
#include <inttypes.h>
#include <string>
#include <stdexcept>

#include "jps_maze_msgs/msg/team.hpp"
#include "jps_maze_msgs/msg/player.hpp"
#include "jps_maze_msgs/srv/move_player.hpp"

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
        bool turn = true;

    public:
        Player(const player_id_t player_id, const team_t team_id, const std::string &player_name);

        Player() {}

        constexpr void set_x(const coord_t new_x)
        {
            x = new_x;
        }
        constexpr void set_y(const coord_t new_y)
        {
            y = new_y;
        }
        constexpr void set_has_flag(bool new_flag)
        {
            has_flag = new_flag;
        }
        constexpr void set_turn(bool new_turn)
        {
            turn = new_turn;
        }
        constexpr void set_color(color_t new_color)
        {
            color = new_color;
        }

        constexpr player_id_t get_player_id() const
        {
            return this->player_id;
        }

        constexpr coord_t get_x() const
        {
            return x;
        }
        constexpr coord_t get_y() const
        {
            return y;
        }
        constexpr team_t get_team() const
        {
            return team;
        }
        constexpr color_t get_color() const
        {
            return this->color;
        }
        std::string get_player_name() const
        {
            return this->player_name;
        }
        constexpr bool get_has_flag() const
        {
            return this->has_flag;
        }
        constexpr bool get_turn() const
        {
            return turn;
        }

        ~Player() = default;
        Player(const Player &other) = default;
    };
}

#endif // INTP_ROS_SERVER_NODE_PLAYER_HPP