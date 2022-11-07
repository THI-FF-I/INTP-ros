#ifndef INTP_ROS_SERVER_NODE_GAME_HPP
#define INTP_ROS_SERVER_NODE_GAME_HPP

#include <map>
#include <random>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "jps_maze_msgs/msg/player.hpp"
#include "jps_maze_msgs/msg/team.hpp"
#include "jps_maze_msgs/msg/status.hpp"

#include "jps_maze_game/board.hpp"
#include "jps_maze_game/player.hpp"
#include "jps_maze_game/types.hpp"

namespace jps_maze_game
{
    class Game
    {
    private:
        Board board;
        std::map<jps_maze_msgs::msg::Player::_id_type, Player> players;
        rclcpp::Logger logger;
        std::mt19937_64 id_gen;
        uint16_t round_cnt;
        uint16_t update_cycle;

    public:
        Game(const coord_t width, const coord_t height, const std::string_view board_path, const uint16_t Pupdate_cycle_ms, const rclcpp::Logger logger);

        Game(const rclcpp::Logger logger) : logger(logger){};

        ~Game() = default;

        Player &add_player(const std::string &name, const team_t team);

        bool move_player(const player_id_t player_id, const direction_t direction);

        bool next_round_ready() const; // Return if all players have moved

        void next_round();

        void get_status(const team_t team, jps_maze_msgs::msg::Status &status) const;

        constexpr coord_t get_width() const
        {
            return this->board.get_width();
        }

        constexpr coord_t get_height() const
        {
            return this->board.get_height();
        }

        constexpr uint16_t get_round_cnt() const
        {
            return this->round_cnt;
        }
    };
}

#endif // INTP_ROS_SERVER_NODE_GAME_HPP