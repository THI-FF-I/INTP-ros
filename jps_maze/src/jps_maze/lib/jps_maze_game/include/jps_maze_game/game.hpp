#ifndef INTP_ROS_SERVER_NODE_GAME_HPP
#define INTP_ROS_SERVER_NODE_GAME_HPP

#include <map>
#include <random>
#include <stdexcept>

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
        uint8_t player_count_per_team;
        game_state_t game_state = GAME_STATE_WAITING_FOR_PLAYERS;

    public:
        Game(const std::string_view board_path, uint8_t Pplayer_count_per_team, const rclcpp::Logger logger);

        Game(const rclcpp::Logger logger) : board(logger), logger(logger){};

        ~Game() = default;

        Player &add_player(const std::string &name, team_t team);

        bool ready();

        bool move_player(const player_id_t player_id, const direction_t direction);

        // TODO return all players of a team
        std::vector<Player> get_players_of_team(team_t team) const;

        bool next_round_ready(); // Return if all players have moved

        void next_round();

        std::vector<std::vector<game_block_type_t>> get_team_board(const team_t team) const;

        constexpr game_state_t get_game_state() const
        {
            return game_state;
        }

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