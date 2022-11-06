#ifndef INTP_ROS_SERVER_NODE_PLAYER_HPP
#define INTP_ROS_SERVER_NODE_PLAYER_HPP

#include <vector>
#include <inttypes.h>
#include <string>

#include "jps_maze_msgs/msg/team.hpp"
#include "jps_maze_msgs/msg/player.hpp"
#include "jps_maze_msgs/srv/move_player.hpp"

#include "jps_maze_game/board.hpp"

namespace jps_maze_game
{
    using player_id_t = jps_maze_msgs::msg::Player::_id_type;
    using color_t = jps_maze_msgs::msg::Player::_color_type;

    typedef enum team : jps_maze_msgs::msg::Team::_team_type
    {
        PLAYER_TEAM_A = jps_maze_msgs::msg::Team::TEAM_A,
        PLAYER_TEAM_B = jps_maze_msgs::msg::Team::TEAM_B,
    } team_t;

    typedef enum direction : jps_maze_msgs::srv::MovePlayer::Request::_dir_type
    {
        PLAYER_DIR_UP = jps_maze_msgs::srv::MovePlayer::Request::UP,
        PLAYER_DIR_DOWN = jps_maze_msgs::srv::MovePlayer::Request::DOWN,
        PLAYER_DIR_LEFT = jps_maze_msgs::srv::MovePlayer::Request::LEFT,
        PLAYER_DIR_RIGHT = jps_maze_msgs::srv::MovePlayer::Request::RIGHT,
    } direction_t;

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
        Player(const player_id_t player_id, const team_t team_id, const std::string& player_name);

        bool move(const direction_t dir, Board &board);

        ~Player() = default;
        Player(const Player& other) = default;
    };
}

#endif // INTP_ROS_SERVER_NODE_PLAYER_HPP