//
// Created by Johan on 06.11.2022.
//

#ifndef INTP_ROS_TYPES_HPP
#define INTP_ROS_TYPES_HPP

namespace jps_maze_game {

    using player_id_t = jps_maze_msgs::msg::Player::_id_type;
    using color_t = jps_maze_msgs::msg::Player::_color_type;
    using coord_t = jps_maze_msgs::msg::Position::_x_type;
    static_assert(sizeof(jps_maze_msgs::msg::Position::_x_type) == sizeof(jps_maze_msgs::msg::Position::_y_type), "Position types for x and y differ!");

    typedef enum game_block_type : int8_t
    {
        GAME_BLOCK_OUT_OF_BOARD = -1,
        GAME_BLOCK_EMPTY = 0,
        GAME_BLOCK_WALL,
        GAME_BLOCK_PORTAL, // Player gets teleported to another random portal
        GAME_BLOCK_FLAG_A,
        GAME_BLOCK_FLAG_B,
        GAME_BLOCK_BASE_A,
        GAME_BLOCK_BASE_B,
    } game_block_type_t;

    typedef struct game_block
    {
        game_block_type_t game_block_type = GAME_BLOCK_EMPTY;
        bool mapped_team_a = false; // true means the block was mapped / seen by a team A member
        bool mapped_team_b = false; // true means the block was mapped / seen by a team B member

        game_block(game_block_type_t pGame_block_type) : game_block_type(pGame_block_type)
        {
        }
    } game_block_t;

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
}

#endif //INTP_ROS_TYPES_HPP
