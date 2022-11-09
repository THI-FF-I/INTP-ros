//
// Created by Johan on 06.11.2022.
//

#ifndef INTP_ROS_TYPES_HPP
#define INTP_ROS_TYPES_HPP

#include <inttypes.h>

#include "jps_maze_msgs/msg/position.hpp"
#include "jps_maze_msgs/msg/block.hpp"

namespace jps_maze_game
{
    // GAME
    typedef enum game_flag_state : uint8_t
    {
        GAME_FLAG_STATE_IN_BASE,
        GAME_FLAG_STATE_BY_PLAYER,
    } game_flag_state_t;

    typedef enum game_state_enum : uint8_t
    {
        GAME_STATE_WAITING_FOR_PLAYER,
        GAME_STATE_RUNNING,
        GAME_STATE_WIN_TEAM_A,
        GAME_STATE_WIN_TEAM_b,
    } game_state_t;

    // BOARD
    using coord_t = jps_maze_msgs::msg::Position::_x_type;
    static_assert(sizeof(jps_maze_msgs::msg::Position::_x_type) == sizeof(jps_maze_msgs::msg::Position::_y_type), "Position types for x and y differ!");

    typedef enum game_block_type : jps_maze_msgs::msg::Block::_block_type_type
    {
        GAME_BLOCK_OUT_OF_BOARD = jps_maze_msgs::msg::Block::OUT_OF_BOARD,
        GAME_BLOCK_EMPTY = jps_maze_msgs::msg::Block::EMPTY,
        GAME_BLOCK_WALL = jps_maze_msgs::msg::Block::WALL,
        GAME_BLOCK_PORTAL = jps_maze_msgs::msg::Block::PORTAL, // Player gets teleported to another random portal
        GAME_BLOCK_FLAG_A = jps_maze_msgs::msg::Block::FLAG_A,
        GAME_BLOCK_FLAG_B = jps_maze_msgs::msg::Block::FLAG_B,
        GAME_BLOCK_BASE_A = jps_maze_msgs::msg::Block::BASE_A,
        GAME_BLOCK_BASE_B = jps_maze_msgs::msg::Block::BASE_B,
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


    // PLAYER
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
}

#endif //INTP_ROS_TYPES_HPP
