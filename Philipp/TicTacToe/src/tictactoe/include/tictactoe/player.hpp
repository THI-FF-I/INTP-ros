#ifndef PLAYER_H
#define PLAYER_H

#include "tictactoe_interfaces/msg/board.hpp"
#include "tictactoe_interfaces/msg/state.hpp"

#include "tictactoe_interfaces/srv/add_player.hpp"
#include "tictactoe_interfaces/srv/turn.hpp"

#include <iostream>
#include <thread>

typedef enum field_type : uint8_t
{
    EMPTY = tictactoe_interfaces::msg::Field::EMPTY,
    PLAYER_A = tictactoe_interfaces::msg::Field::PLAYER_A,
    PLAYER_B = tictactoe_interfaces::msg::Field::PLAYER_B,
} field_t;

typedef enum gamestate_type : uint8_t
{
    WAITING_FOR_PLAYERS = tictactoe_interfaces::msg::State::WAITING_FOR_PLAYERS,
    RUNNING = tictactoe_interfaces::msg::State::RUNNING,
    WIN_PLAYER_A = tictactoe_interfaces::msg::State::WIN_PLAYER_A,
    WIN_PLAYER_B = tictactoe_interfaces::msg::State::WIN_PLAYER_B,
    DRAW = tictactoe_interfaces::msg::State::DRAW,
} gamestate_t;

#endif