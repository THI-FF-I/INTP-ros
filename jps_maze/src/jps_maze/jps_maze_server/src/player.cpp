#include "jps_maze_server/player.hpp"

using namespace jps_maze_server;

Player::Player(uint64_t playerID_p, team_t team_p, uint32_t color_p)
{
    playerID = playerID_p;
    team = team_p;
    color = color_p;
}

bool Player::move(direction_t dir, Board &board)
{
    int16_t new_x = location[COORD_X];
    int16_t new_y = location[COORD_Y];

    switch (dir)
    {
    case PLAYER_DIR_UP:
        new_x -= 1;
        new_y = new_y;
        break;

    case PLAYER_DIR_DOWN:
        new_x += 1;
        new_y = new_y;
        break;

    case PLAYER_DIR_LEFT:
        new_x = new_x;
        new_y -= 1;
        break;

    case PLAYER_DIR_RIGHT:
        new_x = new_x;
        new_y += 1;
        break;

    default:
        throw "[Player::move] Unknown direction given";
    }


    game_block_t new_block = board.get_block_state(new_x, new_y);

    switch (new_block)
    {
    case GAME_BLOCK_OUT_OF_BOARD:
        return false;

    case GAME_BLOCK_EMPTY:
        location[COORD_X] = new_x;
        location[COORD_Y] = new_y;
        return true;

    case GAME_BLOCK_WALL:
        return false;

    case GAME_BLOCK_PORTAL:
        // Teleport player to random portal
        return true;

    case GAME_BLOCK_FLAG_A:
        location[COORD_X] = new_x;
        location[COORD_Y] = new_y;
        if (team == PLAYER_TEAM_A)
            has_flag = true;
        return true;

    case GAME_BLOCK_FLAG_B:
        location[COORD_X] = new_x;
        location[COORD_Y] = new_y;
        if (team == PLAYER_TEAM_B)
            has_flag = true;
        return true;

    case GAME_BLOCK_BASE_A:
        location[COORD_X] = new_x;
        location[COORD_Y] = new_y;
        return true;

    case GAME_BLOCK_BASE_B:
        location[COORD_X] = new_x;
        location[COORD_Y] = new_y;
        return true;

    default:
        throw "[Player::move] Unknown block type";
    }
}