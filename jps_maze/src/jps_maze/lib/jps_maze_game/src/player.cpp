#include "jps_maze_game/player.hpp"

namespace jps_maze_game
{

    Player::Player(const player_id_t player_id, const team_t team, const std::string &player_name) : player_id(player_id), team(team), player_name(player_name)
    {
    }

    bool Player::move(const direction_t dir, Board &board)
    {
        coord_t new_x = this->x;
        coord_t new_y = this->y;

        switch (dir)
        {
        case PLAYER_DIR_UP:
            if (new_x <= 0)
                return false;
            new_x -= 1;
            new_y = new_y;
            break;

        case PLAYER_DIR_DOWN:
            new_x += 1;
            new_y = new_y;
            break;

        case PLAYER_DIR_LEFT:
            if (new_y <= 0)
                return false;
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

        board.map_area(new_x, new_y);
        game_block_type_t new_block = board.get_block_state(new_x, new_y);

        switch (new_block)
        {
        case GAME_BLOCK_OUT_OF_BOARD:
            return false;

        case GAME_BLOCK_EMPTY:
            this->x = new_x;
            this->y = new_y;
            return true;

        case GAME_BLOCK_WALL:
            return false;

        case GAME_BLOCK_PORTAL:
            // Teleport player to random portal
            return true;

        case GAME_BLOCK_FLAG_A:
            this->y = new_x;
            this->x = new_y;
            if (team == PLAYER_TEAM_A)
                has_flag = true;
            return true;

        case GAME_BLOCK_FLAG_B:
            this->y = new_x;
            this->x = new_y;
            if (team == PLAYER_TEAM_B)
                has_flag = true;
            return true;

        case GAME_BLOCK_BASE_A:
            this->x = new_x;
            this->y = new_y;
            return true;

        case GAME_BLOCK_BASE_B:
            this->x = new_x;
            this->y = new_y;
            return true;

        default:
            throw "[Player::move] Unknown block type";
        }
    }

}