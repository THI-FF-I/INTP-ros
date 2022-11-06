#include "jps_maze_game/board.hpp"

namespace jps_maze_game
{
    Board::Board(const coord_t width, const coord_t height) : width(width), height(height)
    {
        board.clear();
        board.reserve(height);

        for (int y = 0; y < height; y++)
        {
            std::vector<game_block_t> new_y;
            new_y.reserve(width);

            for (int x = 0; x < width; x++)
            {
                game_block_t tmp(GAME_BLOCK_EMPTY);
                new_y.push_back(tmp);
            }

            board.push_back(new_y);
        }
    }

    bool Board::load_board_from_file(const std::string_view filename)
    {
        // Load file and store contents and dimensions in board

        // Remove compiler warnings ;)
        return !filename.empty();
    }

    game_block_type_t Board::get_block_state(const coord_t coord_x, const coord_t coord_y) const
    {
        if (coord_x >= width || coord_y >= height)
            return GAME_BLOCK_OUT_OF_BOARD;

        return board.at(coord_x).at(coord_y).game_block_type;
    }

    void Board::map_area(const coord_t coord_x, const coord_t coord_y, team_t team)
    {
        const uint8_t size = 3; // maps a 3x3 square at the player's position

        for (int h = coord_y - (size - 1) / 2; h <= coord_y + (size - 1) / 2; h++)
        {
            for (int w = coord_x - (size - 1) / 2; w <= coord_x + (size - 1) / 2; w++)
            {
                if (h >= 0 && w >= 0 && w < width && h < height)
                {
                    if (team == PLAYER_TEAM_A)
                        board.at(h).at(w).mapped_team_a = true;
                    else if (team == PLAYER_TEAM_B)
                        board.at(h).at(w).mapped_team_b = true;
                }
            }
        }
    }

    bool Board::player_move(const direction_t dir, Player &player)
    {
        coord_t new_x = player.get_x();
        coord_t new_y = player.get_y();

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
            throw "[Board::player_move] Unknown direction given";
        }

        game_block_type_t new_block = get_block_state(new_x, new_y);

        switch (new_block)
        {
        case GAME_BLOCK_OUT_OF_BOARD:
            return false;

        case GAME_BLOCK_EMPTY:
            player.set_x(new_x);
            player.set_y(new_y);
            return true;

        case GAME_BLOCK_WALL:
            return false;

        case GAME_BLOCK_PORTAL:
            // Teleport player to random portal
            return true;

        case GAME_BLOCK_FLAG_A:
            player.set_x(new_x);
            player.set_y(new_y);
            if (player.get_team() == PLAYER_TEAM_A)
                player.set_has_flag(true);
            return true;

        case GAME_BLOCK_FLAG_B:
            player.set_x(new_x);
            player.set_y(new_y);
            if (player.get_team() == PLAYER_TEAM_B)
                player.set_has_flag(true);
            return true;

        case GAME_BLOCK_BASE_A:
            player.set_x(new_x);
            player.set_y(new_y);
            return true;

        case GAME_BLOCK_BASE_B:
            player.set_x(new_x);
            player.set_y(new_y);
            return true;

        default:
            throw "[Board::player_move] Unknown block type";
        }
    }

    std::vector<std::vector<game_block_type_t>> Board::get_team_board(team_t team)
    {
        std::vector<std::vector<game_block_type_t>> res;
        res.reserve(height);

        for (coord_t h = 0; h < height; h++)
        {
            std::vector<game_block_type_t> tmp;
            tmp.reserve(width);

            for (coord_t w = 0; w < width; w++)
            {
                if (team == PLAYER_TEAM_A)
                {
                    if (board.at(h).at(w).mapped_team_a == false)
                    {
                        tmp.push_back(GAME_BLOCK_EMPTY);
                    }
                    else
                    {
                        tmp.push_back(board.at(h).at(w).game_block_type);
                    }
                }
                else if (team == PLAYER_TEAM_B)
                {
                    if (board.at(h).at(w).mapped_team_b == false)
                    {
                        tmp.push_back(GAME_BLOCK_EMPTY);
                    }
                    else
                    {
                        tmp.push_back(board.at(h).at(w).game_block_type);
                    }
                }
            }

            res.push_back(tmp);
        }

        return res;
    }
}