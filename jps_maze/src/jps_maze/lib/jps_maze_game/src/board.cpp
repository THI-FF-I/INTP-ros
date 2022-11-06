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
        // work in progress...
    }
}