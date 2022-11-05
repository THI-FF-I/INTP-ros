#include "jps_maze_server/board.hpp"

using namespace jps_maze_server;

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
            new_y.push_back(GAME_BLOCK_EMPTY);
        }

        board.push_back(new_y);
    }
}

bool Board::load_board_from_file(const std::string filename)
{
    // Load file and store contents and dimensions in board

    // Remove compiler warnings ;)
    if(filename == "") return false;
    else return true;
}

game_block_t Board::get_block_state(const coord_t coord_x, const coord_t coord_y) const
{
    if(coord_x >= width || coord_y >= height) return GAME_BLOCK_OUT_OF_BOARD;

    return board.at(coord_x).at(coord_y);
}