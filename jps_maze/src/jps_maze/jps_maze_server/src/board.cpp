#include "jps_maze_server/board.hpp"

using namespace jps_maze_server;

Board::Board(uint8_t dim_x, uint8_t dim_y)
{
    board.clear();
    dimensions[COORD_X] = dim_x;
    dimensions[COORD_Y] = dim_y;

    for (int y = 0; y < dimensions[COORD_Y]; y++)
    {
        std::vector<game_block_t> new_y;

        for (int x = 0; x < dimensions[COORD_X]; x++)
        {
            new_y.push_back(GAME_BLOCK_EMPTY);
        }

        board.push_back(new_y);
    }
}

game_block_t Board::get_block_state(int16_t coord_x, int16_t coord_y) const
{
    if(coord_x < 0 || coord_y < 0 || coord_x >= dimensions[COORD_X] || coord_y >= dimensions[COORD_Y]) return GAME_BLOCK_OUT_OF_BOARD;

    return board.at(coord_x).at(coord_y);
}