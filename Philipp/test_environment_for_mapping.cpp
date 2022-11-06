#include <iostream>
#include <inttypes.h>
#include <vector>
#include <string>

typedef uint8_t coord_t;

typedef enum team
{
    PLAYER_TEAM_A,
    PLAYER_TEAM_B,
} team_t;

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

std::vector<std::vector<game_block_t>> board;
coord_t width = 40, height = 20;

void map_area(const coord_t coord_x, const coord_t coord_y, team_t team)
{
    const uint8_t size = 3;

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

void print_board()
{
    for (int i = 0; i < height; i++)
    {
        for (int n = 0; n < width; n++)
        {
            std::string res = "";

            switch (board.at(i).at(n).game_block_type)
            {
            case GAME_BLOCK_EMPTY:
                if(board.at(i).at(n).mapped_team_a == true) res = "-";
                else res = " ";
                break;
            case GAME_BLOCK_WALL:
                if(board.at(i).at(n).mapped_team_a == true) res = "X";
                else res = "*";
                break;
            }

            std::cout << res;
        }
        std::cout << "\n";
    }
    std::cout << std::endl;
}

int main()
{
    board.clear();
    board.reserve(height);

    for (int y = 0; y < height; y++)
    {
        std::vector<game_block_t> new_y;
        new_y.reserve(width);

        for (int x = 0; x < width; x++)
        {
            game_block_t tmp(GAME_BLOCK_WALL);
            new_y.push_back(tmp);
        }

        board.push_back(new_y);
    }

    std::cout << "X: ";
    int x = 0;
    std::cin >> x;
    std::cout << "Y: ";
    int y = 0;
    std::cin >> y;
    map_area(x, y, PLAYER_TEAM_A);

    print_board();

    return EXIT_SUCCESS;
}
