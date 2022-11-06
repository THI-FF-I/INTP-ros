#ifndef INTP_ROS_SERVER_NODE_BOARD_HPP
#define INTP_ROS_SERVER_NODE_BOARD_HPP

#include <vector>
#include <inttypes.h>

#include "jps_maze_msgs/msg/position.hpp"

#include "jps_maze_game/player.hpp"

namespace jps_maze_game
{

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

    class Board
    {
    private:
        std::vector<std::vector<game_block_t>> board;
        coord_t width, height;

    public:
        Board(const coord_t width, const coord_t height);

        Board(){};

        bool load_board_from_file(const std::string_view filename);

        game_block_type_t get_block_state(const coord_t coord_x, const coord_t coord_y) const;
        void map_area(const coord_t coord_x, const coord_t coord_y, team_t team);

        ~Board()
        {
        }

        constexpr coord_t get_width() const
        {
            return this->width;
        }

        constexpr coord_t get_height() const
        {
            return this->height;
        }
    };
}

#endif // INTP_ROS_SERVER_NODE_BOARD_HPP