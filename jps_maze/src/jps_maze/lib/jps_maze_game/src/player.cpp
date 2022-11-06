#include "jps_maze_game/player.hpp"

namespace jps_maze_game
{
    Player::Player(const player_id_t player_id, const team_t team, const std::string &player_name) : player_id(player_id), team(team), player_name(player_name)
    {
    }
}