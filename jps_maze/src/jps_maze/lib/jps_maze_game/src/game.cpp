#include "jps_maze_game/game.hpp"

namespace jps_maze_game
{
    Game::Game(const coord_t width, const coord_t height, const std::string_view board_path, const uint16_t Pupdate_cycle_ms, const rclcpp::Logger logger) : board(width, height), logger(logger), round_cnt(0), update_cycle(Pupdate_cycle_ms)
    {
        this->board.load_board_from_file(board_path);
        std::random_device rd;
        this->id_gen = std::mt19937_64(rd());
    }

    Player &Game::add_player(const std::string &name, const team_t team)
    {
        player_id_t player_id = this->id_gen();
        this->players.emplace(std::make_pair(player_id, Player{player_id, team, name}));
        RCLCPP_INFO(this->logger, "Created new player: \"%s\" with id: %ld", name.c_str(), player_id);
        return this->players.at(player_id);
    }

    bool Game::move_player(const player_id_t player_id, const direction_t direction)
    {
    }

    bool Game::next_round_ready() const
    {
        return false;
    }

    void Game::next_round()
    {
    }

    void Game::get_status(const team_t team, jps_maze_msgs::msg::Status &status) const
    {
    }
}