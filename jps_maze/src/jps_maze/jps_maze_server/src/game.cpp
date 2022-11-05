#include "jps_maze_server/game.hpp"

namespace jps_maze_server {
    Game::Game(const coord_t width, const coord_t height, const rclcpp::Logger logger) : board(width, height), logger(logger), round_cnt(0)
    {
        std::random_device rd;
        this->id_gen = std::mt19937_64(rd());
    }

    Player& Game::add_player(const std::string &name, const jps_maze_msgs::msg::Team::_team_type team) {
        player_id_t player_id = this->id_gen();
        this->players.emplace(std::make_pair(player_id, Player{player_id, static_cast<team_t>(team), name}));
        RCLCPP_INFO(this->logger, "Created new player: \"%s\" with id: %ld", name.c_str(), player_id);
        return this->players.at(player_id);
    }

    bool Game::move_player(const jps_maze_server::player_id_t player_id, const jps_maze_server::direction_t direction) {

    }

    bool Game::next_round_ready() const{
        return false;
    }

    void Game::next_round() {

    }

    void Game::get_status(const jps_maze_server::team_t team, jps_maze_msgs::msg::Status &status) const {

    }
}