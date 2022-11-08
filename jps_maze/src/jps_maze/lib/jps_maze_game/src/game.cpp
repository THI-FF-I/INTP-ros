#include "jps_maze_game/game.hpp"

namespace jps_maze_game
{
    Game::Game(const std::string_view board_path, rclcpp::Logger logger) : board(board_path, logger.get_child("board")), logger(logger), round_cnt(0)
    {
        std::random_device rd;
        this->id_gen = std::mt19937_64(rd());
        RCLCPP_INFO(this->logger, "Init of Game done");
    }

    Player &Game::add_player(const std::string &name, const team_t team)
    {
        player_id_t player_id = this->id_gen();
        this->players.emplace(std::make_pair(player_id, Player{player_id, team, name}));
        RCLCPP_INFO(this->logger, "Created new player: \"%s\" with id: %ld", name.c_str(), player_id);
        return this->players.at(player_id);
    }

    bool Game::ready() const {
        return false;
    }

    bool Game::move_player(const player_id_t player_id, const direction_t direction)
    {
        if(board.player_move(direction, players.at(player_id)) == true)
        {
            players.at(player_id).set_turn(false);
            return true;
        }
        else
        {
            return false;
        }
    }

    bool Game::next_round_ready() const // Returns true if next round is ready to start and false if we need to keep waiting
    {
        for(const auto &m: players)
        {
            if(m.second.get_turn() == true) return false;
        }

        // TODO Check for win

        return true;
    }

    void Game::next_round()
    {
        for(auto &m: players) // Reset all turn flags
        {
            m.second.set_turn(false);
        }

        // TODO Probably more to do here
    }

    void Game::get_status(const team_t team, jps_maze_msgs::msg::Status &status) const
    {
        // TODO What should be done here???
    }
}