#include "jps_maze_game/game.hpp"

namespace jps_maze_game
{
    Game::Game(const std::string_view board_path, uint8_t Pplayer_count_per_team, rclcpp::Logger logger) : board(board_path, logger.get_child("board")), logger(logger), round_cnt(0), player_count_per_team(Pplayer_count_per_team)
    {
        std::random_device rd;
        this->id_gen = std::mt19937_64(rd());
        RCLCPP_INFO(this->logger, "Init of Game done");
    }

    Player &Game::add_player(const std::string &name, team_t team)
    {
        uint8_t player_count_team_a = 0, player_count_team_b = 0;

        for (const auto &m : players)
        {
            if (m.second.get_team() == PLAYER_TEAM_A)
            {
                player_count_team_a++;
            }
            else if (m.second.get_team() == PLAYER_TEAM_B)
            {
                player_count_team_b++;
            }
        }

        if (team == PLAYER_TEAM_A && player_count_team_a >= player_count_per_team)
        {
            if (player_count_team_b < player_count_per_team)
            {
                team = PLAYER_TEAM_B;
                RCLCPP_WARN(this->logger, "Changed player: \"%s\" to team: %c", name.c_str(), team == PLAYER_TEAM_A ? 'A' : 'B');
            }
            else
            {
                throw "[Game::add_player] Both teams already full";
            }
        }
        else if (team == PLAYER_TEAM_B && player_count_team_b >= player_count_per_team)
        {
            if (player_count_team_a < player_count_per_team)
            {
                team = PLAYER_TEAM_A;
                RCLCPP_WARN(this->logger, "Changed player: \"%s\" to team: %c", name.c_str(), team == PLAYER_TEAM_A ? 'A' : 'B');
            }
            else
            {
                throw "[Game::add_player] Both teams already full";
            }
        }

        player_id_t player_id = this->id_gen();
        this->players.emplace(std::make_pair(player_id, Player{player_id, team, name}));
        RCLCPP_INFO(this->logger, "Created new player: \"%s\" with id: %ld", name.c_str(), player_id);
        return this->players.at(player_id);
    }

    bool Game::ready() const
    {
        if (player_count_per_team <= 0)
            return false;

        uint8_t player_count_team_a = 0, player_count_team_b = 0;

        for (const auto &m : players)
        {
            if (m.second.get_team() == PLAYER_TEAM_A)
            {
                player_count_team_a++;
            }
            else if (m.second.get_team() == PLAYER_TEAM_B)
            {
                player_count_team_b++;
            }
        }

        if (player_count_team_a == player_count_team_b && player_count_team_a == player_count_per_team && player_count_team_b == player_count_per_team)
        {
            return true;
        }
        else
            return false;
    }

    bool Game::move_player(const player_id_t player_id, const direction_t direction)
    {
        if(players.at(player_id).get_turn() == false) return false;
        
        if (board.player_move(direction, players.at(player_id)) == true)
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
        for (const auto &m : players)
        {
            if (m.second.get_turn() == true)
                return false;
        }

        // TODO Check for win

        return true;
    }

    void Game::next_round()
    {
        for (auto &m : players) // Reset all turn flags
        {
            m.second.set_turn(false);
        }

        // TODO Probably more to do here
    }

    std::vector<std::vector<game_block_type_t>> Game::get_status(const team_t team) const  {
        return this->board.get_team_board(team);
    }
}