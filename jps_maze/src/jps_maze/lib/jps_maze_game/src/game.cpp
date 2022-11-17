#include "jps_maze_game/game.hpp"

namespace jps_maze_game
{
    Game::Game(const std::string_view board_path, uint8_t Pplayer_count_per_team, rclcpp::Logger logger) : board(board_path, logger.get_child("board")), logger(logger), round_cnt(0), player_count_per_team(Pplayer_count_per_team)
    {
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
            RCLCPP_WARN(this->logger, "[Game::add_player] Player couldn't be created - Team A already full");
            throw std::runtime_error("[Game::add_player] Player couldn't be created - Team A already full");
        }
        else if (team == PLAYER_TEAM_B && player_count_team_b >= player_count_per_team)
        {
            RCLCPP_WARN(this->logger, "[Game::add_player] Player couldn't be created - Team B already full");
            throw std::runtime_error("[Game::add_player] Player couldn't be created - Team B already full");
        }

        player_id_t player_id = this->id_gen();

        Player new_player(player_id, team, name);
        this->color_gen = std::mt19937(rd());
        new_player.set_color(color_gen());
        new_player.set_x(0); // Default value
        new_player.set_y(0); // Default value

        auto base = board.get_base(team);

        for (const auto &m : base)
        {
            bool success = true;

            for (const auto &p : players)
            {
                if (p.second.get_x() == m.first && p.second.get_y() == m.second)
                {
                    success = false;
                }
            }

            if (success == true)
            {
                new_player.set_x(m.first);
                new_player.set_y(m.second);
                break;
            }
        }

        board.map_area(new_player.get_x(), new_player.get_y(), new_player.get_team());

        this->players.emplace(std::make_pair(player_id, new_player));
        RCLCPP_INFO(this->logger, "Created new player: \"%s\" with id: %lu in Team %c at x: %d y: %d", name.c_str(), player_id, (team == PLAYER_TEAM_A) ? 'A' : 'B', new_player.get_x(), new_player.get_y());
        return this->players.at(player_id);
    }

    bool Game::ready()
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
            game_state = GAME_STATE_RUNNING;
            RCLCPP_INFO(this->logger, "Okay, let's go!\nGame is running...");
            return true;
        }
        else
        {
            game_state = GAME_STATE_WAITING_FOR_PLAYERS;
            return false;
        }
    }

    bool Game::move_player(const player_id_t player_id, const direction_t direction)
    {
        RCLCPP_DEBUG(logger, "Game::move_player: player_id: %lu, dir: %d", player_id, direction);

        try
        {
            if (players.at(player_id).get_turn() == false)
            {
                RCLCPP_DEBUG(logger, "move player cancelled: not your turn");
                return false;
            }

            if (board.player_move(direction, players.at(player_id)) == true)
            {
                players.at(player_id).set_turn(false);
                RCLCPP_DEBUG(logger, "move player successfull");
                return true;
            }
            else
            {
                RCLCPP_DEBUG(logger, "move player cancelled: move not allowed");
                return false;
            }
        }
        catch (std::out_of_range &m)
        {
            RCLCPP_DEBUG(logger, "exception thrown: %s", m.what());
            return false;
        }
        catch (...)
        {
            RCLCPP_DEBUG(logger, "unknown exception");
            return false;
        }
    }

    std::vector<Player> Game::get_players_of_team(team_t team) const
    {
        std::vector<Player> res;

        for (const auto &m : players)
        {
            if (m.second.get_team() == team)
            {
                res.push_back(m.second);
            }
        }

        return res;
    }

    bool Game::next_round_ready() // Returns true if next round is ready to start and false if we need to keep waiting
    {
        for (const auto &m : players)
        {
            if (m.second.get_turn() == true)
            {
                return false;
            }
        }

        // Check if a team won
        for (const auto &m : players)
        {
            if (m.second.get_has_flag() == true)
            {
                if (m.second.get_team() == PLAYER_TEAM_A && board.get_block_state(m.second.get_x(), m.second.get_y()) == GAME_BLOCK_BASE_A)
                {
                    game_state = GAME_STATE_WIN_TEAM_A;
                    break;
                }
                else if (m.second.get_team() == PLAYER_TEAM_B && board.get_block_state(m.second.get_x(), m.second.get_y()) == GAME_BLOCK_BASE_B)
                {
                    game_state = GAME_STATE_WIN_TEAM_B;
                    break;
                }
            }
        }

        RCLCPP_INFO(this->logger, "Ready for next round");
        return true;
    }

    void Game::next_round()
    {
        for (auto &m : players) // Reset all turn flags
        {
            m.second.set_turn(true);
        }

        round_cnt++;

        if (game_state != GAME_STATE_WIN_TEAM_A && game_state != GAME_STATE_WIN_TEAM_B)
            game_state = GAME_STATE_RUNNING;
    }

    std::vector<std::vector<game_block_type_t>> Game::get_team_board(const team_t team) const
    {
        return this->board.get_team_board(team);
    }

    std::vector<std::vector<game_block_type_t>> Game::get_board() const
    {
        return board.get_board();
    }
}