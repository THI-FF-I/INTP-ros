#include "jps_maze_game/board.hpp"

namespace jps_maze_game
{

    Board::Board(const std::string_view filename, rclcpp::Logger logger) : logger(logger) // Creates board from file
    {
        std::random_device rd;
        srand(rd());

        RCLCPP_INFO(this->logger, "Initiating board with file: \"%s\"", filename.data());
        load_board_from_file(filename);
    }

    bool Board::load_board_from_file(const std::string_view filename) // Creates board from file
    {
        std::ifstream file(filename.data());
        std::string line = "";

        if (file.is_open())
        {
            std::string line = "";
            coord_t h = 0, w = 0;
            std::vector<std::vector<game_block_t>> new_board;

            while (std::getline(file, line))
            {
                h++;
                coord_t w_tmp = 0;
                std::vector<game_block_t> new_width;

                for (size_t i = 0; i < line.length(); i += 2)
                {
                    game_block_t tmp(GAME_BLOCK_EMPTY);
                    tmp.mapped_team_a = false;
                    tmp.mapped_team_b = false;
                    w_tmp++;

                    switch (line[i])
                    {
                    case '0':
                        tmp.game_block_type = GAME_BLOCK_EMPTY;
                        break;
                    case '1':
                        tmp.game_block_type = GAME_BLOCK_WALL;
                        break;
                    case '2':
                        tmp.game_block_type = GAME_BLOCK_PORTAL;
                        {
                            Portal port_tmp(i / 2, h);
                            portals.push_back(port_tmp);
                        }
                        break;
                    case '3':
                        tmp.game_block_type = GAME_BLOCK_FLAG_A;
                        pos_flag_a = {i / 2, h};
                        break;
                    case '4':
                        tmp.game_block_type = GAME_BLOCK_FLAG_B;
                        pos_flag_b = {i / 2, h};
                        break;
                    case '5':
                        tmp.game_block_type = GAME_BLOCK_BASE_A;
                        base_a.push_back({i / 2, h});
                        break;
                    case '6':
                        tmp.game_block_type = GAME_BLOCK_BASE_B;
                        base_b.push_back({i / 2, h});
                        break;
                    default:
                        tmp.game_block_type = GAME_BLOCK_EMPTY;
                        break;
                    }

                    new_width.push_back(tmp);
                }

                if (w > 0 && w != w_tmp)
                    throw std::runtime_error("[Board::load_board_from_file] incorrect file syntax");
                w = w_tmp;

                new_board.push_back(new_width);
            }

            file.close();

            board = new_board;
            width = w;
            height = h;
            return true;
        }
        else
        {
            throw std::runtime_error("[Board::load_board_from_file] file could not be read");
            return false;
        }
    }

    void Board::print_board(std::vector<std::vector<game_block_t>> board) const
    {
        std::ofstream file("board.output");

        int width2 = width;
        int height2 = height;

        for (int i = 0; i < height; i++)
        {
            for (int n = 0; n < width; n++)
            {
                std::string res = "";

                switch (board.at(i).at(n).game_block_type)
                {
                case GAME_BLOCK_EMPTY:
                    if (board.at(i).at(n).mapped_team_a == true)
                        res = "-";
                    else
                        res = " ";
                    break;
                case GAME_BLOCK_WALL:
                    if (board.at(i).at(n).mapped_team_a == true)
                        res = "X";
                    else
                        res = "*";
                    break;
                default:
                    res = "o";
                }

                file << res;
            }

            file << "\n";
        }

        file << "Board: " << width2 << "x" << height2 << "\n";

        file.close();
    }

    void Board::print_board(std::vector<std::vector<game_block_type_t>> board) const
    {
        std::ofstream file("board.output");

        int width2 = width;
        int height2 = height;

        for (int i = 0; i < height; i++)
        {
            for (int n = 0; n < width; n++)
            {
                std::string res = "";

                switch (board.at(i).at(n))
                {
                case GAME_BLOCK_EMPTY:
                    res = " ";
                    break;
                case GAME_BLOCK_WALL:
                    res = "*";
                    break;
                default:
                    res = "o";
                }

                file << res;
            }

            file << "\n";
        }

        file << "Board: " << width2 << "x" << height2 << "\n";

        file.close();
    }

    void Board::print_board_to_command_line(std::map<jps_maze_msgs::msg::Player::_id_type, Player> players) const
    {
        RCLCPP_INFO(this->logger, "Width: %d Height: %d", width, height);

        for (int i = 0; i < height; i++)
        {
            std::string res = "";
            res.reserve(width);

            for (int n = 0; n < width; n++)
            {
                std::string tmp = "";

                for (const auto &m : players)
                {
                    if(m.second.get_x() == n && m.second.get_y() == i)
                    {
                        if(m.second.get_team() == PLAYER_TEAM_A)
                        {
                            if(m.second.get_has_flag() == true) tmp = "A";
                            else tmp = "a";
                        }
                        else
                        {
                            if(m.second.get_has_flag() == true) tmp = "B";
                            else tmp = "b";
                        }
                        
                        break;
                    }
                }

                if(tmp == "")
                {
                    switch (board.at(i).at(n).game_block_type)
                    {
                    case GAME_BLOCK_EMPTY:
                        if (board.at(i).at(n).mapped_team_a == true || board.at(i).at(n).mapped_team_b == true)
                            tmp = "-";
                        else
                            tmp = " ";
                        break;
                    case GAME_BLOCK_WALL:
                        if (board.at(i).at(n).mapped_team_a == true || board.at(i).at(n).mapped_team_b == true)
                            tmp = "X";
                        else
                            tmp = "*";
                        break;
                    default:
                        tmp = "o";
                    }
                }

                res += tmp;
            }

            RCLCPP_INFO(this->logger, "[%2d]%s", i, res.c_str());
        }
    }

    game_block_type_t Board::get_block_state(const coord_t coord_x, const coord_t coord_y) const
    {
        if (coord_x >= width || coord_y >= height)
            return GAME_BLOCK_OUT_OF_BOARD;

        RCLCPP_WARN(this->logger, "[Board::get_block_state] x: %d y: %d - %d", coord_x, coord_y, board.at(coord_y).at(coord_x).game_block_type);

        return board.at(coord_y).at(coord_x).game_block_type;
    }

    void Board::map_area(const coord_t coord_x, const coord_t coord_y, team_t team)
    {
        const uint8_t size = 3; // maps a 3x3 square at the player's position

        RCLCPP_DEBUG(this->logger, "[Board::map_area] x: %d y: %d for team: %c", coord_x, coord_y, (team == PLAYER_TEAM_A) ? 'A' : 'B');

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

    bool Board::player_move(const direction_t dir, Player &player)
    {
        if (player.get_turn() == false)
            return false;

        coord_t new_x = player.get_x();
        coord_t new_y = player.get_y();

        RCLCPP_DEBUG(logger, "[Board::player_move] move player at x: %d y: %d in direction: %d", new_x, new_y, dir);

        switch (dir)
        {
        case PLAYER_DIR_LEFT:
            if (new_x <= 0)
                return false;
            new_x -= 1;
            new_y = new_y;
            break;

        case PLAYER_DIR_RIGHT:
            new_x += 1;
            new_y = new_y;
            break;

        case PLAYER_DIR_UP:
            if (new_y <= 0)
                return false;
            new_x = new_x;
            new_y -= 1;
            break;

        case PLAYER_DIR_DOWN:
            new_x = new_x;
            new_y += 1;
            break;

        default:
            throw std::runtime_error("[Board::player_move] Unknown direction given");
        }

        game_block_type_t new_block = get_block_state(new_x, new_y);

        switch (new_block)
        {
        case GAME_BLOCK_OUT_OF_BOARD:
            return false;

        case GAME_BLOCK_EMPTY:
            player.set_x(new_x);
            player.set_y(new_y);
            map_area(new_x, new_y, player.get_team());
            return true;

        case GAME_BLOCK_WALL:
            return false;

        case GAME_BLOCK_PORTAL:
        {
            uint16_t port_id = rand() % portals.size();
            player.set_x(portals.at(port_id).get_x());
            player.set_y(portals.at(port_id).get_y());
            map_area(new_x, new_y, player.get_team());
            map_area(player.get_x(), player.get_y(), player.get_team());
        }
            return true;

        case GAME_BLOCK_FLAG_A:
            player.set_x(new_x);
            player.set_y(new_y);
            map_area(new_x, new_y, player.get_team());
            if (player.get_team() == PLAYER_TEAM_A)
            {
                flag_a = GAME_FLAG_STATE_BY_PLAYER;
                player.set_has_flag(true);
                board.at(new_y).at(new_x) = GAME_BLOCK_EMPTY;
            }
            return true;

        case GAME_BLOCK_FLAG_B:
            player.set_x(new_x);
            player.set_y(new_y);
            map_area(new_x, new_y, player.get_team());
            if (player.get_team() == PLAYER_TEAM_B)
            {
                flag_b = GAME_FLAG_STATE_BY_PLAYER;
                player.set_has_flag(true);
                board.at(new_y).at(new_x) = GAME_BLOCK_EMPTY;
            }
            return true;

        case GAME_BLOCK_BASE_A:
            player.set_x(new_x);
            player.set_y(new_y);
            map_area(new_x, new_y, player.get_team());
            return true;

        case GAME_BLOCK_BASE_B:
            player.set_x(new_x);
            player.set_y(new_y);
            map_area(new_x, new_y, player.get_team());
            return true;

        default:
            throw std::runtime_error("[Board::player_move] Unknown block type");
        }
    }

    std::vector<std::vector<game_block_type_t>> Board::get_team_board(const team_t team) const
    {
        std::vector<std::vector<game_block_type_t>> res;
        res.reserve(height);

        for (coord_t h = 0; h < height; h++)
        {
            std::vector<game_block_type_t> tmp;
            tmp.reserve(width);

            for (coord_t w = 0; w < width; w++)
            {
                if (team == PLAYER_TEAM_A)
                {
                    if (board.at(h).at(w).mapped_team_a == false)
                    {
                        tmp.push_back(GAME_BLOCK_EMPTY);
                    }
                    else
                    {
                        tmp.push_back(board.at(h).at(w).game_block_type);
                    }
                }
                else if (team == PLAYER_TEAM_B)
                {
                    if (board.at(h).at(w).mapped_team_b == false)
                    {
                        tmp.push_back(GAME_BLOCK_EMPTY);
                    }
                    else
                    {
                        tmp.push_back(board.at(h).at(w).game_block_type);
                    }
                }
            }

            res.push_back(tmp);
        }

        return res;
    }

    std::vector<std::vector<game_block_type_t>> Board::get_board() const
    {
        std::vector<std::vector<game_block_type_t>> res;
        res.reserve(height);

        for (coord_t h = 0; h < height; h++)
        {
            std::vector<game_block_type_t> tmp;
            tmp.reserve(width);

            for (coord_t w = 0; w < width; w++)
            {
                tmp.push_back(board.at(h).at(w).game_block_type);
            }

            res.push_back(tmp);
        }

        return res;
    }

    std::vector<std::pair<coord_t, coord_t>> Board::get_base(team_t team) const
    {
        return team == PLAYER_TEAM_A ? base_a : base_b;
    }
}