//
// Created by Johan on 04.11.2022.
//

#include "jps_maze_server/server.hpp"

using namespace std::placeholders;
using namespace std::literals::chrono_literals;

namespace jps_maze_server {
    Server::Server(const rclcpp::NodeOptions &node_options)
        : rclcpp::Node("server_node", node_options) , game(this->get_logger()), visualizer(this->get_logger()) {
        // Declare Parameters
        this->declare_parameter<std::string>("create_player_topic");
        this->declare_parameter<std::string>("status_topic");
        this->declare_parameter<std::string>("move_player_topic");
        this->declare_parameter<std::string>("next_round_topic");
        this->declare_parameter<std::string>("board_path");
        this->declare_parameter<std::string>("host_name");
        this->declare_parameter<std::string>("target_port");
        this->declare_parameter<int64_t>("player_per_team");

        // Get Parameters
        const std::string create_player_topic = this->get_parameter("create_player_topic").as_string();
        const std::string status_topic = this->get_parameter("status_topic").as_string();
        const std::string move_player_topic = this->get_parameter("move_player_topic").as_string();
        const std::string next_round_topic = this->get_parameter("next_round_topic").as_string();
        const std::string board_path = this->get_parameter("board_path").as_string();
        const std::string host_name = this->get_parameter("host_name").as_string();
        const std::string target_port = this->get_parameter("target_port").as_string();
        const uint8_t player_per_team = this->get_parameter("player_per_team").as_int();

        RCLCPP_INFO(this->get_logger(), "Got all required parameters");

        this->timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        this->timer = this->create_wall_timer(1s, std::bind(&Server::timer_cb, this), this->timer_cb_group);
        this->timer->cancel();

        RCLCPP_INFO(this->get_logger(), "Set up timer");

        // Register Publisher

        //this->pub_cb_group
        this->team_a_status_pub = this->create_publisher<jps_maze_msgs::msg::Status>(this->get_effective_namespace() + "/team_A" + status_topic, 10);
        this->team_b_status_pub = this->create_publisher<jps_maze_msgs::msg::Status>(this->get_effective_namespace() + "/team_B" + status_topic, 10);
        this->next_round_pub = this->create_publisher<std_msgs::msg::Empty>(next_round_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Registered publishers");

        // Register Services
        this->create_player_srv = this->create_service<jps_maze_msgs::srv::CreatePlayer>(create_player_topic, std::bind(&Server::create_player_cb, this, _1, _2));
        this->move_player_srv = this->create_service<jps_maze_msgs::srv::MovePlayer>(move_player_topic, std::bind(&Server::move_player_cb, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Registered services");

        // Init Game
        this->game = jps_maze_game::Game(board_path, player_per_team, this->get_logger().get_child("game"));

        RCLCPP_INFO(this->get_logger(), "Init of game object done");

        //Init visualizer
        this->visualizer = jps_maze_visualizer::Visualizer(host_name, target_port, this->game.get_width(), this->game.get_height(), &this->frame_buffer, this->get_logger().get_child("visualizer"));

        RCLCPP_INFO(this->get_logger(), "Init of visualizer done");

        RCLCPP_INFO(this->get_logger(), "Sending initial state of game");

        update_framebuffer();

        this->visualizer.re_draw();

        RCLCPP_INFO(this->get_logger(), "Init of Node done");
    }

    void Server::send_status() {
        jps_maze_msgs::msg::Status status;
        status.rows.reserve(this->game.get_height());
        auto board = this->game.get_team_board(jps_maze_game::PLAYER_TEAM_A);
        for(const auto &row : board) {
            jps_maze_msgs::msg::Row::_blocks_type cur_row;
            cur_row.reserve(this->game.get_width());
            for(const auto &block : row) {
                cur_row.emplace_back(jps_maze_msgs::msg::Block().set__block_type(block));
            }
            status.rows.emplace_back(jps_maze_msgs::msg::Row().set__blocks(cur_row));
            RCLCPP_DEBUG(this->get_logger(), "Added row with size %zu", cur_row.size());
        }
        RCLCPP_DEBUG(this->get_logger(), "Team A");
        RCLCPP_DEBUG(this->get_logger(), "Board: width: %ld heigth: %ld", board.size(), board.at(0).size());
        RCLCPP_DEBUG(this->get_logger(), "In board: at y = 62, x = 13: %d", board.at(62).at(13));
        RCLCPP_DEBUG(this->get_logger(), "Status Board: width: %ld heigth: %ld", status.rows.size(), status.rows.at(0).blocks.size());
        RCLCPP_DEBUG(this->get_logger(), "In message: at y=62, x=13: %d", status.rows.at(62).blocks.at(13).block_type);
        for(const auto &player : this->game.get_players_of_team(jps_maze_game::PLAYER_TEAM_A)) {
            RCLCPP_DEBUG(this->get_logger(), "Adding Player \"%s\":%lu to Team A status", player.get_player_name().c_str(), player.get_player_id());
            jps_maze_msgs::msg::Player cur;
            cur.pos.x = player.get_x();
            cur.pos.y = player.get_y();
            cur.id = player.get_player_id();
            cur.team.team = static_cast<jps_maze_msgs::msg::Team::_team_type>(player.get_team());
            cur.color = player.get_color();
            cur.name = player.get_player_name();
            cur.has_flag = player.get_has_flag();
            status.players.push_back(cur);
        }
        RCLCPP_DEBUG(this->get_logger(), "Sending status for Team A");
        status.header.stamp = this->now();
        this->team_a_status_pub->publish(status);
        status.rows.clear();
        board = this->game.get_team_board(jps_maze_game::PLAYER_TEAM_B);
        for(const auto &row : board) {
            jps_maze_msgs::msg::Row::_blocks_type cur_row;
            cur_row.reserve(this->game.get_width());
            for(const auto &block : row) {
                cur_row.emplace_back(jps_maze_msgs::msg::Block().set__block_type(block));
            }
            status.rows.emplace_back(jps_maze_msgs::msg::Row().set__blocks(cur_row));
            RCLCPP_DEBUG(this->get_logger(), "Added row with size %zu", cur_row.size());
        }
        RCLCPP_DEBUG(this->get_logger(), "Team B");
        RCLCPP_DEBUG(this->get_logger(), "In board: at y=62, x= 13: %d", board.at(62).at(13));
        RCLCPP_DEBUG(this->get_logger(), "Board: width: %ld heigth: %ld", board.size(), board.at(0).size());
        RCLCPP_DEBUG(this->get_logger(), "Status width: %zu, height: %zu", status.rows.size(), status.rows.at(0).blocks.size());
        RCLCPP_DEBUG(this->get_logger(), "In message: at y=62, x=13: %d", status.rows.at(62).blocks.at(13).block_type);
        for(const auto &player : this->game.get_players_of_team(jps_maze_game::PLAYER_TEAM_B)) {
            RCLCPP_DEBUG(this->get_logger(), "Adding Player \"%s\":%lu to Team A status", player.get_player_name().c_str(), player.get_player_id());
            jps_maze_msgs::msg::Player cur;
            cur.pos.x = player.get_x();
            cur.pos.y = player.get_y();
            cur.id = player.get_player_id();
            cur.team.team = static_cast<jps_maze_msgs::msg::Team::_team_type>(player.get_team());
            cur.color = player.get_color();
            cur.name = player.get_player_name();
            cur.has_flag = player.get_has_flag();
            status.players.push_back(cur);
        }
        RCLCPP_DEBUG(this->get_logger(), "Sending status for Team B");
        status.header.stamp = this->now();
        this->team_b_status_pub->publish(status);
        RCLCPP_DEBUG(this->get_logger(), "Updating Framebuffer");
        update_framebuffer();

        RCLCPP_INFO(this->get_logger(), "Issuing redrawing");
        this->visualizer.re_draw();

        RCLCPP_DEBUG(this->get_logger(), "Resetting timer");
        this->timer->reset();
    }

    void Server::update_framebuffer() {
        size_t y = 0, x = 0;
        for(const auto &row : this->game.get_board()) {
            x = 0;
            for(const auto &block : row) {
                this->frame_buffer[y][x] = block;
                x++;
            }
            y++;
        }

        for(const auto &player : this->game.get_players()) {
            this->frame_buffer[player.second.get_y()][player.second.get_x()] = player.second.get_color() & (static_cast<jps_maze_msgs::msg::Block::_block_type_type>(1) << (std::numeric_limits<jps_maze_msgs::msg::Block::_block_type_type>::digits - 1));
        }
    }

    void Server::create_player_cb(const std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Request> req,
                                  std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Response> res) {
        RCLCPP_INFO(this->get_logger(), "Got new player spawn request with name: \"%s\" and team: %c", req->name.c_str(), req->team.team == req->team.TEAM_A ? 'A': 'B');

        jps_maze_game::Player player;

        try
        {
            player = this->game.add_player(req->name, static_cast<jps_maze_game::team_t>(req->team.team));
        }
        catch(std::runtime_error &err)
        {
            RCLCPP_INFO(this->get_logger(), "Could not create player: \"%s\"", err.what());
            res->success = false;
            std::this_thread::sleep_for(5s);
            RCLCPP_INFO(this->get_logger(), "Returning player object with id: %lu at pos x: %d, y: %d", res->player.id, res->player.pos.x, res->player.pos.y);
            res->header.stamp = this->now();
            return;
        }
        res->player.id = player.get_player_id();
        res->player.team.team = static_cast<jps_maze_msgs::msg::Team::_team_type>(player.get_team());
        res->player.color = player.get_color();
        res->player.pos.x = player.get_x();
        res->player.pos.y = player.get_y();
        res->player.name = player.get_player_name();
        res->player.has_flag = player.get_has_flag();
        res->width = this->game.get_width();
        res->height = this->game.get_height();
        res->success = true;

        if(this->game.ready()) {
            RCLCPP_INFO(this->get_logger(), "Game is ready unregister create_player service and sending first status");
            this->create_player_srv.reset();
            this->send_status();
        }
        std::this_thread::sleep_for(5s);
        RCLCPP_INFO(this->get_logger(), "Returning player object with id: %lu at pos x: %d, y: %d", res->player.id, res->player.pos.x, res->player.pos.y);
        res->header.stamp = this->now();
    }

    void Server::move_player_cb(const std::shared_ptr<jps_maze_msgs::srv::MovePlayer::Request> req,
                                std::shared_ptr<jps_maze_msgs::srv::MovePlayer::Response> res) {
        RCLCPP_INFO(this->get_logger(), "Got new move player request for player_id: %lu dir: %s", req->player_id, req->dir == req->UP ? "UP" : (req->dir == req->DOWN ? "DOWN" : (req->dir == req->LEFT ? "LEFT" : "RIGHT")));
        res->success = this->game.move_player(req->player_id, static_cast<jps_maze_game::direction_t>(req->dir));
        if(this->game.next_round_ready()) {
            RCLCPP_INFO(this->get_logger(), "Round %d finished sending status and moving on", this->game.get_round_cnt());
            this->send_status();
        }
        res->header.stamp = this->now();
    }

    void Server::timer_cb() {
        this->timer->cancel();
        RCLCPP_INFO(this->get_logger(), "Timer expired starting next round");
        this->game.next_round();
        RCLCPP_INFO(this->get_logger(), "Publishing next round");
        this->next_round_pub->publish(std_msgs::msg::Empty());
        RCLCPP_INFO(this->get_logger(), "End of timer");
    }
}

int main(int argc, char **argv) {
    rclcpp::NodeOptions node_options;
    node_options.start_parameter_event_publisher(false);
    node_options.start_parameter_services(false);
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto server_node = std::make_shared<jps_maze_server::Server>(node_options);
    executor.add_node(server_node);
    executor.spin();
    return EXIT_SUCCESS;
}
