//
// Created by Johan on 04.11.2022.
//

#include "jps_maze_server/server.hpp"

using namespace std::placeholders;
using namespace std::literals::chrono_literals;

namespace jps_maze_server {
    Server::Server(const rclcpp::NodeOptions &node_options)
        : rclcpp::Node("server_node", node_options) , game(this->get_logger()){

        // Declare Parameters
        this->declare_parameter<std::string>("create_player_topic");
        this->declare_parameter<std::string>("status_topic");
        this->declare_parameter<std::string>("move_player_topic");
        this->declare_parameter<std::string>("next_round_topic");
        this->declare_parameter<std::string>("board_path");
        this->declare_parameter<int64_t>("player_per_team");

        // Get Parameters
        const std::string create_player_topic = this->get_parameter("create_player_topic").as_string();
        const std::string status_topic = this->get_parameter("status_topic").as_string();
        const std::string move_player_topic = this->get_parameter("move_player_topic").as_string();
        const std::string next_round_topic = this->get_parameter("next_round_topic").as_string();
        const std::string board_path = this->get_parameter("board_path").as_string();
        const uint8_t player_per_team = this->get_parameter("player_per_team").as_int();

        RCLCPP_INFO(this->get_logger(), "Got all required parameters");

        this->timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        this->timer = this->create_wall_timer(1s, std::bind(&Server::timer_cb, this), this->timer_cb_group);
        this->timer->cancel();

        // Register Publisher
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

        RCLCPP_INFO(this->get_logger(), "Init of Node done");
    }

    void Server::send_status() {
        jps_maze_msgs::msg::Status status;
        this->game.get_status(jps_maze_game::PLAYER_TEAM_A, status);
        status.header.stamp = this->now();
        this->team_a_status_pub->publish(status);
        this->game.get_status(jps_maze_game::PLAYER_TEAM_B, status);
        status.header.stamp = this->now();
        this->team_b_status_pub->publish(status);
        this->timer->reset();
    }

    void Server::create_player_cb(const std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Request> req,
                                  std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Response> res) {
        RCLCPP_INFO(this->get_logger(), "Got new player spawn request with name: \"%s\" and team: %c", req->name.c_str(), req->team.team == req->team.TEAM_A ? 'A': 'B');

        jps_maze_game::Player player = this->game.add_player(req->name, static_cast<jps_maze_game::team_t>(req->team.team));
        res->player.id = player.get_player_id();
        res->player.team.team = static_cast<jps_maze_msgs::msg::Team::_team_type>(player.get_team());
        res->player.color = player.get_color();
        res->player.pos.x = player.get_x();
        res->player.pos.y = player.get_y();
        res->player.name = player.get_player_name();
        res->player.has_flag = player.get_has_flag();
        res->width = this->game.get_width();
        res->height = this->game.get_height();

        if(this->game.ready()) {
            RCLCPP_INFO(this->get_logger(), "Game is ready unregister create_player service and sending first status");
            this->create_player_srv.reset();
            this->send_status();
        }

        RCLCPP_INFO(this->get_logger(), "Returning player object with id: %ld at pos x: %d, y: %d", res->player.id, res->player.pos.x, res->player.pos.y);
        res->header.stamp = this->now();
    }

    void Server::move_player_cb(const std::shared_ptr<jps_maze_msgs::srv::MovePlayer::Request> req,
                                std::shared_ptr<jps_maze_msgs::srv::MovePlayer::Response> res) {
        RCLCPP_INFO(this->get_logger(), "Got new move player request for player_id: %ld dir: %s", req->player_id, req->dir == req->UP ? "UP" : (req->dir == req->DOWN ? "DOWN" : (req->dir == req->LEFT ? "LEFT" : "RIGHT")));
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
        this->next_round_pub->publish(std_msgs::msg::Empty());
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
