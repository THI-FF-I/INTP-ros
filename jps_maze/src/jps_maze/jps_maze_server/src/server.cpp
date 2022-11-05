//
// Created by Johan on 04.11.2022.
//

#include "jps_maze_server/server.hpp"

using namespace std::placeholders;

namespace jps_maze_server {
    Server::Server(const rclcpp::NodeOptions &node_options)
        : rclcpp::Node("server_node", node_options) , game(0, 0, this->get_logger()){

        // Declare Parameters
        this->declare_parameter<int64_t>("height");
        this->declare_parameter<int64_t>("width");
        this->declare_parameter<std::string>("create_player_topic");
        this->declare_parameter<std::string>("status_topic");
        this->declare_parameter<std::string>("move_player_topic");

        // Get Parameters
        const uint8_t height = this->get_parameter("height").as_int();
        const uint8_t width = this->get_parameter("width").as_int();
        const std::string create_player_topic = this->get_parameter("create_player_topic").as_string();
        const std::string status_topic = this->get_parameter("status_topic").as_string();
        const std::string move_player_topic = this->get_parameter("move_player_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Got all required parameters");

        // Register Publisher
        this->team_a_status_pub = this->create_publisher<jps_maze_msgs::msg::Status>(this->get_effective_namespace() + "/team_A" + status_topic, 10);
        this->team_b_status_pub = this->create_publisher<jps_maze_msgs::msg::Status>(this->get_effective_namespace() + "/team_B" + status_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Registered publishers");

        // Register Services
        this->create_player_srv = this->create_service<jps_maze_msgs::srv::CreatePlayer>(create_player_topic, std::bind(&Server::create_player_cb, this, _1, _2));
        this->move_player_srv = this->create_service<jps_maze_msgs::srv::MovePlayer>(move_player_topic, std::bind(&Server::move_player_cb, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Registered services");

        // Init Game
        this->game = Game(width, height, this->get_logger());

        RCLCPP_INFO(this->get_logger(), "Init of game object done");

        RCLCPP_INFO(this->get_logger(), "Init of Node done");
    }

    void Server::create_player_cb(const std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Request> req,
                                  std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Response> res) {
        RCLCPP_INFO(this->get_logger(), "Got new player spawn request with name: \"%s\" and team: %c", req->name.c_str(), req->team.team == req->team.TEAM_A ? 'A': 'B');

        RCLCPP_INFO(this->get_logger(), "Returning player object with id: %ld at pos x: %d, y: %d", res->player.id, res->player.pos.x, res->player.pos.y);
        res->header.stamp = this->now();
    }

    void Server::move_player_cb(const std::shared_ptr<jps_maze_msgs::srv::MovePlayer::Request> req,
                                std::shared_ptr<jps_maze_msgs::srv::MovePlayer::Response> res) {
        RCLCPP_INFO(this->get_logger(), "Got new move player request for player_id: %ld dir: %s", req->player_id, req->dir == req->UP ? "UP" : (req->dir == req->DOWN ? "DOWN" : (req->dir == req->LEFT ? "LEFT" : "RIGHT")));
        res->success = this->game.move_player(req->player_id, static_cast<direction_t>(req->dir));
        if(this->game.next_round_ready()) {
            RCLCPP_INFO(this->get_logger(), "Round %d finished sending status and moving on", this->game.get_round_cnt());
            jps_maze_msgs::msg::Status status;
            this->game.get_status(PLAYER_TEAM_A, status);
            status.header.stamp = this->now();
            this->team_a_status_pub->publish(status);
            this->game.get_status(PLAYER_TEAM_B, status);
            status.header.stamp = this->now();
            this->team_b_status_pub->publish(status);
            this->game.next_round();
        }
        res->header.stamp = this->now();
    }
}

int main(int argc, char **argv) {
    rclcpp::NodeOptions node_options;
    node_options.start_parameter_event_publisher(false);
    node_options.start_parameter_services(false);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<jps_maze_server::Server>(node_options));
}
