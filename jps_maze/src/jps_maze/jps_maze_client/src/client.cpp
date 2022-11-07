//
// Created by Johan on 04.11.2022.
//

#include "jps_maze_client/client.hpp"

using namespace std::placeholders;

namespace jps_maze_client {
    Client::Client(rclcpp::NodeOptions node_options)
            : rclcpp::Node("client_node", node_options) {

        // Declare Parameters
        this->declare_parameter<std::string>("create_player_topic");
        this->declare_parameter<std::string>("status_topic");
        this->declare_parameter<std::string>("move_player_topic");
        this->declare_parameter<std::string>("next_round_topic");
        this->declare_parameter<bool>("team");
        this->declare_parameter<std::string>("player_name");

        // Get Parameters
        const std::string create_player_topic = this->get_parameter("create_player_topic").as_string();
        const std::string status_topic = this->get_parameter("status_topic").as_string();
        const std::string move_player_topic = this->get_parameter("move_player_topic").as_string();
        const std::string next_round_topic = this->get_parameter("next_round_topic").as_string();
        const bool team_A = this->get_parameter("team").as_bool();
        this->player_name = this->get_parameter("player_name").as_string();

        RCLCPP_INFO(this->get_logger(), "Got all required parameters");

        // Register Subscriber
        this->team_status_sub = this->create_subscription<jps_maze_msgs::msg::Status>(this->get_effective_namespace() + (team_A ? "/team_A" : "team_B") + status_topic, 10,
                                                                                      std::bind(&Client::status_cb, this, _1));
        this->next_round_sub = this->create_subscription<std_msgs::msg::Empty>(next_round_topic, 10, std::bind(&Client::next_round_cb, this, _1));

        RCLCPP_INFO(this->get_logger(), "Registered subscribers");

        // Register Clients
        this->create_player_clt = this->create_client<jps_maze_msgs::srv::CreatePlayer>(create_player_topic);
        this->move_player_clt = this->create_client<jps_maze_msgs::srv::MovePlayer>(move_player_topic);

        RCLCPP_INFO(this->get_logger(), "Registered clients");

        RCLCPP_INFO(this->get_logger(), "Init of Node done");
    }

    void Client::status_cb(const std::shared_ptr<jps_maze_msgs::msg::Status> msg) {

    }

    void Client::next_round_cb(const std::shared_ptr<std_msgs::msg::Empty> msg) {

    }
}

int main(int argc, char **argv) {
    rclcpp::NodeOptions node_options;
    node_options.start_parameter_event_publisher(false);
    node_options.start_parameter_services(false);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<jps_maze_client::Client>(node_options));
}