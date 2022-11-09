//
// Created by Johan on 04.11.2022.
//

#include "jps_maze_client/client.hpp"

using namespace std::placeholders;

namespace jps_maze_client
{
    Client::Client(rclcpp::NodeOptions node_options)
        : rclcpp::Node("client_node", node_options), visualizer(this->get_logger())
    {

        // Declare Parameters
        this->declare_parameter<std::string>("create_player_topic");
        this->declare_parameter<std::string>("status_topic");
        this->declare_parameter<std::string>("move_player_topic");
        this->declare_parameter<std::string>("next_round_topic");
        this->declare_parameter<std::string>("player_name");
        this->declare_parameter<std::string>("host_name");
        this->declare_parameter<std::string>("target_port");
        this->declare_parameter<bool>("team");

        // Get Parameters
        const std::string create_player_topic = this->get_parameter("create_player_topic").as_string();
        const std::string status_topic = this->get_parameter("status_topic").as_string();
        const std::string move_player_topic = this->get_parameter("move_player_topic").as_string();
        const std::string next_round_topic = this->get_parameter("next_round_topic").as_string();
        this->player_name = this->get_parameter("player_name").as_string();
        const std::string host_name = this->get_parameter("host_name").as_string();
        const std::string target_port = this->get_parameter("target_port").as_string();
        const bool team_A = this->get_parameter("team").as_bool();

        this->team = team_A ? jps_maze_game::PLAYER_TEAM_A : jps_maze_game::PLAYER_TEAM_B;
        this->cur_dir = team_A ? jps_maze_game::PLAYER_DIR_RIGHT : jps_maze_game::PLAYER_DIR_LEFT;

        RCLCPP_INFO(this->get_logger(), "Got all required parameters");

        // Register Subscriber
        this->team_status_sub = this->create_subscription<jps_maze_msgs::msg::Status>(this->get_effective_namespace() + (team_A ? "/team_A" : "team_B") + status_topic, 10,
                                                                                      std::bind(&Client::status_cb, this, _1));
        this->next_round_sub = this->create_subscription<std_msgs::msg::Empty>(next_round_topic, 10, std::bind(&Client::next_round_cb, this, _1));

        RCLCPP_INFO(this->get_logger(), "Registered subscribers");

        // Register Clients
        this->move_player_clt = this->create_client<jps_maze_msgs::srv::MovePlayer>(move_player_topic);

        RCLCPP_INFO(this->get_logger(), "Registered clients");

        this->request_player(create_player_topic);

        RCLCPP_INFO(this->get_logger(), "Initialising visualizer");

        this->visualizer = jps_maze_visualizer::Visualizer(host_name, target_port, this->width, this->height, &this->frame_buffer, this->get_logger().get_child("visualizer"));

        RCLCPP_INFO(this->get_logger(), "Init of Node done");
    }

    void Client::request_player(const std::string &create_player_topic)
    {
        std::shared_ptr<rclcpp::Client<jps_maze_msgs::srv::CreatePlayer>> create_player_clt = this->create_client<jps_maze_msgs::srv::CreatePlayer>(create_player_topic);
        RCLCPP_INFO(this->get_logger(), "Requesting player with name: \"%s\"", player_name.c_str());
        auto req = std::make_shared<jps_maze_msgs::srv::CreatePlayer::Request>();
        req->name = player_name;
        req->team.team = this->team;
        req->header.stamp = this->now();
        auto fut = create_player_clt->async_send_request(req);
        RCLCPP_DEBUG(this->get_logger(), "Send player request");
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut);
        std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Response> res = fut.get();
        RCLCPP_INFO(this->get_logger(), "Got back Player with id: %ld, at x: %d y: %d", res->player.id,
                    res->player.pos.x, res->player.pos.y);
        this->player_name = res->player.name;
        this->player_id = res->player.id;
        this->team = static_cast<jps_maze_game::team_t>(res->player.team.team);
        this->width = res->width;
        this->height = res->height;
        RCLCPP_INFO(this->get_logger(), "Game board width: %d, height: %d", res->width, res->height);
    }

    void Client::calculate_next_move()
    {
        const jps_maze_game::coord_t cur_pos[2] = {this->x, this->y};
        jps_maze_game::coord_t next_pos[2] = {this->x, this->y};
        jps_maze_game::direction_t next_dir_res = cur_dir;

        size_t cnt = 0;
        bool success = false;

        while (cur_pos == next_pos && cnt < 4)
        {
            cnt++;

            switch (next_dir_res)
            {
            case jps_maze_game::PLAYER_DIR_UP:
                if (next_pos[0] > 0)
                    next_pos[0] -= 1;
                break;
            case jps_maze_game::PLAYER_DIR_DOWN:
                if (next_pos[0] < height - 1)
                    next_pos[0] += 1;
                break;
            case jps_maze_game::PLAYER_DIR_LEFT:
                if (next_pos[1] > 0)
                    next_pos[1] -= 1;
                break;
            case jps_maze_game::PLAYER_DIR_RIGHT:
                if (next_pos[1] < width - 1)
                    next_pos[1] += 1;
                break;
            }

            if (next_pos[0] >= height - 1 || next_pos[1] >= width - 1)
                throw std::runtime_error("[Client::calculate_next_move] Position out of bound"); // This shouldn't happen

            if (frame_buffer[next_pos[1]][next_pos[0]] != jps_maze_msgs::msg::Block::WALL && cur_pos != next_pos)
            {
                success = true;
                break;
            }
            else
            {
                next_dir_res = (jps_maze_game::direction_t)((next_dir_res + 1) % 4);
            }
        }

        if (success)
        {
            next_dir = next_dir_res;
        }
        else
        {
            throw std::runtime_error("[Client::calculate_next_move] Couldn't calculate a move");
        }
    }

    void Client::status_cb(const std::shared_ptr<jps_maze_msgs::msg::Status> msg)
    {
        RCLCPP_INFO(this->get_logger(), "Got new status message");
        size_t i = 0;
        RCLCPP_INFO(this->get_logger(), "Copying board into framebuffer");
        for (const auto &row : msg->rows)
        {
            size_t j = 0;
            for (const auto &block : row.blocks)
            {
                this->frame_buffer[i][j] = block.block_type;
            }
        }
        for (const auto &player : msg->players)
        {
            frame_buffer[player.pos.y][player.pos.y] = player.color;

            if (player.id == player_id)
            {
                this->x = player.pos.x;
                this->y = player.pos.y;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Triggering re_draw");
        this->visualizer.re_draw();
        RCLCPP_INFO(this->get_logger(), "Calculating next move");
        this->calculate_next_move();
    }

    void Client::next_round_cb(const std::shared_ptr<std_msgs::msg::Empty> msg)
    {
        RCLCPP_INFO(this->get_logger(), "Next round started");
        auto req = std::make_shared<jps_maze_msgs::srv::MovePlayer::Request>();
        req->dir = this->next_dir;
        req->player_id = this->player_id;
        RCLCPP_INFO(this->get_logger(), "Sending player move request");
        req->header.stamp = this->now();
        auto fut = this->move_player_clt->async_send_request(req);
        rclcpp::spin_until_future_complete(this->shared_from_this(), fut);
        auto res = fut.get();
        RCLCPP_INFO(this->get_logger(), "Got response for player move");
        if (!res->success)
        {
            RCLCPP_FATAL(this->get_logger(), "Move was rejected");
            throw std::runtime_error("Move was rejected");
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::NodeOptions node_options;
    node_options.start_parameter_event_publisher(false);
    node_options.start_parameter_services(false);
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<jps_maze_client::Client>(node_options);
    rclcpp::spin(client_node);
    return EXIT_SUCCESS;
}