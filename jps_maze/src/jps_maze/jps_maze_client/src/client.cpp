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
        this->create_player_topic = this->get_parameter("create_player_topic").as_string();
        const std::string status_topic = this->get_parameter("status_topic").as_string();
        const std::string move_player_topic = this->get_parameter("move_player_topic").as_string();
        const std::string next_round_topic = this->get_parameter("next_round_topic").as_string();
        this->player_name = this->get_parameter("player_name").as_string();
        this->host_name = this->get_parameter("host_name").as_string();
        this->target_port = this->get_parameter("target_port").as_string();
        const bool team_A = this->get_parameter("team").as_bool();

        this->team = team_A ? jps_maze_game::PLAYER_TEAM_A : jps_maze_game::PLAYER_TEAM_B;
        this->cur_dir = team_A ? jps_maze_game::PLAYER_DIR_RIGHT : jps_maze_game::PLAYER_DIR_LEFT;

        this->create_player_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        RCLCPP_INFO(this->get_logger(), "Got all required parameters");

        // Register Subscriber
        this->team_status_sub = this->create_subscription<jps_maze_msgs::msg::Status>(this->get_effective_namespace() + (team_A ? "/team_A" : "team_B") + status_topic, 10,
                                                                                      std::bind(&Client::status_cb, this, _1));
        this->next_round_sub = this->create_subscription<std_msgs::msg::Empty>(next_round_topic, 10, std::bind(&Client::next_round_cb, this, _1));

        RCLCPP_INFO(this->get_logger(), "Registered subscribers");

        // Register Clients
        this->create_player_clt = this->create_client<jps_maze_msgs::srv::CreatePlayer>(create_player_topic, rmw_qos_profile_services_default, this->create_player_cb_group);
        this->move_player_clt = this->create_client<jps_maze_msgs::srv::MovePlayer>(move_player_topic);

        this->got_player_wait_set = std::make_shared<rclcpp::WaitSet>();
        this->got_player_guard = std::make_shared<rclcpp::GuardCondition>();
        this->got_player_wait_set->add_guard_condition(this->got_player_guard);

        this->request_player();

        RCLCPP_INFO(this->get_logger(), "Registered clients");

        RCLCPP_INFO(this->get_logger(), "Init of Node done");
    }

    void Client::request_player()
    {
        RCLCPP_INFO(this->get_logger(), "Requesting player with name: \"%s\"", player_name.c_str());
        auto req = std::make_shared<jps_maze_msgs::srv::CreatePlayer::Request>();
        req->name = player_name;
        req->team.team = this->team;
        req->header.stamp = this->now();
        this->create_player_clt->async_send_request(req, [this](std::shared_future<jps_maze_msgs::srv::CreatePlayer::Response::SharedPtr> fut) {
            RCLCPP_DEBUG(this->get_logger(), "Send player request");
            std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Response> res = fut.get();
            RCLCPP_INFO(this->get_logger(), "Got back Player with id: %ld, at x: %d y: %d", res->player.id,
                        res->player.pos.x, res->player.pos.y);
            this->player_name = res->player.name;
            this->player_id = res->player.id;
            this->team = static_cast<jps_maze_game::team_t>(res->player.team.team);
            this->width = res->width;
            this->height = res->height;
            RCLCPP_INFO(this->get_logger(), "Game board width: %d, height: %d", res->width, res->height);

            this->create_player_clt.reset();
            this->create_player_cb_group.reset();

            RCLCPP_INFO(this->get_logger(), "Initialising visualizer");

            this->visualizer = jps_maze_visualizer::Visualizer(host_name, target_port, this->width, this->height, &this->frame_buffer, this->get_logger().get_child("visualizer"));
            this->got_player_guard->trigger();
        });
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

            if (next_pos[0] >= width || next_pos[1] >= height)
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
            RCLCPP_INFO(this->get_logger(), "Next direction: %d", next_dir);
        }
        else
        {
            throw std::runtime_error("[Client::calculate_next_move] Couldn't calculate a move");
        }
    }

    void Client::status_cb(const std::shared_ptr<jps_maze_msgs::msg::Status> msg)
    {
        static bool first_status = true;
        if(first_status) {
            this->got_player_wait_set->wait();
            this->status_cb(msg);
            RCLCPP_INFO(this->get_logger(), "Removing guard_conditions");
            this->got_player_wait_set->remove_guard_condition(this->got_player_guard);
            this->got_player_wait_set.reset();
            this->got_player_guard.reset();
            first_status = false;
        }
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
            frame_buffer[player.pos.y][player.pos.y] = player.color & (static_cast<jps_maze_msgs::msg::Block::_block_type_type>(1) << (std::numeric_limits<jps_maze_msgs::msg::Block::_block_type_type>::digits - 1));

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
        fut.wait();
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
    rclcpp::executors::MultiThreadedExecutor executor;
    auto client_node = std::make_shared<jps_maze_client::Client>(node_options);
    executor.add_node(client_node);
    executor.spin();
    return EXIT_SUCCESS;
}