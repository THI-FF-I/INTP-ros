//
// Created by Johan on 04.11.2022.
//

#include "jps_maze_client/client.hpp"

using namespace std::placeholders;

namespace jps_maze_client
{
    Client::Client(rclcpp::NodeOptions node_options)
        : rclcpp::Node("client_node", node_options), already_got_player(false),  next_move_ready(false), visualizer(this->get_logger())
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

        RCLCPP_INFO(this->get_logger(), "Got all required parameters");

        this->create_player_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        this->status_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        this->move_player_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        RCLCPP_INFO(this->get_logger(), "Created callback groups");

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = this->status_cb_group;

        // Register Subscriber
        this->team_status_sub = this->create_subscription<jps_maze_msgs::msg::Status>(status_topic, 10,
                                                                                      std::bind(&Client::status_cb, this, _1), sub_options);
        this->next_round_sub = this->create_subscription<std_msgs::msg::Empty>(next_round_topic, 10, std::bind(&Client::next_round_cb, this, _1));

        RCLCPP_INFO(this->get_logger(), "Registered subscribers");

        // Register Clients
        this->create_player_clt = this->create_client<jps_maze_msgs::srv::CreatePlayer>(create_player_topic, rmw_qos_profile_services_default, this->create_player_cb_group);
        this->move_player_clt = this->create_client<jps_maze_msgs::srv::MovePlayer>(move_player_topic, rmw_qos_profile_default, this->move_player_cb_group);

        RCLCPP_INFO(this->get_logger(), "Registered clients");

        this->got_player_wait_set = std::make_shared<rclcpp::WaitSet>();
        this->got_player_guard = std::make_shared<rclcpp::GuardCondition>();
        this->got_player_wait_set->add_guard_condition(this->got_player_guard);

        this->next_move_ready_wait_set = std::make_shared<rclcpp::WaitSet>();
        this->next_move_ready_guard = std::make_shared<rclcpp::GuardCondition>();
        this->next_move_ready_wait_set->add_guard_condition(this->next_move_ready_guard);

        RCLCPP_INFO(this->get_logger(), "Set up guard conditions");

        this->request_player();

        RCLCPP_INFO(this->get_logger(), "Requested player");

        RCLCPP_INFO(this->get_logger(), "Init of Node done");

        srand(time(0));
    }

    void Client::request_player()
    {
        RCLCPP_INFO(this->get_logger(), "Requesting player with name: \"%s\"", player_name.c_str());
        auto req = std::make_shared<jps_maze_msgs::srv::CreatePlayer::Request>();
        req->name = player_name;
        req->team.team = this->team;
        RCLCPP_DEBUG(this->get_logger(), "Sending player request");
        req->header.stamp = this->now();
        this->create_player_clt->async_send_request(req, [this](std::shared_future<jps_maze_msgs::srv::CreatePlayer::Response::SharedPtr> fut) {
            std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Response> res = fut.get();
            if(!res->success) {
                RCLCPP_FATAL(this->get_logger(), "Player request failed");
                throw std::runtime_error("Player request failed");
            }
            RCLCPP_INFO(this->get_logger(), "Got back Player with id: %lu, at x: %d y: %d", res->player.id,
                        res->player.pos.x, res->player.pos.y);
            this->x = res->player.pos.x;
            this->y = res->player.pos.y;
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
            RCLCPP_DEBUG(this->get_logger(), "Signaling that player was received");
            this->already_got_player = true;
            this->got_player_guard->trigger();
        });
    }

    void Client::calculate_next_move()
    {
        RCLCPP_DEBUG(this->get_logger(), "[Client::calculate_next_move] starting");

        const jps_maze_game::coord_t cur_pos[2] = {this->x, this->y};
        jps_maze_game::coord_t next_pos[2] = {this->x, this->y};
        jps_maze_game::direction_t next_dir_res = cur_dir;

        size_t cnt = 0;
        bool success = false;

        RCLCPP_DEBUG(this->get_logger(), "[Client::calculate_next_move] x: %d y: %d  cur_dir: %d", x, y, cur_dir);

        while (cur_pos[0] == next_pos[0] && cur_pos[1] == next_pos[1] && cnt < 5)
        {
            cnt++;

            if(cnt == 2)
            {
                auto tmp = next_dir_res;

                while(tmp == next_dir_res) next_dir_res = (jps_maze_game::direction_t) (rand() % 3);
            }

            RCLCPP_DEBUG(this->get_logger(), "[Client::calculate_next_move] Trying direction: %d", next_dir_res);

            switch (next_dir_res)
            {
            case jps_maze_game::PLAYER_DIR_LEFT:
                if (next_pos[0] > 0)
                    next_pos[0] -= 1;
                break;
            case jps_maze_game::PLAYER_DIR_RIGHT:
                if (next_pos[0] < height - 1)
                    next_pos[0] += 1;
                break;
            case jps_maze_game::PLAYER_DIR_UP:
                if (next_pos[1] > 0)
                    next_pos[1] -= 1;
                break;
            case jps_maze_game::PLAYER_DIR_DOWN:
                if (next_pos[1] < width - 1)
                    next_pos[1] += 1;
                break;
            }

            if (next_pos[0] >= width || next_pos[1] >= height)
                throw std::runtime_error("[Client::calculate_next_move] Position out of bound"); // This shouldn't happen

            RCLCPP_DEBUG(this->get_logger(), "[Client::calculate_next_move] Checking: x: %d y: %d - %d", next_pos[0], next_pos[1], frame_buffer[next_pos[1]][next_pos[0]]);

            if (frame_buffer[next_pos[1]][next_pos[0]] != jps_maze_msgs::msg::Block::WALL && !(cur_pos[0] == next_pos[0] && cur_pos[1] == next_pos[1]))
            {
                RCLCPP_DEBUG(this->get_logger(), "[Client::calculate_next_move] found a valid direction");
                success = true;
                break;
            }
            else
            {
                if(team == jps_maze_game::PLAYER_TEAM_A) next_dir_res = (jps_maze_game::direction_t)((next_dir_res + 1) % 4);
                else next_dir_res = next_dir_res - 1 >= 0 ? (jps_maze_game::direction_t)((next_dir_res - 1)) : (jps_maze_game::direction_t)3;
                next_pos[0] = this->x;
                next_pos[1] = this->y;
            }
        }

        if (success)
        {
            next_dir = next_dir_res;
            cur_dir = next_dir_res;
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
            RCLCPP_DEBUG(this->get_logger(), "Received first status");
            if(!already_got_player) {
                RCLCPP_DEBUG(this->get_logger(), "Waiting for create_player response");
                this->got_player_wait_set->wait();
            }
            RCLCPP_INFO(this->get_logger(), "Removing guard_conditions");
            this->got_player_wait_set->remove_guard_condition(this->got_player_guard);
            this->got_player_wait_set.reset();
            this->got_player_guard.reset();
            first_status = false;
        }
        RCLCPP_INFO(this->get_logger(), "Got new status message");
        RCLCPP_DEBUG(this->get_logger(), "Number of rows: %zu, columns: %zu", msg->rows.size(), msg->rows.at(0).blocks.size());
        RCLCPP_DEBUG(this->get_logger(), "Received number of players: %lu", msg->players.size());
        RCLCPP_INFO(this->get_logger(), "Copying board into framebuffer");
        size_t x= 0, y = 0;
        for (const auto &row : msg->rows)
        {
            x= 0;
            for (const auto &block : row.blocks)
            {
                this->frame_buffer[y][x] = block.block_type;
                x++;
            }
            y++;
        }
        for (const auto &player : msg->players)
        {
            frame_buffer[player.pos.y][player.pos.x] = player.color | (static_cast<jps_maze_msgs::msg::Block::_block_type_type>(1) << std::numeric_limits<jps_maze_msgs::msg::Block::_block_type_type>::digits - 1); // Set MSB

            if (player.id == player_id)
            {
                this->x = player.pos.x;
                this->y = player.pos.y;
                RCLCPP_DEBUG(this->get_logger(), "Got a new position from server: x: %d y: %d", this->x, this->y);
            }

            if(player.team.team == jps_maze_msgs::msg::Team::TEAM_A) {
                this->frame_buffer[player.pos.y][player.pos.x] |= (static_cast<jps_maze_msgs::msg::Block::_block_type_type>(1) << std::numeric_limits<jps_maze_msgs::msg::Block::_block_type_type>::digits - 2); // Set 2nd MSB
            } else {
                this->frame_buffer[player.pos.y][player.pos.x] &= ~(static_cast<jps_maze_msgs::msg::Block::_block_type_type>(1) << std::numeric_limits<jps_maze_msgs::msg::Block::_block_type_type>::digits - 2); // Reset 2nd MSB
            }
        }
        RCLCPP_INFO(this->get_logger(), "Triggering re_draw");
        this->visualizer.re_draw();
        if(msg->game_over) {
            if(msg->winning_team.team == this->team) {
                RCLCPP_INFO(this->get_logger(), "We won!");
            } else {
                RCLCPP_INFO(this->get_logger(), "We lost!");
            }
            std::exit(EXIT_SUCCESS);
        }
        RCLCPP_INFO(this->get_logger(), "Calculating next move");
        this->calculate_next_move();
        RCLCPP_DEBUG(this->get_logger(), "Signaling, that the next round is ready");
        this->next_move_ready = true;
        this->next_move_ready_guard->trigger();
    }

    void Client::next_round_cb(const std::shared_ptr<std_msgs::msg::Empty> msg)
    {
        RCLCPP_INFO(this->get_logger(), "Next round started");
        if(!next_move_ready) {
            RCLCPP_DEBUG(this->get_logger(), "Next move is not ready waiting");
            this->next_move_ready_wait_set->wait();
        }
        auto req = std::make_shared<jps_maze_msgs::srv::MovePlayer::Request>();
        req->dir = this->next_dir;
        req->player_id = this->player_id;
        RCLCPP_INFO(this->get_logger(), "Sending player move request");
        req->header.stamp = this->now();
        this->next_move_ready = false;
        this->move_player_clt->async_send_request(req, [this](std::shared_future<jps_maze_msgs::srv::MovePlayer::Response::SharedPtr> fut) {
            auto res = fut.get();
            RCLCPP_INFO(this->get_logger(), "Got response for player move");
            if (!res->success)
            {
                RCLCPP_FATAL(this->get_logger(), "Move was rejected");
                throw std::runtime_error("Move was rejected");
            }
        });
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