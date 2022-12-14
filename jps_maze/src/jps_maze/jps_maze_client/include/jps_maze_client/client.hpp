//
// Created by Johan on 04.11.2022.
//

#ifndef INTP_ROS_CLIENT_NODE_HPP
#define INTP_ROS_CLIENT_NODE_HPP

#include <stdexcept>
#include <random>
#include <time.h>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"

#include "jps_maze_msgs/msg/status.hpp"
#include "jps_maze_msgs/msg/team.hpp"
#include "jps_maze_msgs/srv/create_player.hpp"
#include "jps_maze_msgs/srv/move_player.hpp"

#include "jps_maze_game/game.hpp"

#include "jps_maze_visualizer/visualizer.hpp"

namespace jps_maze_client {

    class Client : public rclcpp::Node {
    public:
        Client(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());
        void request_player();

    private:

        void calculate_next_move();

        //Callbackgroups
        std::shared_ptr<rclcpp::CallbackGroup> create_player_cb_group;
        std::shared_ptr<rclcpp::CallbackGroup> status_cb_group;
        std::shared_ptr<rclcpp::CallbackGroup> move_player_cb_group;

        // Subscriber
        std::shared_ptr<rclcpp::Subscription<jps_maze_msgs::msg::Status>> team_status_sub;
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> next_round_sub;

        // Clients
        std::shared_ptr<rclcpp::Client<jps_maze_msgs::srv::CreatePlayer>> create_player_clt;
        std::shared_ptr<rclcpp::Client<jps_maze_msgs::srv::MovePlayer>> move_player_clt;

        // Guard Conditions
        std::shared_ptr<rclcpp::GuardCondition> got_player_guard;
        std::shared_ptr<rclcpp::WaitSet> got_player_wait_set;

        volatile bool already_got_player;

        std::shared_ptr<rclcpp::GuardCondition> next_move_ready_guard;
        std::shared_ptr<rclcpp::WaitSet> next_move_ready_wait_set;

        volatile bool next_move_ready;

        // Callbacks
        void status_cb(const std::shared_ptr<jps_maze_msgs::msg::Status> msg);
        void next_round_cb(const std::shared_ptr<std_msgs::msg::Empty> msg);

        // Attributes
        std::string player_name;
        std::string host_name;
        std::string target_port;
        jps_maze_game::coord_t width;
        jps_maze_game::coord_t height;
        jps_maze_game::player_id_t player_id;
        jps_maze_game::direction_t next_dir, cur_dir;
        jps_maze_game::team_t team;
        jps_maze_visualizer::Visualizer visualizer;
        jps_maze_visualizer::block_t **frame_buffer;
        jps_maze_game::coord_t x = 0, y = 0;
        std::string create_player_topic;
        std::mt19937 dir_gen;
        std::uniform_int_distribution<> next_rotation;
    };
}

#endif //INTP_ROS_CLIENT_NODE_HPP
