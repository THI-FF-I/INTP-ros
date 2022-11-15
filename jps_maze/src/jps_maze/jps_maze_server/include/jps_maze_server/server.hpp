//
// Created by Johan on 04.11.2022.
//

#ifndef INTP_ROS_SERVER_NODE_HPP
#define INTP_ROS_SERVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "std_msgs/msg/empty.hpp"

#include "jps_maze_msgs/msg/row.hpp"
#include "jps_maze_msgs/msg/status.hpp"
#include "jps_maze_msgs/srv/create_player.hpp"
#include "jps_maze_msgs/srv/move_player.hpp"

#include "jps_maze_game/game.hpp"

#include "jps_maze_visualizer/visualizer.hpp"

namespace jps_maze_server {

    class Server : public rclcpp::Node {
    public:
        Server(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());

    private:

        void send_status();
        void update_framebuffer();

        // Callbackgroups
        std::shared_ptr<rclcpp::CallbackGroup> timer_cb_group;
        //std::shared_ptr<rclcpp::CallbackGroup> pub_cb_group;

        // Timer
        std::shared_ptr<rclcpp::TimerBase> timer;
        std::shared_ptr<rclcpp::TimerBase> simon_fix_das_mal_bitte;

        // Publisher
        std::shared_ptr<rclcpp::Publisher<jps_maze_msgs::msg::Status>> team_a_status_pub;
        std::shared_ptr<rclcpp::Publisher<jps_maze_msgs::msg::Status>> team_b_status_pub;
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> next_round_pub;

        // Services
        std::shared_ptr<rclcpp::Service<jps_maze_msgs::srv::CreatePlayer>> create_player_srv;
        std::shared_ptr<rclcpp::Service<jps_maze_msgs::srv::MovePlayer>> move_player_srv;

        // Callbacks
        void create_player_cb(const std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Request> req, std::shared_ptr<jps_maze_msgs::srv::CreatePlayer::Response> res);
        void move_player_cb(const std::shared_ptr<jps_maze_msgs::srv::MovePlayer::Request> req, std::shared_ptr<jps_maze_msgs::srv::MovePlayer::Response> res);
        void timer_cb();

        // Attributes
        jps_maze_game::Game game;
        jps_maze_visualizer::Visualizer visualizer;
        jps_maze_visualizer::block_t **frame_buffer;
    };
}

#endif //INTP_ROS_SERVER_NODE_HPP
