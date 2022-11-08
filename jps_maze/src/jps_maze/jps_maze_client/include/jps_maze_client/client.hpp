//
// Created by Johan on 04.11.2022.
//

#ifndef INTP_ROS_CLIENT_NODE_HPP
#define INTP_ROS_CLIENT_NODE_HPP

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

    private:
        // Subscriber
        std::shared_ptr<rclcpp::Subscription<jps_maze_msgs::msg::Status>> team_status_sub;
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> next_round_sub;

        // Clients
        std::shared_ptr<rclcpp::Client<jps_maze_msgs::srv::MovePlayer>> move_player_clt;

        // Callbacks
        void status_cb(const std::shared_ptr<jps_maze_msgs::msg::Status> msg);
        void next_round_cb(const std::shared_ptr<std_msgs::msg::Empty> msg);

        // Attributes
        std::string player_name;
        jps_maze_visualizer::Visualizer visualizer;
        jps_maze_visualizer::block_t **frame_buffer;
    };
}

#endif //INTP_ROS_CLIENT_NODE_HPP
