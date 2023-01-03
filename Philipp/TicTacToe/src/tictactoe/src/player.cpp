#include <functional>
#include <memory>

#include "tictactoe/player.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(char *argv[]) : Node("minimal_subscriber")
  {
    player = atoi(argv[1]);

    node2 = std::make_shared<rclcpp::Node>("minimal_subscriber_2");

    client_add_player = this->create_client<tictactoe_interfaces::srv::AddPlayer>("add_player");
    client_turn = node2->create_client<tictactoe_interfaces::srv::Turn>("turn");

    subscription_ = this->create_subscription<tictactoe_interfaces::msg::Board>("topic", 10, std::bind(&MinimalSubscriber::board_update, this, _1));
    timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalSubscriber::turn, this));
    timer_->cancel();

    register_player();
  }

private:
  void register_player()
  {
    auto request = std::make_shared<tictactoe_interfaces::srv::AddPlayer::Request>();
    request->player = player;

    while (!client_add_player->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        exit(0);
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto result = client_add_player->async_send_request(request);

    struct noop_deleter
    {
      void operator()(void *) {}
    };
    std::shared_ptr<rclcpp::Node> node(this, noop_deleter());

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      if (result.get()->ok)
      {
        RCLCPP_INFO(this->get_logger(), "Register player successfull.");
        client_add_player.reset();
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Couldn't register player.");
        exit(EXIT_FAILURE);
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service turn_player");
    }
  }

  void turn()
  {
    if (new_turn == false)
      return;
    new_turn = false;
    timer_->cancel();

    while (true)
    {
      int x = 0;
      int y = 0;

      std::cout << "X: ";
      std::cin >> x;
      std::cout << "Y: ";
      std::cin >> y;
      
      if(!std::cin.good())
      {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        RCLCPP_ERROR(this->get_logger(), "Invalid input. Try again.");
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Starting turn");

      auto request = std::make_shared<tictactoe_interfaces::srv::Turn::Request>();
      request->player = player;
      request->field_x = x;
      request->field_y = y;

      while (!client_turn->wait_for_service(1s))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          exit(EXIT_SUCCESS);
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
      }

      auto result = client_turn->async_send_request(request);

      if (rclcpp::spin_until_future_complete(node2, result) == rclcpp::FutureReturnCode::SUCCESS)
      {
        if (!result.get()->ok)
        {
          RCLCPP_ERROR(this->get_logger(), "Invalid turn. Try again.");
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Turn successfull.");
          break;
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service turn_player");
      }
    }
  }

  void board_update(const tictactoe_interfaces::msg::Board &msg)
  {
    if (msg.player == player || msg.player == 0)
    {
      RCLCPP_INFO(this->get_logger(), " ");
      RCLCPP_INFO(this->get_logger(), "Round number: %d", msg.roundnumber);

      RCLCPP_INFO(this->get_logger(), "       x");
      RCLCPP_INFO(this->get_logger(), "     0 1 2");
      RCLCPP_INFO(this->get_logger(), "     _____");
      RCLCPP_INFO(this->get_logger(), "  0 |%d %d %d", msg.x0y0.fieldstate, msg.x1y0.fieldstate, msg.x2y0.fieldstate);
      RCLCPP_INFO(this->get_logger(), "y 1 |%d %d %d", msg.x0y1.fieldstate, msg.x1y1.fieldstate, msg.x2y1.fieldstate);
      RCLCPP_INFO(this->get_logger(), "  2 |%d %d %d", msg.x0y2.fieldstate, msg.x1y2.fieldstate, msg.x2y2.fieldstate);

      if (msg.gamestate.gamestate == tictactoe_interfaces::msg::State::RUNNING)
      {
        new_turn = true;
        timer_->reset();
      }
      else
      {
        switch(msg.gamestate.gamestate)
        {
          case WIN_PLAYER_A:
            if (player == PLAYER_A)
            {
              RCLCPP_INFO(this->get_logger(), "Congratulation! You won the game.\nRounds: %d", msg.roundnumber);
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "You lost the game.\nRounds: %d", msg.roundnumber);
            }
            break;
          case WIN_PLAYER_B:
            if (player == PLAYER_B)
            {
              RCLCPP_INFO(this->get_logger(), "Congratulation! You won the game.\nRounds: %d", msg.roundnumber);
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "You lost the game.\nRounds: %d", msg.roundnumber);
            }
            break;
          case DRAW:
            RCLCPP_INFO(this->get_logger(), "Draw! Try again next time.\nRounds: %d", msg.roundnumber);
            break;
        }

        exit(EXIT_SUCCESS);
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for the other player...");
    }
  }

  rclcpp::Node::SharedPtr node2;
  rclcpp::Subscription<tictactoe_interfaces::msg::Board>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<tictactoe_interfaces::srv::Turn>::SharedPtr client_turn;
  rclcpp::Client<tictactoe_interfaces::srv::AddPlayer>::SharedPtr client_add_player;

  uint8_t player = 0;
  bool new_turn = false;
};

int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    std::cout << "Enter '1' for Player A and '2' for Player B to continue.\nExiting game." << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>(argv));
  rclcpp::shutdown();
  return 0;
}
