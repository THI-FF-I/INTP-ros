#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "tictactoe/server.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher")
  {
    publisher_ = this->create_publisher<tictactoe_interfaces::msg::Board>("topic", 10);

    timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::new_round, this));
    timer_->cancel();

    AddPlayerService = this->create_service<tictactoe_interfaces::srv::AddPlayer>("add_player", std::bind(&MinimalPublisher::addPlayer, this, std::placeholders::_1, std::placeholders::_2));
    TurnService = this->create_service<tictactoe_interfaces::srv::Turn>("turn", std::bind(&MinimalPublisher::turn, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void publish_board(uint8_t player)
  {
    auto message = tictactoe_interfaces::msg::Board();
    message.player = player;
    message.gamestate.gamestate = uint8_t(gamestate);
    message.roundnumber = uint8_t(round_number);

    message.x0y0.fieldstate = board[0][0];
    message.x1y0.fieldstate = board[1][0];
    message.x2y0.fieldstate = board[2][0];

    message.x0y1.fieldstate = board[0][1];
    message.x1y1.fieldstate = board[1][1];
    message.x2y1.fieldstate = board[2][1];

    message.x0y2.fieldstate = board[0][2];
    message.x1y2.fieldstate = board[1][2];
    message.x2y2.fieldstate = board[2][2];

    RCLCPP_INFO(this->get_logger(), "Publishing board with state: %d and round number: %d", message.gamestate.gamestate, message.roundnumber);
    publisher_->publish(message);
  }

  gamestate_t check_for_win(field_t player)
  {
    gamestate_t res = RUNNING;
    bool win = false;

    do
    {
      // horizontal checking
      for (uint8_t y = 0; y < 3; y++)
      {
        uint8_t count = 0;

        for(uint8_t x = 0; x < 3; x++)
        {
          if(board[x][y] == player)
          {
            count++;
          }
          else
          {
            count = 0;
            break;
          }
        }

        if(count == 3)
        {
          win = true;
          break;
        }
      }

      // vertical checking
      for(uint8_t x = 0; x < 3; x++)
      {
        uint8_t count = 0;

        for(uint8_t y = 0; y < 3; y++)
        {
          if(board[x][y] == player)
          {
            count++;
          }
          else
          {
            count = 0;
            break;
          }
        }

        if(count == 3)
        {
          win = true;
          break;
        }
      }

      // diagonal checking
      {
        uint8_t count = 0;
        for(uint8_t xy = 0; xy < 3; xy++)
        {
          if(board[xy][xy] == player)
          {
            count++;
          }
          else
          {
            count = 0;
            break;
          }
        }

        if(count == 3)
        {
          win = true;
          break;
        }
      }

      {
        uint8_t count = 0;
        for(uint8_t xy = 0; xy < 3; xy++)
        {
          if(board[xy][2 - xy] == player)
          {
            count++;
          }
          else
          {
            count = 0;
            break;
          }
        }

        if(count == 3)
        {
          win = true;
          break;
        }
      }
    } while (false);

    if(win == true)
    {
      switch(player)
      {
        case PLAYER_A:
          res = WIN_PLAYER_A;
          break;
        case PLAYER_B:
          res = WIN_PLAYER_B;
          break;
        default:
          break;
      }
    }
    else
    {
      // check for draw
      bool draw = true;

      for(uint8_t x = 0; x < 3; x++)
      {
        for(uint8_t y = 0; y < 3; y++)
        {
          if (board[x][y] == EMPTY)
          {
            draw = false;
            break;
          }
        }

        if(draw == false) break;
      }

      if (draw == true)
      {
        res = DRAW;
      }
    }

    return res;
  }

  void turn(const std::shared_ptr<tictactoe_interfaces::srv::Turn::Request> request,
            std::shared_ptr<tictactoe_interfaces::srv::Turn::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Got Turn Request from player %d: x: %u, y: %u", request->player, request->field_x, request->field_y);

    bool res = false;

    if (request->field_x <= 2 && request->field_y <= 2 && board[request->field_x][request->field_y] == EMPTY)
    {
      board[request->field_x][request->field_y] = (field_t)request->player;
      player_turn[request->player - 1] = true;
      res = true;
    }
    else
      res = false;

    gamestate = check_for_win((field_t) request->player);
    RCLCPP_INFO(this->get_logger(), "Gamestate: %d", gamestate);

    response->ok = res;
    if(res == false) return;

    if (gamestate == WIN_PLAYER_A || gamestate == WIN_PLAYER_B || gamestate == DRAW)
    {
      timer_->reset();
    }

    if (player_turn[0] == true && player_turn[1] == true)
    {
      timer_->reset();
    }
    else if (player_turn[0] == true && player_turn[1] == false)
    {
      publish_board(PLAYER_B);
    }

    RCLCPP_INFO(this->get_logger(), "Finishing Turn Request from player %d", request->player);
  }

  void new_round()
  {
    timer_->cancel();

    if (gamestate == WIN_PLAYER_A || gamestate == WIN_PLAYER_B || gamestate == DRAW)
    {
      publish_board(0);
      RCLCPP_INFO(this->get_logger(), "Game ended - Gamestate: %d", gamestate);
      exit(EXIT_SUCCESS);
    }

    gamestate = RUNNING;
    round_number++;
    player_turn[0] = false;
    player_turn[1] = false;
    RCLCPP_INFO(this->get_logger(), "Starting new round: %ld", round_number);
    publish_board(PLAYER_A);
  }

  void addPlayer(const std::shared_ptr<tictactoe_interfaces::srv::AddPlayer::Request> request,
                 std::shared_ptr<tictactoe_interfaces::srv::AddPlayer::Response> response)
  {
    bool res = false;
    RCLCPP_INFO(this->get_logger(), "Got addPlayer Request: %d", request->player);

    if (player[request->player - 1] == false)
    {
      player[request->player - 1] = true;
      res = true;
      RCLCPP_INFO(this->get_logger(), "addPlayer successfull: %d", request->player);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "ERROR: addPlayer failed: %d", request->player);
    }

    response->ok = res;

    if (player[0] == true && player[1] == true)
    {
      round_number = 0;
      RCLCPP_INFO(this->get_logger(), "All players connected - Starting round: %ld", round_number + 1);
      new_round();
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tictactoe_interfaces::msg::Board>::SharedPtr publisher_;
  rclcpp::Service<tictactoe_interfaces::srv::AddPlayer>::SharedPtr AddPlayerService;
  rclcpp::Service<tictactoe_interfaces::srv::Turn>::SharedPtr TurnService;
  size_t count_;
  gamestate_t gamestate = WAITING_FOR_PLAYERS;

  size_t round_number = 0;
  bool player[2] = {false, false};
  bool player_turn[2] = {false, false};
  field_t board[3][3] = {{EMPTY, EMPTY, EMPTY}, {EMPTY, EMPTY, EMPTY}, {EMPTY, EMPTY, EMPTY}};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto pub = std::make_shared<MinimalPublisher>();
  rclcpp::spin(pub);
  rclcpp::shutdown();
  return 0;
}
