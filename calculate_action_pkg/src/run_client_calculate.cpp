#include <geometry_msgs/msg/pose.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "calculate_action_pkg/action/calculate.hpp"

using Calculate = calculate_action_pkg::action::Calculate;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Calculate>;
using namespace std::chrono_literals;
using namespace std::placeholders;

class CalculateClient : public rclcpp::Node {
public:
  CalculateClient() : Node("calculate_action_client") {
    client_ = rclcpp_action::create_client<Calculate>(this, "calculate");

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/goal_pose", 10,
        std::bind(&CalculateClient::goal_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[constructor] Client is Ready !!!");
  }

private:
  rclcpp_action::Client<Calculate>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<Calculate>::SharedPtr client_goal_handle_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;

  void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {

    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(),
                   "[goal_callback] Action Server not available!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[goal_callback] Received new goal!");
    // If an active goal exists, cancel it before sending a new one
    if (client_goal_handle_) {
      RCLCPP_WARN(this->get_logger(),
                  "[goal_callback] Current Goal Active, Cancelling it");

      client_->async_cancel_goal(client_goal_handle_);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    send_new_goal_to_server();
  }

  void create_goal_action(Calculate::Goal &goal) {
    std::random_device rd;
    std::mt19937 mt(rd());

    std::uniform_real_distribution<float> value(-10.0, 10.0);
    goal.value_1 = value(mt);
    goal.value_2 = value(mt);

    std::uniform_int_distribution<int> idx(0, 3);
    std::vector<std::string> operations = {"add", "subtract", "multiply",
                                           "divide"};
    goal.operation = operations[idx(mt)];
    RCLCPP_INFO(this->get_logger(), "[create_goal_msg] Sending goal: %f %s %f",
                goal.value_1, goal.operation.c_str(), goal.value_2);
  }

  void send_new_goal_to_server() {
    auto goal = Calculate::Goal();
    create_goal_action(goal);

    auto send_goal_options =
        rclcpp_action::Client<Calculate>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&CalculateClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&CalculateClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&CalculateClient::result_callback, this, _1);

    RCLCPP_INFO(this->get_logger(),
                "[send_new_goal_to_server] Sending new goal");
    auto future_goal_handle = client_->async_send_goal(goal, send_goal_options);

    // Handle future goal assignment
    std::thread([this, future_goal_handle]() {
      // Blocks until goal handle is ready
      auto goal_handle = future_goal_handle.get();
      if (goal_handle)
        client_goal_handle_ = goal_handle;
    }).detach();
  }

  void goal_response_callback(const ClientGoalHandle::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(),
                   "[goal_response_callback] Goal was rejected by server.");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "[goal_response_callback] Goal accepted!");
    }
  }

  void
  feedback_callback(ClientGoalHandle::SharedPtr,
                    const std::shared_ptr<const Calculate::Feedback> feedback) {
    float processing_time_left = feedback->time_remaining;
    RCLCPP_DEBUG(this->get_logger(),
                 "[feedback_callback] Time left to process: %f",
                 processing_time_left);
  }

  void result_callback(const ClientGoalHandle::WrappedResult &result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(),
                  "[result_callback] Goal Succeeded, Result: %f",
                  result.result.get()->result);
      client_goal_handle_ = nullptr;
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(this->get_logger(), "[result_callback] Goal Cancelled.");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "[result_callback] Goal failed with code: %d",
                   static_cast<int>(result.code));
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalculateClient>());
  rclcpp::shutdown();
  return 0;
}