#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "action_cpp/action/move_to_pose.hpp"

class MyActionClient : public rclcpp::Node {
 public:
  using MyAction = action_cpp::action::MoveToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MyAction>;

  MyActionClient() : Node("move_to_pose_action_client") {
    action_client_ =
        rclcpp_action::create_client<MyAction>(this, "move_to_pose");

    // Subscribe to a topic that publishes geometry_msgs::msg::Pose
    subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/goal_pose", 10,
        std::bind(&MyActionClient::goal_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[constructor] Action Client is ready!");
  }

 private:
  rclcpp_action::Client<MyAction>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
  rclcpp_action::ClientGoalHandle<MyAction>::SharedPtr current_goal_handle_;

  void send_new_goal() {
    auto send_goal_options = rclcpp_action::Client<MyAction>::SendGoalOptions();
    send_goal_options.result_callback =
        [this](const GoalHandle::WrappedResult &result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_WARN(this->get_logger(), "[send_new_goal] Succeeded");
            current_goal_handle_ = nullptr;
          } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
            current_goal_handle_ = nullptr;
            RCLCPP_WARN(this->get_logger(), "[send_new_goal] Cancelled");
          } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_WARN(this->get_logger(), "[send_new_goal] Aborted");
            current_goal_handle_ = nullptr;
          } else if (result.code == rclcpp_action::ResultCode::UNKNOWN) {
            RCLCPP_WARN(this->get_logger(), "[send_new_goal] Unknown");
            current_goal_handle_ = nullptr;
          }
        };

    RCLCPP_INFO(this->get_logger(), "[send_new_goal] sending new goal");
    auto goal = MyAction::Goal();
    goal.target_pose = true;
    // Send the goal asynchronously
    auto future_goal_handle =
        action_client_->async_send_goal(goal, send_goal_options);

    // Handle future goal assignment
    std::thread([this, future_goal_handle]() {
      // Blocks until goal handle is ready
      auto goal_handle = future_goal_handle.get();
      if (goal_handle) current_goal_handle_ = goal_handle;
    }).detach();
  }

  void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    using namespace std::chrono_literals;
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(),
                   "[goal_callback] Action Server not available!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[goal_callback]  Received new goal!");
    // If an active goal exists, cancel it before sending a new one
    if (current_goal_handle_) {
      RCLCPP_WARN(this->get_logger(),
                  "[goal_callback] Current Goal Active, Cancelling it");

      auto cancel_future =
          action_client_->async_cancel_goal(current_goal_handle_);
    }
    send_new_goal();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyActionClient>());
  rclcpp::shutdown();
  return 0;
}
