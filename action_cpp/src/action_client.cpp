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

  void send_new_goal(const geometry_msgs::msg::Pose &goal_pose) {
    auto send_goal_options = rclcpp_action::Client<MyAction>::SendGoalOptions();
    send_goal_options.result_callback =
        [this](const GoalHandle::WrappedResult &result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_WARN(this->get_logger(), "[send_new_goal] Succeeded");
          } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "[send_new_goal] Cancelled");
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "[send_new_goal] Failed with code: %d",
                         static_cast<int>(result.code));
            return;
          }
        };

    RCLCPP_INFO(this->get_logger(), "[send_new_goal] sending new goal");
    auto goal = MyAction::Goal();
    goal.target_pose = true;
    auto future_goal_handle =
        action_client_->async_send_goal(goal, send_goal_options);
  }

  void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    using namespace std::chrono_literals;
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(),
                   "[goal_callback] Action Server not available!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[goal_callback]  Received new goal!");

    send_new_goal(*msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyActionClient>());
  rclcpp::shutdown();
  return 0;
}
