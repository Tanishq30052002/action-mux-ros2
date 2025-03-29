#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "action_cpp/action/move_to_pose.hpp"

class MyActionClient : public rclcpp::Node {
 public:
  using MyAction = action_cpp::action::MoveToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MyAction>;

  MyActionClient() : Node("action_client") {
    action_client_ =
        rclcpp_action::create_client<MyAction>(this, "move_to_pose");

    // Subscribe to a topic that publishes geometry_msgs::msg::Pose
    subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/goal_pose", 10,
        std::bind(&MyActionClient::goal_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action Client is ready!");
  }

 private:
  rclcpp_action::Client<MyAction>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
  rclcpp_action::ClientGoalHandle<MyAction>::SharedPtr current_goal_handle_;

  void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    using namespace std::chrono_literals;

    // if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    //   RCLCPP_ERROR(this->get_logger(), "Action Server not available!");
    //   return;
    // }

    // Cancel current goal if one exists
    if (current_goal_handle_) {
      RCLCPP_WARN(this->get_logger(), "Cancelling previous goal...");
      action_client_->async_cancel_goal(current_goal_handle_);
    }

    // Set goal with received pose message
    auto goal = MyAction::Goal();
    goal.target_pose = *msg;  // Assign the received pose directly

    RCLCPP_INFO(this->get_logger(),
                "Sending goal - Position: [x: %f, y: %f, z: %f]",
                goal.target_pose.position.x, goal.target_pose.position.y,
                goal.target_pose.position.z);

    auto send_goal_options = rclcpp_action::Client<MyAction>::SendGoalOptions();
    send_goal_options.result_callback =
        [](const GoalHandle::WrappedResult &result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(rclcpp::get_logger("action_client"),
                        "Goal reached successfully!");
          } else {
            RCLCPP_WARN(rclcpp::get_logger("action_client"),
                        "Goal execution failed!");
          }
        };

    auto future_goal_handle =
        action_client_->async_send_goal(goal, send_goal_options);
    current_goal_handle_ = future_goal_handle.get();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyActionClient>());
  rclcpp::shutdown();
  return 0;
}
