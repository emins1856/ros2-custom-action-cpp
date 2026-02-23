#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <random>
#include "custom_action_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace custom_action_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = custom_action_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options), dist_(500, 2000) // Initialize distribution
  {
    
    rng_ = std::mt19937(rd_());

   
    client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

    
    set_random_interval_timer();
  }

private:
  void set_random_interval_timer()
  {
    
    int random_interval = dist_(rng_);

    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(random_interval),
      [this]() { check_and_send_goal(); }
    );
  }

  void check_and_send_goal()
  {
    // Cancel the timer after first execution
    timer_->cancel();

    if (!client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    send_goal();
  }

  void send_goal()
  {
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleFibonacci::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](GoalHandleFibonacci::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
      handle_feedback(feedback);
    };

    send_goal_options.result_callback = [this](const GoalHandleFibonacci::WrappedResult & result)
    {
      handle_result(result);
    };

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void handle_feedback(const std::shared_ptr<const Fibonacci::Feedback> & feedback)
  {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    std::stringstream ss;
    ss << "Timestamp: " << ms << " ms | Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  void handle_result(const GoalHandleFibonacci::WrappedResult & result)
  {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    std::stringstream ss;
    ss << "Timestamp: " << ms << " ms | Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    rclcpp::shutdown();
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Random interval setup
  std::random_device rd_;                   // Random device for seeding
  std::mt19937 rng_;                        // Random number generator
  std::uniform_int_distribution<int> dist_; // Random interval generator (500ms to 2000ms)
};

}  // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::FibonacciActionClient)

