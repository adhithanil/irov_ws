// collect_client.cpp
//
// Minimal C++ action client that drives one sediment-collection goal, prints
// feedback and the result, and exits 0 only on success (non-zero on reject /
// abort / cancel). Used by the demo and the test script.
//
// Ctrl-C while a goal is running sends a cancel request (same as
// `ros2 action send_goal`) so the server safe-stows instead of continuing
// unattended; a second Ctrl-C force-exits immediately.
//
// Usage:  collect_client [target_mass_g] [max_duration_s]

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rov_sediment/action/collect_sediment.hpp"

using CollectSediment = rov_sediment::action::CollectSediment;
using GoalHandle = rclcpp_action::ClientGoalHandle<CollectSediment>;

namespace
{
std::atomic<int> g_sigint_count{0};

void on_sigint(int)
{
  // First Ctrl-C: let main() see it and cancel the goal. Second Ctrl-C:
  // give up waiting (e.g. server is unresponsive) and exit immediately.
  if (g_sigint_count.fetch_add(1) >= 1) {
    std::_Exit(130);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, on_sigint);
  // Install no rclcpp signal handler: we handle SIGINT ourselves so we can
  // cancel the in-flight goal instead of just tearing down the client.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto args = rclcpp::remove_ros_arguments(argc, argv);

  double target_mass_g = (args.size() > 1) ? std::stod(args[1]) : 50.0;
  double max_duration_s = (args.size() > 2) ? std::stod(args[2]) : 30.0;

  auto node = rclcpp::Node::make_shared("collect_client");
  auto client = rclcpp_action::create_client<CollectSediment>(node, "collect_sediment");

  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "action server not available");
    rclcpp::shutdown();
    return 2;
  }

  CollectSediment::Goal goal;
  goal.target_mass_g = target_mass_g;
  goal.max_duration_s = max_duration_s;

  auto send_opts = rclcpp_action::Client<CollectSediment>::SendGoalOptions();
  send_opts.feedback_callback =
    [node](GoalHandle::SharedPtr, const std::shared_ptr<const CollectSediment::Feedback> fb) {
      RCLCPP_INFO(node->get_logger(), "  [%-8s] %.1f g  (t=%.1fs)",
                  fb->phase.c_str(), fb->collected_mass_g, fb->elapsed_s);
    };

  RCLCPP_INFO(node->get_logger(), "sending goal: %.1f g, timeout %.1f s",
              target_mass_g, max_duration_s);

  auto goal_future = client->async_send_goal(goal, send_opts);
  if (rclcpp::spin_until_future_complete(node, goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "send goal failed");
    rclcpp::shutdown();
    return 2;
  }

  auto handle = goal_future.get();
  if (!handle) {
    RCLCPP_ERROR(node->get_logger(), "goal was REJECTED by the server");
    rclcpp::shutdown();
    return 1;
  }

  auto result_future = client->async_get_result(handle);

  // Poll instead of a single blocking spin so a SIGINT can be noticed and
  // turned into a cancel request while the sequence is still running.
  bool cancel_sent = false;
  rclcpp::FutureReturnCode rc_code;
  while (true) {
    rc_code = rclcpp::spin_until_future_complete(
      node, result_future, std::chrono::milliseconds(200));
    if (rc_code == rclcpp::FutureReturnCode::SUCCESS) {
      break;
    }
    if (rc_code == rclcpp::FutureReturnCode::INTERRUPTED) {
      RCLCPP_ERROR(node->get_logger(), "get result failed");
      rclcpp::shutdown();
      return 2;
    }
    if (g_sigint_count.load() > 0 && !cancel_sent) {
      cancel_sent = true;
      RCLCPP_WARN(node->get_logger(), "Ctrl-C: canceling goal so the hardware safe-stows...");
      auto cancel_future = client->async_cancel_goal(handle);
      rclcpp::spin_until_future_complete(node, cancel_future, std::chrono::seconds(2));
    }
  }

  auto wrapped = result_future.get();
  const auto & res = wrapped.result;
  int rc = 1;
  switch (wrapped.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node->get_logger(), "SUCCEEDED: %s (%.1f g)",
                  res->message.c_str(), res->collected_mass_g);
      rc = res->success ? 0 : 1;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node->get_logger(), "ABORTED: %s (%.1f g)",
                  res->message.c_str(), res->collected_mass_g);
      rc = 1;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node->get_logger(), "CANCELED: %s (%.1f g)",
                  res->message.c_str(), res->collected_mass_g);
      rc = 1;
      break;
    default:
      RCLCPP_ERROR(node->get_logger(), "unknown result code");
      rc = 2;
      break;
  }

  rclcpp::shutdown();
  return rc;
}
