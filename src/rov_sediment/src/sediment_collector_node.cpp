// sediment_collector_node.cpp
//
// Lifecycle node that runs the ROV sediment-collection sequence as a ROS 2
// action. Only the collection sequence is implemented here; navigation,
// localization and station keeping are assumed to be handled elsewhere and
// are out of scope.
//
// Fail-safes (all explicit):
//   1. Lifecycle gating  - goals are rejected unless the node is ACTIVE.
//   2. Single-goal lock  - only one collection may run at a time.
//   3. E-stop            - /rov/estop (Bool) aborts immediately + safe-stow.
//   4. Leak detection    - /rov/leak  (Bool) aborts immediately + safe-stow.
//   5. Operation timeout - goal.max_duration_s bounds the whole sequence.
//   6. Pump overcurrent  - /rov/pump_current (Float64) guarded during PUMPING.
//   7. Deactivate/cleanup - leaving ACTIVE aborts a running goal + safe-stow.
// Every abort/cancel path calls safe_stow() so the hardware ends up safe.

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rov_sediment/action/collect_sediment.hpp"

using CollectSediment = rov_sediment::action::CollectSediment;
using GoalHandle = rclcpp_action::ServerGoalHandle<CollectSediment>;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;
using std::placeholders::_2;

class SedimentCollectorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  SedimentCollectorNode()
  : rclcpp_lifecycle::LifecycleNode("sediment_collector")
  {
    // Tunables kept as parameters so nothing is hard-magic.
    pump_rate_g_s_    = this->declare_parameter<double>("pump_rate_g_s", 25.0);
    pump_current_lim_ = this->declare_parameter<double>("pump_current_limit", 10.0);
  }

  // ---- Lifecycle transitions ------------------------------------------------

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    reset_hardware_state();

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/rov/estop", 10,
      [this](const std_msgs::msg::Bool::SharedPtr m) { estop_.store(m->data); });
    leak_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/rov/leak", 10,
      [this](const std_msgs::msg::Bool::SharedPtr m) { leak_.store(m->data); });
    pump_current_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/rov/pump_current", 10,
      [this](const std_msgs::msg::Float64::SharedPtr m) { pump_current_ext_.store(m->data); });

    action_server_ = rclcpp_action::create_server<CollectSediment>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "collect_sediment",
      std::bind(&SedimentCollectorNode::handle_goal,    this, _1, _2),
      std::bind(&SedimentCollectorNode::handle_cancel,  this, _1),
      std::bind(&SedimentCollectorNode::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "configured: action 'collect_sediment' ready (node INACTIVE)");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    active_.store(true);
    RCLCPP_INFO(get_logger(), "ACTIVE: now accepting collection goals");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    // Leaving ACTIVE is itself a fail-safe: stop accepting goals and abort any
    // running sequence. The execute loop sees !active_ and safe-stows.
    active_.store(false);
    join_worker();
    RCLCPP_INFO(get_logger(), "INACTIVE: goals rejected");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    active_.store(false);
    join_worker();
    action_server_.reset();
    estop_sub_.reset();
    leak_sub_.reset();
    pump_current_sub_.reset();
    RCLCPP_INFO(get_logger(), "cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    active_.store(false);
    join_worker();
    safe_stow();
    return CallbackReturn::SUCCESS;
  }

  ~SedimentCollectorNode() override
  {
    active_.store(false);
    join_worker();
  }

private:
  // ---- Action callbacks -----------------------------------------------------

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const CollectSediment::Goal> goal)
  {
    if (!active_.load()) {
      RCLCPP_WARN(get_logger(), "goal REJECTED: node is not ACTIVE (lifecycle fail-safe)");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal_active_.load()) {
      RCLCPP_WARN(get_logger(), "goal REJECTED: a collection is already running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (estop_.load() || leak_.load()) {
      RCLCPP_WARN(get_logger(), "goal REJECTED: e-stop or leak currently asserted");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "goal ACCEPTED: target=%.1f g, timeout=%.1f s",
                goal->target_mass_g, goal->max_duration_s);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> gh)
  {
    join_worker();                 // previous goal is finished; reclaim the thread
    goal_active_.store(true);
    worker_ = std::thread([this, gh]() {
      run_sequence(gh);
      goal_active_.store(false);
    });
  }

  // ---- The sediment-collection sequence ------------------------------------

  void run_sequence(const std::shared_ptr<GoalHandle> gh)
  {
    const auto goal = gh->get_goal();
    const double target  = (goal->target_mass_g  > 0.0) ? goal->target_mass_g  : 50.0;
    const double timeout = (goal->max_duration_s > 0.0) ? goal->max_duration_s : 30.0;

    auto feedback = std::make_shared<CollectSediment::Feedback>();
    auto result   = std::make_shared<CollectSediment::Result>();

    reset_hardware_state();

    enum Phase { ARMING, LOWERING, PUMPING, SEALING, STOWING, DONE };
    Phase phase = ARMING;
    const char * names[] = {"ARMING", "LOWERING", "PUMPING", "SEALING", "STOWING", "DONE"};

    const double dt = 0.1;         // 10 Hz control/feedback loop
    double elapsed = 0.0;
    double phase_t = 0.0;
    rclcpp::Rate rate(1.0 / dt);

    auto fail = [&](const std::string & why) {
      RCLCPP_WARN(get_logger(), "FAIL-SAFE tripped: %s -> safe-stow + abort", why.c_str());
      safe_stow();
      result->success = false;
      result->collected_mass_g = collected_;
      result->message = why;
      gh->abort(result);
    };

    RCLCPP_INFO(get_logger(), "sequence start: collect %.1f g", target);

    while (rclcpp::ok()) {
      // ---- Fail-safe guards, evaluated every cycle in every phase ----
      if (!active_.load()) { fail("node left ACTIVE state during collection"); return; }
      if (estop_.load())   { fail("emergency stop asserted"); return; }
      if (leak_.load())    { fail("water ingress (leak) detected"); return; }
      if (elapsed > timeout) { fail("operation timeout exceeded"); return; }
      if (gh->is_canceling()) {
        RCLCPP_INFO(get_logger(), "operator cancel -> safe-stow");
        safe_stow();
        result->success = false;
        result->collected_mass_g = collected_;
        result->message = "canceled by operator";
        gh->canceled(result);
        return;
      }

      // ---- Sequence state machine ----
      switch (phase) {
        case ARMING:                                   // run self-check, power rails
          if (phase_t >= 1.0) { phase = LOWERING; phase_t = 0.0; }
          break;
        case LOWERING:                                 // deploy sampler to the seabed
          sampler_deployed_ = true;
          if (phase_t >= 3.0) { phase = PUMPING; phase_t = 0.0; pump_on_ = true; }
          break;
        case PUMPING:                                  // draw sediment into the chamber
          if (pump_current() > pump_current_lim_) { fail("pump overcurrent (jam)"); return; }
          collected_ += pump_rate_g_s_ * dt;
          if (collected_ >= target) {
            collected_ = target;
            pump_on_ = false;
            phase = SEALING; phase_t = 0.0;
          }
          break;
        case SEALING:                                  // close/seal the sample chamber
          if (phase_t >= 1.0) { phase = STOWING; phase_t = 0.0; }
          break;
        case STOWING:                                  // retract sampler to transit pose
          sampler_deployed_ = false;
          if (phase_t >= 2.0) { phase = DONE; }
          break;
        case DONE:
          break;
      }

      feedback->phase = names[phase];
      feedback->collected_mass_g = collected_;
      feedback->elapsed_s = elapsed;
      gh->publish_feedback(feedback);

      if (phase == DONE) {
        result->success = true;
        result->collected_mass_g = collected_;
        result->message = "sample collected, sealed and stowed";
        gh->succeed(result);
        RCLCPP_INFO(get_logger(), "collection COMPLETE: %.1f g stowed", collected_);
        return;
      }

      phase_t += dt;
      elapsed += dt;
      rate.sleep();
    }
  }

  // ---- Simulated hardware ---------------------------------------------------
  // Hardware is abstracted so this demo runs without a vehicle. In a real ROV
  // these would command thrusters/pump drivers and read real sensors.

  double pump_current() const
  {
    if (!pump_on_) return 0.0;
    double ext = pump_current_ext_.load();
    if (ext > 0.0) return ext;
    return pump_current_lim_ * 0.4;
  }

  void safe_stow()
  {
    pump_on_ = false;             // stop the pump
    sampler_deployed_ = false;    // retract the sampler to a safe transit pose
    RCLCPP_INFO(get_logger(), "safe-stow: pump OFF, sampler retracted");
  }

  void reset_hardware_state()
  {
    collected_ = 0.0;
    pump_on_ = false;
    sampler_deployed_ = false;
  }

  void join_worker()
  {
    if (worker_.joinable()) worker_.join();
  }

  // ---- Members --------------------------------------------------------------
  rclcpp_action::Server<CollectSediment>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr leak_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pump_current_sub_;

  std::thread worker_;
  std::atomic<bool> active_{false};       // true only in ACTIVE lifecycle state
  std::atomic<bool> goal_active_{false};  // single-goal lock
  std::atomic<bool> estop_{false};
  std::atomic<bool> leak_{false};
  std::atomic<double> pump_current_ext_{0.0};

  // simulated actuator / sample state
  double collected_{0.0};
  bool   pump_on_{false};
  bool   sampler_deployed_{false};

  // parameters
  double pump_rate_g_s_{25.0};
  double pump_current_lim_{10.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SedimentCollectorNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
