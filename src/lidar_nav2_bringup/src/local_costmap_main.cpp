#include <csignal>
#include <chrono>
#include <memory>
#include <thread>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{

// 清理插件订阅的弱引用后再释放插件库，保持 Nav2 地图计算逻辑不变
class ManagedCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:

  ManagedCostmap(pluginlib::ClassLoader<nav2_costmap_2d::Layer> & libraries) : Costmap2DROS(rclcpp::NodeOptions()), libraries_(libraries)
  {
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    try
    {
      getParameters();
      for (const auto & type : plugin_types_)
      {
        libraries_.loadLibraryForClass(type);
      }
      for (const auto & type : filter_types_)
      {
        libraries_.loadLibraryForClass(type);
      }
    }
    catch (const std::exception & error)
    {
      RCLCPP_ERROR(get_logger(), "Plugin lifetime setup failed: %s", error.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
    return Costmap2DROS::on_configure(state);
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    const auto result = Costmap2DROS::on_cleanup(state);
    executor_.reset();
    callback_group_.reset();
    return result;
  }

private:
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> & libraries_;
};

volatile std::sig_atomic_t stopping = 0;

// 主线程在 ROS 上下文仍有效时停止生命周期，信号处理器只设置标志
void request_stop(int)
{
  stopping = 1;
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> libraries("nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto costmap = std::make_shared<ManagedCostmap>(libraries);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(costmap->get_node_base_interface());
  while (rclcpp::ok() && !stopping)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  using lifecycle_msgs::msg::State;
  using lifecycle_msgs::msg::Transition;
  if (rclcpp::ok())
  {
    if (costmap->get_current_state().id() == State::PRIMARY_STATE_ACTIVE)
    {
      costmap->trigger_transition(Transition::TRANSITION_DEACTIVATE);
    }
    if (costmap->get_current_state().id() == State::PRIMARY_STATE_INACTIVE)
    {
      costmap->trigger_transition(Transition::TRANSITION_CLEANUP);
    }
  }
  executor.remove_node(costmap->get_node_base_interface());
  costmap.reset();
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
  return 0;
}
