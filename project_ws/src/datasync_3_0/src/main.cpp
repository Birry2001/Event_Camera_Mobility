#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "datasync_3_0/event_visualizer.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<EventVisualizer>();

    // Executeur multi-thread (IMU + events simultanes)
    rclcpp::executors::MultiThreadedExecutor exec(
        rclcpp::ExecutorOptions(), 3);

    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
