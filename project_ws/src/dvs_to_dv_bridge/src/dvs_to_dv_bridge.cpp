#include <rclcpp/rclcpp.hpp>

#include <dvs_msgs/msg/event_array.hpp>
#include <dv_ros2_msgs/msg/event_array.hpp>

class DvsToDvBridge : public rclcpp::Node {
public:
    DvsToDvBridge() : Node("dvs_to_dv_bridge") {
        declare_parameter<std::string>("input_topic", "/dvs/events");
        declare_parameter<std::string>("output_topic", "/events");

        get_parameter("input_topic", input_topic_);
        get_parameter("output_topic", output_topic_);

        auto qos = rclcpp::SensorDataQoS();
        sub_ = create_subscription<dvs_msgs::msg::EventArray>(
            input_topic_, qos,
            std::bind(&DvsToDvBridge::callback, this, std::placeholders::_1));
        pub_ = create_publisher<dv_ros2_msgs::msg::EventArray>(output_topic_, 10);

        RCLCPP_INFO(get_logger(), "Bridging %s -> %s", input_topic_.c_str(), output_topic_.c_str());
    }

private:
    void callback(const dvs_msgs::msg::EventArray::SharedPtr msg) {
        dv_ros2_msgs::msg::EventArray out;
        out.header = msg->header;
        out.height = msg->height;
        out.width = msg->width;
        out.events.reserve(msg->events.size());

        for (const auto &evt : msg->events) {
            dv_ros2_msgs::msg::Event out_evt;
            out_evt.x = evt.x;
            out_evt.y = evt.y;
            out_evt.ts = evt.ts;
            out_evt.polarity = evt.polarity;
            out.events.push_back(out_evt);
        }

        pub_->publish(out);
    }

    std::string input_topic_;
    std::string output_topic_;
    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr sub_;
    rclcpp::Publisher<dv_ros2_msgs::msg::EventArray>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DvsToDvBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
