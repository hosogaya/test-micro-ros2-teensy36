

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include <memory>
#include <string>
#include <functional>

// Using for bind callback function.
using std::placeholders::_1;

class SimpleListener :  public rclcpp::Node {
public:
    SimpleListener() : rclcpp::Node("simple_listener") {
        // Create subscriber which subscribe messages in "topic". 
        // When `subsciber_` subscribe message, a function `topic_callback` is called. 
        subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/micro_ros_platformio_node_publisher", // topic name
            rclcpp::QoS(1).best_effort(), // Quality of Service. The larger value is, the higher communication quality is among nodes, topics and services in exchange for communication speed.
            std::bind(&SimpleListener::topic_callback, this, _1) // Callback function
        );

        // time_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        //     "micro_ros_platformio_node_time_publisher", // topic name
        //     1, // Quality of Service. The larger value is, the higher communication quality is among nodes, topics and services in exchange for communication speed.
        //     std::bind(&SimpleListener::time_callback, this, _1) // Callback function
        // );
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // Print message to terminal.
        RCLCPP_INFO(this->get_logger(), "%f", msg->data[msg->data.size()-1]);
    }
    void time_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "%d", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr time_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleListener>());
  rclcpp::shutdown();
  return 0;
}