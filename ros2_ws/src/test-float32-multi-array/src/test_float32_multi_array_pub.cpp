#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <memory>
#include <string>
#include <functional>

// To simplify timer definition
using namespace std::chrono_literals;

class SimpleTalker : public rclcpp::Node {
public:
    SimpleTalker() : rclcpp::Node("simple_float32_multi_array_talker"), count_(0) {
       // Create topic and publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "micro_ros_platformio_node_subscriber", // topic name
             rclcpp::QoS(1).best_effort()); // Quality of Service. The larger value is, the higher communication quality is among nodes, topics and services in exchange for communication speed.
        // call function 'timer_callback' every 500ms.
        timer_ = this->create_wall_timer(
           10ms, std::bind(&SimpleTalker::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        // Create message
        size_t row = 6, column = 18;
        auto message = std_msgs::msg::Float32MultiArray();
        message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        message.layout.dim[0].label = "row";
        message.layout.dim[0].size = row;
        message.layout.dim[0].stride = row*column;
        message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        message.layout.dim[1].label = "column";
        message.layout.dim[1].size = column;
        message.layout.dim[1].stride = column;
        message.layout.data_offset = 0;

        message.data.resize(message.layout.dim[0].size*message.layout.dim[1].size);
        for (size_t i=0; i<message.layout.dim[0].size; ++i) {
            for (size_t j=0; j<message.layout.dim[1].size; ++j) {
                message.data[i*column+j] = i*column+j+count_; 
            }
        }

        // print message to terminal
        RCLCPP_INFO(this->get_logger(), "Publish");

        // publish message
        publisher_->publish(message);
        count_++;
    }
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTalker>());
    rclcpp::shutdown();

    return 0;
}