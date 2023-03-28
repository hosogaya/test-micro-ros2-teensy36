#include <micro_ros_platformio.h>
#include <Arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#include <vector>
#include <memory>

// https://forum.pjrc.com/threads/71420-Undefined-reference-to-_write?mode=hybrid
extern "C" {
__attribute__((weak))
int _write(int file, char *ptr, int len)
{
	((class Print *)file)->write((uint8_t *)ptr, len);
	return len;
}
}

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray sending_msg;
std_msgs__msg__Float32MultiArray received_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
    while(1) {
      	delay(100);
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	sending_msg.layout.data_offset = 0;
	sending_msg.layout.dim.capacity = 2;
	sending_msg.layout.dim.size = 2;
	sending_msg.layout.dim.data = new std_msgs__msg__MultiArrayDimension[2];

	size_t ROW = 3, COLUMN=18;
	sending_msg.layout.dim.data[0].label.data = "row";
	sending_msg.layout.dim.data[0].label.size = 3;
	sending_msg.layout.dim.data[0].label.capacity = 10;
	sending_msg.layout.dim.data[0].size = ROW;
	sending_msg.layout.dim.data[0].stride = ROW*COLUMN;

	sending_msg.layout.dim.data[1].label.data = "column";
	sending_msg.layout.dim.data[1].label.size = 6;
	sending_msg.layout.dim.data[1].label.capacity = 10;
	sending_msg.layout.dim.data[1].size = COLUMN;
	sending_msg.layout.dim.data[1].stride = COLUMN;

	std::vector<float> vec(ROW*COLUMN,0);
	for (size_t i=0; i<ROW; ++i)
		for (size_t j=0; j<COLUMN; ++j)
			vec[i*COLUMN+j] = i*COLUMN+j;

	sending_msg.data.data = vec.data();
	sending_msg.data.size = vec.size();
	sending_msg.data.capacity = vec.capacity();

	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &sending_msg, NULL));
		// sending_msg.data++;
	}
	delete sending_msg.layout.dim.data;
}

void subscription_callback(const void* msgin) {
	const std_msgs__msg__Float32MultiArray* received_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
	sending_msg.layout = received_msg->layout;
	sending_msg.data = received_msg->data;
}

void setup() {
	// Configure serial transport
	Serial.begin(921600);
	set_microros_serial_transports(Serial);
	delay(2000);

	allocator = rcl_get_default_allocator();

	//create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"micro_ros_platformio_node_publisher"));

	// create timer,
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));
	
	// RCCHECK(rclc_subscription_init_default(
	// 	&subscriber, 
	// 	&node, 
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	// 	"micro_ros_platformio_node_subscriber"
	// ));

	// create executor
	// number_of_handle_sizeの数だけタイマーやサブスクライバーを登録することができる
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); 
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	// RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &received_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
	delay(100);
	RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}