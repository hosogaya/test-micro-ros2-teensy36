#include <micro_ros_platformio.h>
#include <Arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/type_utilities.h>

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

static micro_ros_utilities_memory_conf_t conf = {0};

std::string row = "row";
std::string col = "column";

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_publisher_t time_publisher;
std_msgs__msg__Int32 time_msg;
std_msgs__msg__Int32 test_msg;
std_msgs__msg__Int32 test_subsc_msg;
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
	if (timer != NULL) {
		unsigned int s = micros();
		RCSOFTCHECK(rcl_publish(&publisher, &sending_msg, NULL));
		// sending_msg.data++;
		time_msg.data = micros() - s;
		RCSOFTCHECK(rcl_publish(&time_publisher, &time_msg, NULL));
	}
	// delete sending_msg.layout.dim.data;
}

void subscription_callback(const void* msgin) {
	// const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray *)msgin;
	// test_msg.data++;

	const std_msgs__msg__Float32MultiArray* received_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
	sending_msg.data = received_msg->data;
	sending_msg.layout = received_msg->layout;

	// delete sending_msg.layout.dim.data;
	// delete sending_msg.data.data;
	// sending_msg.layout.data_offset = received_msg->layout.data_offset;
	// sending_msg.layout.dim.capacity = received_msg->layout.dim.capacity;
	// sending_msg.layout.dim.size = received_msg->layout.dim.size;
	// sending_msg.layout.dim.data = new std_msgs__msg__MultiArrayDimension[received_msg->layout.dim.size];
	// for (size_t i=0; i<sending_msg.layout.dim.size; ++i) {
	// 	sending_msg.layout.dim.data[i].size = received_msg->layout.dim.data[i].size;
	// 	sending_msg.layout.dim.data[i].stride = received_msg->layout.dim.data[i].stride;
	// 	sending_msg.layout.dim.data[i].label.capacity = received_msg->layout.dim.data[i].label.capacity;
	// 	sending_msg.layout.dim.data[i].label.size = received_msg->layout.dim.data[i].label.size;
	// 	sending_msg.layout.dim.data[i].label.data = new char[received_msg->layout.dim.data[i].label.size];
	// 	for (size_t j=0; j<received_msg->layout.dim.data[i].size; ++j) {
	// 		sending_msg.layout.dim.data[i].label.data[j] = received_msg->layout.dim.data[i].label.data[j];
	// 	}
	// }
	// sending_msg.data.size = received_msg->data.size;
	// sending_msg.data.capacity = received_msg->data.capacity;
	// sending_msg.data.data = new float[sending_msg.data.size];
	// for (size_t i=0; i<sending_msg.data.size; ++i) {
	// 	sending_msg.data.data[i] = received_msg->data.data[i];
	// }
}

void setup() {
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

	sending_msg.data.size = 54;
	sending_msg.data.capacity = 54;
	sending_msg.data.data = new float[54];
	for (size_t i=0; i<ROW; ++i)
		for (size_t j=0; j<COLUMN; ++j)
			sending_msg.data.data[i*COLUMN+j] = i*COLUMN+j;

	// https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/
	conf.max_string_capacity = 54;
	conf.max_basic_type_sequence_capacity = 108;
	conf.max_ros2_type_sequence_capacity = 108;
	bool success = micro_ros_utilities_create_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		&received_msg,
		conf 
	);

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
	RCCHECK(rclc_publisher_init_best_effort(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		// ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"micro_ros_platformio_node_publisher"));

	RCCHECK(rclc_publisher_init_best_effort(
		&time_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"micro_ros_platformio_node_time_publisher"));

	// create timer,
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));
	
	RCCHECK(rclc_subscription_init_best_effort(
		&subscriber, 
		&node, 
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		// ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"micro_ros_platformio_node_subscriber"
	));
	

	// create executor
	// number_of_handle_sizeの数だけタイマーやサブスクライバーを登録することができる
	RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator)); 
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &received_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
	// delay(100);
	RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}