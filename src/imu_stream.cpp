
// Include libraries
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "tf/tfMessage.h"
#include "std_msgs/Header.h"
#include "std_msgs/UInt32.h"
#include <sstream>
#include "vn/ezasyncdata.h"
#include "vn/thread.h"
#include "tf/transform_listener.h"

using namespace std;
using namespace vn::math;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace vn::sensors;

// Main function
int main(int argc, char** argv) {
	
	// Initialize node parameter variables
	int publishing_buffer_size;
	int publishing_frequency;
	string SensorPort;
	int baudrate;
	uint32_t SensorBaudrate;
	bool force_flat;

	// Initialize ROS
	ros::init(argc, argv, "imu_stream");

	// Create node handler
	ros::NodeHandle nh;
	
	// Read in parameters
	if (!nh.getParam("/imu_stream/buffer_size", publishing_buffer_size)) publishing_buffer_size = 100;
	if (!nh.getParam("/imu_stream/frequency", publishing_frequency)) publishing_frequency = 40;
	if (!nh.getParam("/imu_stream/port", SensorPort)) SensorPort = "/dev/ttyUSB0";
	if (!nh.getParam("/imu_stream/baudrate", baudrate)) baudrate = 115200;
	if (!nh.getParam("/imu_stream/force_flat", force_flat)) force_flat = false;
	SensorBaudrate = (uint32_t) baudrate;

	// Print parameter values
	ROS_INFO("Buffer size: %i", publishing_buffer_size);
	ROS_INFO("Frequency: %i", publishing_frequency);
	ROS_INFO("Port: %s", SensorPort.c_str());
	ROS_INFO("Baudrate: %i", (int) SensorBaudrate);

	// Create publisher with specified message type and buffer size
	ros::Publisher imu_stream_pub = nh.advertise<sensor_msgs::Imu>("imu_stream", publishing_buffer_size);

	// Set frequency
	ros::Rate loop_rate(publishing_frequency);
	//ROS_INFO("ROS initialization complete");

	// Create and connect to a sensor
	EzAsyncData* ez = EzAsyncData::connect(SensorPort, SensorBaudrate);
	//ROS_INFO("Async connection complete");
	
	// Counter for number of loops - used in published message's header
	uint32_t count = 0;

	// Main loop
	while (ros::ok()) {
		// Create message object for publishing
		sensor_msgs::Imu msg;
		
		// Create string message objects for data read from IMU
		// These message are later assigned to the final output object
		geometry_msgs::Vector3 msg_acc;
		geometry_msgs::Quaternion msg_quat;
		geometry_msgs::Vector3 msg_ang_vel;

		// Read in the latest data processed by the IMU's  EzAsyncData class
		CompositeData cd = ez->currentData();

		// Check if there is any acceleration data available
		if (!cd.hasAcceleration()) {
			msg_acc.x = -1.0;
			msg_acc.y = -1.0;
			msg_acc.z = -1.0;

			if (count > 1) ROS_ERROR("No acceleration data available");
		} else {
			// If data is available, read in velocity
			vn::math::vec3 acc = cd.acceleration();
			
			// std::stringstream ss;
			// ss << "Acc: " << acc;
			// ROS_INFO("%s", ss.str().c_str());

			msg_acc.x = acc.x;
			msg_acc.y = acc.y;
			msg_acc.z = acc.z;
		}

		// Check if there is any rotation data available
		if (!cd.hasAnyAttitude()) {
			msg_quat.w = -1.0;
			msg_quat.x = -1.0;
			msg_quat.y = -1.0;
			msg_quat.z = -1.0;			

			if (count > 1) ROS_ERROR("No rotation data available");
		} else {
			// If data is available, read in attitude data
			vn::math::vec4 quat = cd.anyAttitude().quat();
			
			// std::stringstream ss;
			// ss << "Quat: " << quat;
			// ROS_INFO("%s", ss.str().c_str());

			// Hard-coded to be flat
			msg_quat.w = quat.w;
			if (force_flat) {
				msg_quat.x = 0;
				msg_quat.y = 0;
			} else {
				msg_quat.x = quat.x;
				msg_quat.y = quat.y;
			}
			msg_quat.z = quat.z;
		}

		// Check if there is any angular velocity/rate data available
		if (!cd.hasAngularRate()) {
			msg_ang_vel.x = -1.0;
			msg_ang_vel.y = -1.0;
			msg_ang_vel.z = -1.0;			

			if (count > 1) ROS_ERROR("No angular velocity data is available");
		} else {
			// If data is available, read in angular velocity
			vn::math::vec3 ang = cd.angularRate();
			
			// std::stringstream ss;
			// ss << "Ang: " << ang;
			// ROS_INFO("%s", ss.str().c_str());

			if (force_flat) {
				msg_ang_vel.x = 0;
				msg_ang_vel.y = 0;
			} else {
				msg_ang_vel.x = ang.x;
				msg_ang_vel.y = ang.y;
			}
			msg_ang_vel.z = ang.z;
		}

		// Assign data that was read-in to the main message object
		msg.linear_acceleration = msg_acc;
		msg.orientation = msg_quat;
		msg.angular_velocity = msg_ang_vel;

		// Set first element of covariance to be -1 for invalid/unavailable data
		msg.orientation_covariance[0] = 0;
		
		// Create message header
		std_msgs::Header header;
		header.frame_id = "imu_link";
		header.seq = count++;
		header.stamp = ros::Time::now();
		msg.header = header;

		// Publish message to topic
		imu_stream_pub.publish(msg);

		// Allow ros::Rate object to sleep to hit specified frequency
		loop_rate.sleep();
	}

	return 0;
}

