#include <iostream>
#include "adis_driver/adis_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>

sensor_msgs::Imu GenerateRosIMUMsg(IMU_Data &imu_data)
{
	// std::cout << "Convert IMU data to ROS IMU msg!" << std::endl;

	sensor_msgs::Imu imu_msg;
	imu_msg.angular_velocity.x = imu_data.gyro_x;
	imu_msg.angular_velocity.y = imu_data.gyro_y;
	imu_msg.angular_velocity.z = imu_data.gyro_z;

	imu_msg.linear_acceleration.x = imu_data.acc_x;
	imu_msg.linear_acceleration.y = imu_data.acc_y;
	imu_msg.linear_acceleration.z = imu_data.acc_z;

	// std::cout << "Convert IMU data to ROS IMU msg Done!" << std::endl;

	return imu_msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "adis_driver");

	std::string pkg_path = ros::package::getPath("adis_driver");

	std::cout << "Current Package Path: " << std::endl << pkg_path << std::endl;

	AdisDriver adis_driver(pkg_path);

	IMU_Data imu_data;

	ros::NodeHandle nh;

	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 100);


	ros::Time start_ts = ros::Time::now();

	while(ros::ok())
	{
		if(adis_driver.ReceiveData(imu_data) == true)
		{
			sensor_msgs::Imu imu_msgs = GenerateRosIMUMsg(imu_data);

			// This should be give by the mcu, TO change its
			imu_msgs.header.stamp = start_ts + ros::Duration(imu_data.time);

			imu_pub.publish(imu_msgs);
		}

	}

	return 0;
}