#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <random>

float g = 9.81947;

int main(int argc, char** argv) {
    ros::init(argc, argv, "faker");

    ros::NodeHandle n("~");

    ros::Rate loop_rate(100);
    auto publisher = n.advertise<sensor_msgs::Imu>("/imu/data", 100);

    double mean = 0.0;
    double stddev = 0.1;
    std::default_random_engine generator(400);
    std::normal_distribution<double> distribution(mean, stddev);
    auto get_random = [&]() { return distribution(generator); };

    double variance = 1;

    while (ros::ok()) {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "base_link";

        imu_msg.linear_acceleration.x = get_random();
        imu_msg.linear_acceleration.y = get_random();
        imu_msg.linear_acceleration.z = g;

        imu_msg.linear_acceleration_covariance[0] = variance;
        imu_msg.linear_acceleration_covariance[4] = variance;
        imu_msg.linear_acceleration_covariance[8] = variance;

        imu_msg.angular_velocity.x = 0;
        imu_msg.angular_velocity.y = 0;
        imu_msg.angular_velocity.z = 0;

        imu_msg.angular_velocity_covariance[0] = variance;
        imu_msg.angular_velocity_covariance[4] = variance;
        imu_msg.angular_velocity_covariance[8] = variance;

        imu_msg.orientation.x = 0;
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
        imu_msg.orientation.w = 1;

        imu_msg.orientation_covariance[0] = variance;
        imu_msg.orientation_covariance[4] = variance;
        imu_msg.orientation_covariance[8] = variance;

        publisher.publish(imu_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}