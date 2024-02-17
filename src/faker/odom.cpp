#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <random>

int main(int argc, char** argv) {
    ros::init(argc, argv, "faker");

    ros::NodeHandle n("~");

    ros::Rate loop_rate(100);
    auto publisher = n.advertise<nav_msgs::Odometry>("/odom/data", 100);

    double mean = 0;
    double stddev = 0.1;
    std::default_random_engine generator(201);
    std::normal_distribution<double> distribution(mean, stddev);
    auto get_random = [&]() { return distribution(generator); };

    double variance = 1;

    while (ros::ok()) {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";

        odom_msg.pose.pose.position.x = get_random();
        odom_msg.pose.pose.position.y = get_random();

        odom_msg.pose.covariance[0] = variance;
        odom_msg.pose.covariance[7] = variance;
        odom_msg.pose.covariance[14] = variance;

        publisher.publish(odom_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
