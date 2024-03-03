/**
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 * @author Ethan Uppal
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define NODE_NAME "lidar_test_node"
#define LIDAR_TOPIC "/scan"

void on_lidar_scan(sensor_msgs::LaserScan scan) {
    int length = (scan.angle_max - scan.angle_min) / scan.angle_increment + 1;
    ROS_INFO("angle_max = %f\n", scan.angle_max);
    ROS_INFO("angle_min = %f\n", scan.angle_min);
    ROS_INFO("angle_increment = %f\n", scan.angle_increment);
    ROS_INFO("length = %d\n", length);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle n("~");

    ros::Subscriber lidar_subscriber = n.subscribe(LIDAR_TOPIC, 1000,
        on_lidar_scan);
    if (lidar_subscriber) {
        ROS_INFO("made subscriber\n");
        while (n.ok()) {
            ros::spinOnce();
        }
    }

    return 0;
}
