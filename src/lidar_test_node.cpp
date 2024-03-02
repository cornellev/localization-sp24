/**
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 * @author Ethan Uppal
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "follow_the_gap.h"

#define NODE_NAME "lidar_test_node"
#define LIDAR_TOPIC "/scan"

void on_lidar_scan(sensor_msgs::LaserScan scan) {
    int length = (scan.angle_max - scan.angle_min) / scan.angle_increment + 1;
    // float min_dist = INT_MAX;
    // for (int i = 0; i < length; i++) {
    //     if (scan.ranges[i] >= scan.range_min && scan.ranges[i] < min_dist) {
    //         min_dist = scan.ranges[i];
    //     }
    // }
    // ROS_INFO("min_dist=%f\n", min_dist);

    // 1147-(286+287)
    std::array<double, 574> range;
    for (int i = 0; i <= 574; i++) {
        range[i] = std::min(
            std::max((double)scan.ranges[i + 286], 
                (double)scan.range_min), 
            (double)scan.range_max);
    }
    double heading = follow_the_gap(range, 0.1);
    ROS_INFO("heading = %f\n", heading);
    // ROS_INFO("angle_max = %f\n", scan.angle_max);
    // ROS_INFO("angle_min = %f\n", scan.angle_min);
    // ROS_INFO("angle_increment = %f\n", scan.angle_increment);
    // ROS_INFO("length = %d\n", length);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle n("~");

    ros::Subscriber lidar_subscriber = n.subscribe(LIDAR_TOPIC, 1000, on_lidar_scan);
    if (lidar_subscriber) {
        ROS_INFO("made subscriber\n");
        while (n.ok()) {
            ros::spinOnce();
        }
    }

    return 0;
}
