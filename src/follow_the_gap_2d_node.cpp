/**
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 * @author Ethan Uppal
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "aux/follow_the_gap.h"
#include "aux/follow_the_gap.cpp"  // because catkin_make does not seem to work

#define NODE_NAME "follow_the_gap_2d"
#define LIDAR_TOPIC "/scan"

std::array<double, 574> range;

void on_lidar_scan(sensor_msgs::LaserScan scan) {
    // 1147-(286+287) get middle 180 deg of data
    for (int i = 0; i <= 574; i++) {
        range[i] = std::min(std::max((double)scan.ranges[i + 286],
                                (double)scan.range_min),
            (double)scan.range_max);
    }
    double heading = follow_the_gap(range, 0, 180.0);
    ROS_INFO("heading = %f\n", heading);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle n("~");

    ros::Subscriber lidar_subscriber = n.subscribe(LIDAR_TOPIC, 1000,
        on_lidar_scan);
    if (lidar_subscriber) {
        while (n.ok()) {
            ros::spinOnce();
        }
    } else {
        ROS_INFO("ERROR: could not make subscriber");
        return 1;
    }

    return 0;
}
