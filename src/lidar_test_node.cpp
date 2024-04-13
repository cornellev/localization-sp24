/**
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 * @author Ethan Uppal
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>

#define NODE_NAME "lidar_test_node"
#define LIDAR_TOPIC "/scan"

bool has_initial = false;
sensor_msgs::LaserScan initial_scan;
sensor_msgs::LaserScan final_scan;

void dump_scan(std::ofstream& out, sensor_msgs::LaserScan& scan) {
    int length = (scan.angle_max - scan.angle_min) / scan.angle_increment;
    out << "angle_max = " << scan.angle_max << std::endl;
    out << "angle_min = " << scan.angle_min << std::endl;
    out << "angle_increment = " << scan.angle_increment << std::endl;
    out << "length = " << length << std::endl;
    out << "range_max = " << scan.range_max << std::endl;
    out << "range_min = " << scan.range_min << std::endl;
    for (int i = 0; i < length; i++) {
        out << i << " = " << scan.ranges[i] << std::endl;
    }
}

void on_lidar_scan(sensor_msgs::LaserScan scan) {
    if (has_initial) {
        final_scan = scan;
    } else {
        initial_scan = scan;
        has_initial = true;
    }
}

void ctrl_c_handler(int _) {
    (void)_;
    printf("exit\n");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);

    // std::signal(SIGINT, ctrl_c_handler);

    ros::NodeHandle n("~");
    std::string lib_dir;
    assert(n.getParam("lib_dir", lib_dir));

    ros::Subscriber lidar_subscriber = n.subscribe(LIDAR_TOPIC, 1000,
        on_lidar_scan);
    if (lidar_subscriber) {
        ROS_INFO("made subscriber\n");
        while (n.ok()) {
            ros::spinOnce();
        }
        std::ofstream out1(lib_dir + "/test_data/scan1.conf");
        std::ofstream out2(lib_dir + "/test_data/scan2.conf");
        dump_scan(out1, initial_scan);
        dump_scan(out2, final_scan);
    }

    return 0;
}
