/**
 * @copyright Copyright (C) 2024 Ethan Uppal and Utku Melemetci. All rights
 * reserved.
 * @author Ethan Uppal
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <chrono>
#include <vector>
#include <fstream>
#include <ros/console.h>

/* Not sure if this should be the same as that in launch/sim.launch, but we're
 * keeping it consistent to be safe. */
#define NODE_NAME "data_simulation_node"

/* The name of the topic that the node publishes to, consistent with
 * params/sim.yaml. */
#define OUTPUT_TOPIC "/car/lord_imu"

/* Location of non-header CSV file. The code will be hardcoded to parse the
 * format of that file, which is not very modular. */
#define TEST_DATA_FILE "SensorConnectData_raw.csv"

/* LOL. */
#define now() std::chrono::steady_clock::now()

/* Having it at global scope zeros everything out. */
sensor_msgs::Imu msg;

/** A specific hardcoded log entry. As said before, this design is not very
 * modular. Might want to improve it later with actual CSV parsing so you know
 * what fiels are available and have the log entry represent all possible
 * fields. */
struct LogEntry {
    using timestamp_t = uint64_t;

    timestamp_t timestamp;
    double yaw;
    double z_accel;

    /** Sends an IMU data packet to the given publisher, which should be set to
     * publish `sensor_msgs::Imu`. */
    void publish_to(ros::Publisher& publisher) {
        ROS_INFO("TODO: LogEntry::publish_to not implemented");
    }
};

std::ifstream& operator>>(std::ifstream& in, LogEntry& entry) {
    char comma;
    in >> entry.timestamp >> comma >> entry.yaw >> comma >> entry.z_accel;
    return in;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);

    // Passing "~" allows relative parameters to work.
    ros::NodeHandle n("~");

    // We should always have test_data_dir passed in a launch file to run this
    // node.
    std::string test_data_dir;
    assert(n.getParam("test_data_dir", test_data_dir));

    // Parse the log entries.
    std::ifstream datafile(test_data_dir + "/" + TEST_DATA_FILE);
    std::vector<LogEntry> log;
    while (!datafile.eof()) {
        LogEntry entry;
        datafile >> entry;
        log.push_back(entry);
    }
    assert(!log.empty());

    // We publish to the Kalman filter.
    ros::Publisher publisher = n.advertise<sensor_msgs::Imu>(OUTPUT_TOPIC,
        1000);

    // Run the simulation.
    size_t i = 0;
    auto start_time = now();
    while (ros::ok() && i < log.size()) {
        auto now = now();
        auto duration_since_start = (uint64_t)(now - start_time).count();
        if (duration_since_start >= log[i].timestamp - log[0].timestamp) {
            log[i].publish_to(publisher);
            i++;
        }
    }

    return 0;
}
