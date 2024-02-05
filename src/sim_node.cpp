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
#include <tf2/LinearMath/Quaternion.h>

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
#define time_now() std::chrono::steady_clock::now()

/* Having it at global scope zeros everything out. */
sensor_msgs::Imu msg;

/** A specific hardcoded log entry. As said before, this design is not very
 * modular. Might want to improve it later with actual CSV parsing so you know
 * what fiels are available and have the log entry represent all possible
 * fields. */
class LogEntry {
    using timestamp_t = uint64_t;
    friend class Simulator;

    timestamp_t timestamp;
    double yaw;
    double z_accel;
    tf2::Quaternion q;
    sensor_msgs::Imu imu;

    friend std::ifstream& operator>>(std::ifstream& in, LogEntry& entry);

    /** Sends an IMU data packet to the given publisher, which should be set to
     * publish `sensor_msgs::Imu`.
     *
     * @pre LogEntry::init() has been called on this instance. */
    void publish_to(ros::Publisher& publisher) {
        imu.header.stamp = ros::Time::now();
        publisher.publish(imu);
        // imu.ROS_INFO("TODO: LogEntry::publish_to not implemented");
    }

    /** Prepares the log entry for publishing. */
    void init() {
        q.setRPY(0, 0, yaw);

        imu.header.frame_id = "base_link";

        imu.linear_acceleration.z = z_accel;
        imu.linear_acceleration_covariance[0] = 1;
        imu.linear_acceleration_covariance[4] = 1;
        imu.linear_acceleration_covariance[8] = 1;

        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation.w = q.w();
        imu.orientation_covariance[0] = 1;
        imu.orientation_covariance[4] = 1;
        imu.orientation_covariance[8] = 1;
    }
};

class Simulator {
    std::vector<LogEntry> log;
    ros::Publisher publisher;

public:
    Simulator(ros::NodeHandle& n, std::ifstream& datafile) {
        // Parse the log entries.
        while (!datafile.eof()) {
            LogEntry entry;
            datafile >> entry;
            log.push_back(entry);
        }
        assert(!log.empty());

        // We publish to the Kalman filter.
        publisher = n.advertise<sensor_msgs::Imu>(OUTPUT_TOPIC, 1000);
    }

    void run() {
        size_t i = 0;
        auto start_time = time_now();
        while (ros::ok() && i < log.size()) {
            auto now = time_now();
            auto duration_since_start = (uint64_t)(now - start_time).count();
            if (duration_since_start >= log[i].timestamp - log[0].timestamp) {
                log[i].publish_to(publisher);
                i++;
            }
        }
    }
};

std::ifstream& operator>>(std::ifstream& in, LogEntry& entry) {
    char comma;
    in >> entry.timestamp >> comma >> entry.yaw >> comma >> entry.z_accel;
    entry.init();
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

    std::ifstream datafile(test_data_dir + "/" + TEST_DATA_FILE);
    Simulator simulator(n, datafile);
    simulator.run();

    return 0;
}
