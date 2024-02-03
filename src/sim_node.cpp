#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <sstream>
#include <fstream>
// #include "../../robot_localization/include/robot_localization/"

#define TEST_DATA_DIR "../test_data"
#define TEST_DATA_FILE TEST_DATA_DIR "/SensorConnectData_raw.csv"

sensor_msgs::Imu msg;

int main(int argc, char** argv) {
    std::ifstream datafile(TEST_DATA_DIR "/utku.csv");

    ros::init(argc, argv, "data_simulation_node");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("/car/lord_imu",
        1000);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        char comma;
        uint64_t timestamp;
        datafile
            >> timestamp;  // >> comma >> msg.orientation.x /* todo */ >> comma
                           //>> msg.linear_accele/ration.z;

        // chatter_pub.publish(msg);

        ROS_INFO("%zu\n", timestamp);
        return 0;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
