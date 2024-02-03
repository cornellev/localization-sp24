#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <sstream>
#include <fstream>
#include <ros/console.h>

#define TEST_DATA_DIR                                                          \
    "/home/utku/Code/CEV/imu_ws/src/localization-sp24/test_data"
#define TEST_DATA_FILE TEST_DATA_DIR "/SensorConnectData_raw.csv"

sensor_msgs::Imu msg;

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_simulation_node");

    ros::NodeHandle n("~");

    std::string test_data_dir;
    assert(n.getParam("test_data_dir", test_data_dir));

    std::ifstream datafile(test_data_dir + "/utku.csv");

    ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("/car/lord_imu",
        1000);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        char comma;
        int ts;
        datafile >> ts;

        ROS_INFO("%d", ts);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
