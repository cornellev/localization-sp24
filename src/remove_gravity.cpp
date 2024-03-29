#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

bool started;
sensor_msgs::Imu removed;
double g = 9.81947;

// low pass filter, madgwack, complimentary, ekf, moving averages, exponential
// moving averages
// https://stackoverflow.com/questions/18252692/gravity-compensation-in-accelerometer-data
void remove_gravity(const sensor_msgs::Imu::ConstPtr& msg) {
    removed = *msg;

    tf2::Quaternion q = tf2::Quaternion(msg->orientation.x, msg->orientation.y,
        msg->orientation.z, msg->orientation.w);
    tf2::Vector3 a = tf2::Vector3(msg->linear_acceleration.x,
        msg->linear_acceleration.y, msg->linear_acceleration.z);

    tf2::Vector3 rotated = quatRotate(q, a) - tf2::Vector3(0, 0, g);

    removed.linear_acceleration.x = rotated.x();
    removed.linear_acceleration.y = rotated.y();
    removed.linear_acceleration.z = rotated.z();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "remove_gravity");

    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe("/imu/data", 100, remove_gravity);
    ros::Publisher pub = n.advertise<sensor_msgs::Imu>("/imu/grav_removed", 1);

    ros::Rate rate = ros::Rate(100);

    while (ros::ok()) {
        pub.publish(removed);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}