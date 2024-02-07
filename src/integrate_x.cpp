#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

double velocity = 0;
double x = 0;
ros::Time last_time;
bool first = true;

// low pass filter, madgwack, complimentary, ekf, moving averages, exponential
// moving averages
void integrate(const sensor_msgs::Imu::ConstPtr& msg) {
    if (first) {
        last_time = msg.get()->header.stamp;
        first = false;
        return;
    }

    ros::Duration delta_time = msg.get()->header.stamp - last_time;

    // https://www.reddit.com/r/gamedev/comments/5wb4hf/have_i_been_doing_second_degree_euler_integration/
    double accel = msg.get()->linear_acceleration.x;
    velocity = velocity + accel * delta_time.toSec();
    x = x + velocity * delta_time.toSec()
        + 0.5 * accel * delta_time.toSec() * delta_time.toSec();

    ROS_INFO("%f", x);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "integrator");

    ros::NodeHandle n("~");

    ROS_INFO("Start");

    ros::Subscriber sub = n.subscribe("/imu/data", 10, integrate);

    ros::spin();

    return 0;
}