#include <mscl/mscl.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

float g = 9.80665;

int main(int argc, char** argv) {
    ros::init(argc, argv, "driver");

    auto connection = mscl::Connection::Serial("/dev/ttyACM0");

    mscl::InertialNode node(connection);

    bool success = node.ping();
    if (!success) {
        ROS_ERROR("Failed to communicate with the IMU. Exiting.");
        return 1;
    }

    ROS_INFO("IMU Model Number: %s", node.modelNumber().c_str());

    // https://github.com/LORD-MicroStrain/MSCL/tree/master/MSCL_Examples/Inertial/C%2B%2B/MSCL_Inertial_Example_C%2B%2B/MSCL_Inertial_Example_C%2B%2B
    mscl::MipChannels ahrsImuChs;
    ahrsImuChs.push_back(
        mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC,
            mscl::SampleRate::Hertz(100)));
    ahrsImuChs.push_back(
        mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC,
            mscl::SampleRate::Hertz(100)));
    ahrsImuChs.push_back(
        mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_QUATERNION,
            mscl::SampleRate::Hertz(100)));

    node.setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, ahrsImuChs);

    ROS_INFO("Set active channel fields.");

    auto low_pass_config = mscl::LowPassFilterConfig{
        mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC,
        mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC,
        mscl::MipTypes::CH_FIELD_SENSOR_SCALED_MAG_VEC,
        mscl::MipTypes::CH_FIELD_SENSOR_SCALED_AMBIENT_PRESSURE,
    };

    for (auto& filter: low_pass_config) {
        filter.applyLowPassFilter = false;
    }

    node.setLowPassFilterSettings(low_pass_config);
    ROS_INFO("Set low pass filter settings.");

    node.enableDataStream(mscl::MipTypes::CLASS_AHRS_IMU);
    ROS_INFO("Enabled data streaming for AHRS/IMU.");

    ros::NodeHandle n("~");

    ros::Rate loop_rate(100);
    auto publisher = n.advertise<sensor_msgs::Imu>("/imu/data", 100);

    sensor_msgs::Imu imu_msg;

    while (ros::ok()) {
        mscl::MipDataPackets packets =
            node.getDataPackets(500);  // 500 ms timeout (might need to be more
                                       // like 10 to keep loop rate)

        for (mscl::MipDataPacket packet: packets) {
            imu_msg.header.stamp = ros::Time(0,
                packet.collectedTimestamp().nanoseconds());

            // get all of the points in the packet
            mscl::MipDataPoints points = packet.data();

            for (mscl::MipDataPoint point: points) {
                if (point.channelName() == "scaledAccelX") {
                    imu_msg.linear_acceleration.x = point.as_float() * g;
                }

                if (point.channelName() == "scaledAccelY") {
                    imu_msg.linear_acceleration.y = point.as_float() * g;
                }

                if (point.channelName() == "scaledAccelZ") {
                    imu_msg.linear_acceleration.z = point.as_float() * g;
                }

                if (point.channelName() == "scaledGyroX") {
                    imu_msg.angular_velocity.x = point.as_float();
                }

                if (point.channelName() == "scaledGyroY") {
                    imu_msg.angular_velocity.y = point.as_float();
                }

                if (point.channelName() == "scaledGyroZ") {
                    imu_msg.angular_velocity.z = point.as_float();
                }

                if (point.channelName() == "orientQuaternion") {
                    auto q = point.as_Vector();
                    imu_msg.orientation.x = q.as_floatAt(0);
                    imu_msg.orientation.y = q.as_floatAt(1);
                    imu_msg.orientation.z = q.as_floatAt(2);
                    imu_msg.orientation.w = q.as_floatAt(3);
                }
            }

            publisher.publish(imu_msg);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}