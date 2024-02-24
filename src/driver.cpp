#include <mscl/mscl.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

float g = 9.81947;

sensor_msgs::Imu imu_msg;

tf2::Quaternion ned_to_enu(tf2::Quaternion q) {
    // I think this is correct, but I need to read on quaternion math to see if
    // I can do this properly instead of just guessing at how it's meant to work
    // by looking at Euler Angles.
    double yaw, pitch, roll;
    tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

    // https://answers.ros.org/question/336814/how-to-change-ned-to-enu/
    auto ret = tf2::Quaternion();
    ret.setRPY(roll, -pitch, M_PI_2 - yaw);

    return ret;
}

void processPacket(mscl::MipDataPacket packet) {
    imu_msg.header.frame_id = "gx5_45_link";

    imu_msg.linear_acceleration_covariance[0] = 1;
    imu_msg.linear_acceleration_covariance[4] = 1;
    imu_msg.linear_acceleration_covariance[8] = 1;

    imu_msg.angular_velocity_covariance[0] = 1;
    imu_msg.angular_velocity_covariance[4] = 1;
    imu_msg.angular_velocity_covariance[8] = 1;

    imu_msg.orientation_covariance[0] = 1;
    imu_msg.orientation_covariance[4] = 1;
    imu_msg.orientation_covariance[8] = 1;

    auto stamp = packet.collectedTimestamp();  // we don't have device time

    uint64_t leftover_nanos = stamp.nanoseconds()
                              - stamp.seconds() * (uint64_t)1e9;
    auto computed_time = ros::Time(stamp.seconds(), leftover_nanos);
    imu_msg.header.stamp = computed_time;

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
            // IMU gives negated z for some reason, inconsistent with the
            // right-hand rule
            imu_msg.linear_acceleration.z = -point.as_float() * g;
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
            auto q_sensor = point.as_Vector();
            auto q = tf2::Quaternion(q_sensor.as_floatAt(1),
                q_sensor.as_floatAt(2), q_sensor.as_floatAt(3),
                q_sensor.as_floatAt(0));  // Pretty sure the sensor outputs wxyz
            imu_msg.orientation = tf2::toMsg(ned_to_enu(q));
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "driver");

    ros::NodeHandle n("~");

    mscl::Connection connection;
    try {
        connection = mscl::Connection::Serial("/dev/ttyACM0");
    } catch (mscl::Error_InvalidSerialPort& e) {
        ROS_ERROR("Failed to open serial port. Is the IMU connected?");
        return 1;
    } catch (mscl::Error_Connection& e) {
        ROS_ERROR("Failed to connect to IMU. Do you have read/write access?");
        return 1;
    }

    mscl::InertialNode node(connection);

    ros::Duration ping_timeout = ros::Duration(5, 0);
    ros::Time start = ros::Time::now();
    ros::Rate ping_rate = ros::Rate(5);
    bool success = false;

    while (ros::Time::now() - start < ping_timeout) {
        success = node.ping();
        if (success) break;

        ping_rate.sleep();
    }

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

    ros::Rate loop_rate(100);
    auto publisher = n.advertise<sensor_msgs::Imu>("/imu/data", 100);

    while (ros::ok()) {
        mscl::MipDataPackets packets =
            node.getDataPackets(500);  // 500 ms timeout (might need to be more
                                       // like 10 to keep loop rate)

        for (mscl::MipDataPacket packet: packets) {
            processPacket(packet);
            publisher.publish(imu_msg);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}