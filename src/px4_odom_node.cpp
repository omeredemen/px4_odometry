#include <iostream>
#include <chrono>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "frame_transforms.hpp"
#include "px4_odom_node.hpp"

using namespace px4_ros_com::frame_transforms;

PX4_ODOM::PX4_ODOM(): Node("px4_odom_node")
{
    std::string vio_topic = "/odometry";
    std::string px4_pose_topic = "px4_odom/pose";

    this->declare_parameter<bool>("sensor_debug", false);
    this->declare_parameter<bool>("brodcast_tf", false);
    this->declare_parameter<std::string>("vio_topic", vio_topic);
    this->declare_parameter<std::string>("px4_pose_topic", px4_pose_topic);

    sensor_debug = this->get_parameter("sensor_debug").get_value<bool>();
    broadcast_tf = this->get_parameter("brodcast_tf").get_value<bool>();
    vio_topic = this->get_parameter("vio_topic").get_value<std::string>();
    px4_pose_topic = this->get_parameter("px4_pose_topic").get_value<std::string>();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    vio_sub = this->create_subscription<Odometry>(
        vio_topic, 1000,
        std::bind(&PX4_ODOM::VIOCallback,
                   this, std::placeholders::_1));

    px4_odom_sub = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        std::bind(&PX4_ODOM::VehicleOdomCallback,
                    this, std::placeholders::_1));

    px4_sensor_sub = this->create_subscription<SensorCombined>(
        "/fmu/out/sensor_combined", qos,
        std::bind(&PX4_ODOM::SensorCombinedCallback,
                    this, std::placeholders::_1));

    px4_vio_odom_pub = this->create_publisher<VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 1000);
    px4_pose_stamped_pub = this->create_publisher<PoseStamped>(px4_pose_topic, 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

/* This callback subscribes to px4 odometry data */
void PX4_ODOM::VehicleOdomCallback(VehicleOdometry::SharedPtr msg)
{
    Eigen::Vector3d pos_ned(
        msg->position[0],
        msg->position[1],
        msg->position[2]
    );

    Eigen::Quaterniond q_px4(
        msg->q[0],
        msg->q[1],
        msg->q[2],
        msg->q[3]
    );

    Eigen::Vector3d vel_ned(
        msg->velocity[0],
        msg->velocity[1],
        msg->velocity[2]
    );

    Eigen::Vector3d angular_vel_ned(
        msg->angular_velocity[0],
        msg->angular_velocity[1],
        msg->angular_velocity[2]
    );

    Eigen::Vector3d pos_enu = ned_to_enu_local_frame(pos_ned);
    Eigen::Quaterniond q_ros = px4_to_ros_orientation(q_px4);
    Eigen::Vector3d vel_enu = ned_to_enu_local_frame(vel_ned);
    Eigen::Vector3d angular_vel_enu = aircraft_to_baselink_body_frame(angular_vel_ned);

    PoseStamped px4_pose;
    px4_pose.pose.position.x = pos_enu.x();
    px4_pose.pose.position.y = pos_enu.y();
    px4_pose.pose.position.z = pos_enu.z();
    px4_pose.pose.orientation.w = q_ros.w();
    px4_pose.pose.orientation.x = q_ros.x();
    px4_pose.pose.orientation.y = q_ros.y();
    px4_pose.pose.orientation.z = q_ros.z();
    px4_pose_stamped_pub->publish(px4_pose);

    if(!broadcast_tf) {
        return;
    }
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "camera_link";
    t.child_frame_id = "";
    t.header.stamp = this->get_clock()->now();
    t.transform.rotation.w = q_ros.w();
    t.transform.rotation.x = q_ros.x();
    t.transform.rotation.y = q_ros.y();
    t.transform.rotation.z = q_ros.z();
    t.transform.translation.x = pos_enu.x();
    t.transform.translation.y = pos_enu.y();
    t.transform.translation.z = pos_enu.z();
    tf_broadcaster_->sendTransform(t);
}

/* This callback subscribes to px4 sensors data */
void PX4_ODOM::SensorCombinedCallback(SensorCombined::SharedPtr msg)
{
    if (!sensor_debug) { 
        return;
    }
    std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
    std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
    std::cout << "============================="   << std::endl;
    std::cout << "ts: "          << msg->timestamp    << std::endl;
    std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
    std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
    std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
    std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
    std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
    std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
    std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
    std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
    std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
}

/*
This callback subscribes to vio (openvins etc.)
PX4 uses NED and ROS uses ENU coordinate systems
So if we want to use VIO data on PX4 we need to convert NED.
*/
void PX4_ODOM::VIOCallback(Odometry::SharedPtr msg)
{
    Eigen::Vector3d pos_enu(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );

    Eigen::Quaterniond q_vio(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );

    Eigen::Vector3d vel_enu(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z
    );

    Eigen::Vector3d angular_vel_ros(
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z
    );

    Eigen::Vector3d pos_ned = enu_to_ned_local_frame(pos_enu);
    Eigen::Quaterniond q_px4 = ros_to_px4_orientation(q_vio);
    Eigen::Vector3d vel_ned = enu_to_ned_local_frame(vel_enu);
    Eigen::Vector3d angular_vel_px4 = baselink_to_aircraft_body_frame(angular_vel_ros);

    // Pose covariance
    Covariance3d cov_pos_ros = {
        msg->pose.covariance[0], msg->pose.covariance[1], msg->pose.covariance[2],
        msg->pose.covariance[6], msg->pose.covariance[7], msg->pose.covariance[8],
        msg->pose.covariance[12], msg->pose.covariance[13], msg->pose.covariance[14]
    };

    Covariance3d cov_rot_ros = {
        msg->pose.covariance[21], msg->pose.covariance[22], msg->pose.covariance[23],
        msg->pose.covariance[27], msg->pose.covariance[28], msg->pose.covariance[29],
        msg->pose.covariance[33], msg->pose.covariance[34], msg->pose.covariance[35]
    };

    // Velocity covariance
    Covariance3d cov_vel_ros = {
        msg->twist.covariance[0], msg->twist.covariance[1], msg->twist.covariance[2],
        msg->twist.covariance[6], msg->twist.covariance[7], msg->twist.covariance[8],
        msg->twist.covariance[12], msg->twist.covariance[13], msg->twist.covariance[14]
    };

    Covariance3d cov_pos_px4 = transform_static_frame(cov_pos_ros, StaticTF::ENU_TO_NED);
    Covariance3d cov_vel_px4 = transform_static_frame(cov_vel_ros, StaticTF::ENU_TO_NED);
    Covariance3d cov_rot_px4 = transform_frame(cov_rot_ros, q_px4);

    VehicleOdometry px4_odom_msg;
    px4_odom_msg.timestamp = static_cast<uint64_t>(msg->header.stamp.sec) * \
        1000000 + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000;
    px4_odom_msg.timestamp_sample = 0;

    px4_odom_msg.pose_frame = px4_odom_msg.POSE_FRAME_FRD;
    px4_odom_msg.velocity_frame = px4_odom_msg.POSE_FRAME_FRD;

    px4_odom_msg.position[0] = pos_ned.x();
    px4_odom_msg.position[1] = pos_ned.y();
    px4_odom_msg.position[2] = pos_ned.z();

    px4_odom_msg.velocity[0] = vel_ned.x();
    px4_odom_msg.velocity[1] = vel_ned.y();
    px4_odom_msg.velocity[2] = vel_ned.z();
    
    px4_odom_msg.q[0] = q_px4.w();
    px4_odom_msg.q[1] = q_px4.x();
    px4_odom_msg.q[2] = q_px4.y();
    px4_odom_msg.q[3] = q_px4.z();

    px4_odom_msg.angular_velocity[0] = angular_vel_px4.x();
    px4_odom_msg.angular_velocity[1] = angular_vel_px4.y();
    px4_odom_msg.angular_velocity[2] = angular_vel_px4.z();

    px4_odom_msg.position_variance[0] = cov_pos_px4[0];
    px4_odom_msg.position_variance[1] = cov_pos_px4[4];
    px4_odom_msg.position_variance[2] = cov_pos_px4[8];

    px4_odom_msg.velocity_variance[0] = cov_vel_px4[0];
    px4_odom_msg.velocity_variance[1] = cov_vel_px4[4];
    px4_odom_msg.velocity_variance[2] = cov_vel_px4[8];

    px4_odom_msg.orientation_variance[0] = cov_rot_px4[0];
    px4_odom_msg.orientation_variance[1] = cov_rot_px4[4];
    px4_odom_msg.orientation_variance[2] = cov_rot_px4[8];

    px4_vio_odom_pub->publish(px4_odom_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4_ODOM>());
    rclcpp::shutdown();
    return 0;
}