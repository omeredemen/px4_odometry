#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"

using namespace px4_msgs::msg;
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;

class PX4_ODOM: public rclcpp::Node
{
public:
    PX4_ODOM();

private:
    rclcpp::Subscription<Odometry>::SharedPtr vio_sub;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr px4_odom_sub;
    rclcpp::Subscription<SensorCombined>::SharedPtr px4_sensor_sub;

    rclcpp::Publisher<VehicleOdometry>::SharedPtr px4_vio_odom_pub;
    rclcpp::Publisher<PoseStamped>::SharedPtr px4_pose_stamped_pub;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void VehicleOdomCallback(VehicleOdometry::SharedPtr msg);
    void SensorCombinedCallback(SensorCombined::SharedPtr msg);
    void VIOCallback(Odometry::SharedPtr msg);

    bool sensor_debug = false;
    bool broadcast_tf = false;
};