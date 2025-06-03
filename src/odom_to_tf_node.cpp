#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// This node subscribes to /odometry and broadcasts a TF "odom → base_link"
class OdomToTf
{
public:
  OdomToTf()
  {
    odom_sub_ = nh_.subscribe("/odometry", 10, &OdomToTf::odomCallback, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  tf::TransformBroadcaster tf_broadcaster_;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // Extract (x, y, orientation) from Odometry
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;  // usually zero in planar robots
    tf::Quaternion q;
    geometry_msgs::Quaternion geo_q = msg->pose.pose.orientation;
    tf::quaternionMsgToTF(geo_q, q);

    // Build a tf::Transform from odom → base_link
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(q);

    // Broadcast with the original timestamp
    tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform,
                            msg->header.stamp,
                            "odom",        // parent frame
                            "base_link")   // child frame
    );

    // Print to terminal for verification
    double yaw = tf::getYaw(q);
    ROS_INFO("Odom → TF: x=%.3f, y=%.3f, yaw=%.3f", x, y, yaw);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_tf_node");
  OdomToTf node;
  ros::spin();
  return 0;
}
