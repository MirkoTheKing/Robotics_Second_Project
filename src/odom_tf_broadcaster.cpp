#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OdomTFBroadcaster
{
public:
  OdomTFBroadcaster()
  {
    sub_ = nh_.subscribe("/odometry", 50, &OdomTFBroadcaster::callback, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber  sub_;
  tf2_ros::TransformBroadcaster tf_pub_;

  void callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp            = msg->header.stamp;
    tf.header.frame_id         = msg->header.frame_id;     // e.g. "odom"
    tf.child_frame_id          = msg->child_frame_id;      // e.g. "base_link"
    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation      = msg->pose.pose.orientation;
    tf_pub_.sendTransform(tf);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_tf_broadcaster");
  OdomTFBroadcaster node;
  ros::spin();
  return 0;
}
