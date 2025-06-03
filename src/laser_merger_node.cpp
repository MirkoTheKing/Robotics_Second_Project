#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <limits>

class LaserMerger
{
public:
  LaserMerger()
    : tf_listener_(),
      front_sub_(nh_, "/scan_front", 10),
      back_sub_(nh_,  "/scan_back",  10),
      sync_(SyncPolicy(20), front_sub_, back_sub_)
  {
    sync_.registerCallback(
      boost::bind(&LaserMerger::scanCallback, this, _1, _2));
    merged_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_combined", 10);

    ROS_INFO("[laser_merger] ready: waiting for [/scan_front] & [/scan_back]...");
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher merged_pub_;
  tf::TransformListener tf_listener_;

  typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::LaserScan,
            sensor_msgs::LaserScan> SyncPolicy;

  message_filters::Subscriber<sensor_msgs::LaserScan> front_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> back_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;

  void scanCallback(const sensor_msgs::LaserScanConstPtr& front_msg,
                    const sensor_msgs::LaserScanConstPtr& back_msg)
  {
    // 1) Log incoming scan info
    ROS_INFO_STREAM("[laser_merger] front ts=" << front_msg->header.stamp.toSec()
                    << " beams=" << front_msg->ranges.size());
    ROS_INFO_STREAM("[laser_merger] back  ts=" << back_msg->header.stamp.toSec()
                    << " beams=" << back_msg->ranges.size());

    // 2) Prepare a 360° combined scan message
    sensor_msgs::LaserScan combined;
    combined.header.stamp    = ros::Time::now();
    combined.header.frame_id = "base_link";  // we'll project both into base_link

    combined.angle_min       = -M_PI;    // –180°
    combined.angle_max       = +M_PI;    // +180°
    combined.angle_increment = front_msg->angle_increment;   // assume same for front/back
    combined.time_increment  = 0.0;
    combined.scan_time       = front_msg->scan_time;

    // Determine range_min/max conservatively
    combined.range_min = std::max(front_msg->range_min, back_msg->range_min);
    combined.range_max = std::min(front_msg->range_max, back_msg->range_max);

    int total_beams = std::round((combined.angle_max - combined.angle_min)
                                 / combined.angle_increment);
    combined.ranges.assign(total_beams, std::numeric_limits<float>::infinity());
    combined.intensities.assign(total_beams, 0.0f);

    // 3) Helper: take a single scan, transform each beam end‐point into base_link,
    //    and insert valid ranges into 'combined'
    auto projectAndInsert = [&](const sensor_msgs::LaserScanConstPtr& scan_msg) {
      float angle = scan_msg->angle_min;
      for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float r = scan_msg->ranges[i];
        if (!std::isfinite(r)) {
          angle += scan_msg->angle_increment;
          continue;
        }
        // Compute point in laser frame
        float x_l = r * std::cos(angle);
        float y_l = r * std::sin(angle);
        geometry_msgs::PointStamped p_laser, p_base;
        p_laser.header = scan_msg->header;  // carries frame_id = laser frame
        p_laser.point.x = x_l;
        p_laser.point.y = y_l;
        p_laser.point.z = 0.0;

        // Transform into base_link
        try {
          tf_listener_.transformPoint("base_link", p_laser, p_base);
        }
        catch (tf::TransformException& ex) {
          ROS_WARN_THROTTLE(1.0, "[laser_merger] TF exception: %s", ex.what());
          angle += scan_msg->angle_increment;
          continue;
        }

        // Compute range and angle in base_link
        float xb = p_base.point.x;
        float yb = p_base.point.y;
        float rb = std::hypot(xb, yb);
        float angle_base = std::atan2(yb, xb);

        // Normalize to [–π, +π)
        while (angle_base < -M_PI) angle_base += 2.0 * M_PI;
        while (angle_base > +M_PI) angle_base -= 2.0 * M_PI;

        // Check if within robot footprint
        if (std::fabs(xb) < 0.27 && std::fabs(yb) < 0.20) {
          angle += scan_msg->angle_increment;
          continue;
        }

        // Determine index in combined.ranges
        int idx = std::round((angle_base - combined.angle_min)
                              / combined.angle_increment);
        if (idx < 0 || idx >= total_beams) {
          angle += scan_msg->angle_increment;
          continue;
        }

        // Keep only the minimum range if multiple beams hit same bin
        combined.ranges[idx] = std::min(combined.ranges[idx], rb);
        if (i < scan_msg->intensities.size()) {
          combined.intensities[idx] = scan_msg->intensities[i];
        }

        angle += scan_msg->angle_increment;
      }
    };

    // 4) Project both front and back
    projectAndInsert(front_msg);
    projectAndInsert(back_msg);

    // 5) Count valid beams for logging
    int valid_count = 0;
    for (float d : combined.ranges) {
      if (std::isfinite(d)) ++valid_count;
    }
    ROS_INFO_STREAM("[laser_merger] merged beams: " << valid_count << "/" << total_beams);

    // 6) Publish the merged scan
    merged_pub_.publish(combined);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_merger_node");
  LaserMerger node;
  ros::spin();
  return 0;
}
