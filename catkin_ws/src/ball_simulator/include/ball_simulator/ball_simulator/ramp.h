#ifndef BALL_SIMULATOR_RAMP_H
#define BALL_SIMULATOR_RAMP_H

#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

namespace ball_simulator
{
struct RampConfig
{
  RampConfig(ros::NodeHandle nh)
  {
    double ball_radius;
    nh.param("parent_frame_id", parent_frame_id, parent_frame_id);
    nh.param("ramp_length", length, length);
    nh.param("ramp_width", width, width);
    nh.param("ramp_thickness", thickness, thickness);
    nh.param("ramp_end_height", end_height, end_height);
    nh.param("ramp_slope_angle", slope_angle, slope_angle);
    nh.param("radius", ball_radius, ball_radius);
    end_offset = -ball_radius;
  }

  std::string parent_frame_id = "world";
  double length = 1.5;
  double width = 0.10;
  double thickness = 0.05;
  double end_height = 3.0;
  double slope_angle = 0.4;
  double end_offset = -0.05;
};

class RampPublisher
{
public:
  static visualization_msgs::Marker marker(const RampConfig& ramp_config)
  {
    tf::Quaternion orientation(tf::Vector3(0.0, 1.0, 0.0), ramp_config.slope_angle);

    visualization_msgs::Marker marker;
    marker.header.frame_id = ramp_config.parent_frame_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = -0.5 * ramp_config.length * cos(ramp_config.slope_angle) + ramp_config.end_offset;
    marker.pose.position.y = 0.5 * ramp_config.width;
    marker.pose.position.z =
        ramp_config.end_height + 0.5 * ramp_config.length * cos(0.5 * M_PI - ramp_config.slope_angle);
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    marker.scale.x = ramp_config.length;
    marker.scale.y = ramp_config.width;
    marker.scale.z = ramp_config.thickness;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.frame_locked = true;
    return marker;
  }
};
}  // namespace ball_simulator

#endif