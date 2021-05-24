#ifndef BALL_SIMULATOR_TARGET_H
#define BALL_SIMULATOR_TARGET_H

#include <ball_simulator/BallSimulatorConfig.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

namespace ball_simulator
{
struct TargetConfig
{
  TargetConfig(ros::NodeHandle nh)
  {
    nh.param("parent_frame_id", parent_frame_id, parent_frame_id);
    nh.param("target_width", width, width);
    nh.param("target_thickness", thickness, thickness);
    nh.param("target_height", height, height);
  }

  void reconfigure(ball_simulator::BallSimulatorConfig& config)
  {
    height = config.target_height;
  }

  std::string parent_frame_id = "world";
  double width = 1.0;
  double thickness = 0.02;
  double height = 1.75;
};

class TargetPublisher
{
public:
  static visualization_msgs::Marker marker(const TargetConfig& target_config)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_config.parent_frame_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = target_config.height;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = target_config.width;
    marker.scale.y = target_config.width;
    marker.scale.z = target_config.thickness;
    marker.color.a = 0.7;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.frame_locked = true;
    return marker;
  }
};

struct PaddleLimitsConfig
{
  PaddleLimitsConfig(ros::NodeHandle nh)
  {
    nh.param("parent_frame_id", parent_frame_id, parent_frame_id);
    nh.param("paddle_lower_limit_height", l_height, l_height);
    nh.param("paddle_upper_limit_height", u_height, u_height);
  }

  std::string parent_frame_id = "world";
  double width = 1.0;
  double thickness = 0.02;
  //double l_height = -0.363;
  double l_height = -0.120;
  //double u_height = 0.345;
  double u_height = 0.120;
};


class PaddleLimitsPublisher
{
public:
  static visualization_msgs::Marker l_marker(const PaddleLimitsConfig& paddle_limits_config)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = paddle_limits_config.parent_frame_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = paddle_limits_config.l_height;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = paddle_limits_config.width;
    marker.scale.y = paddle_limits_config.width;
    marker.scale.z = paddle_limits_config.thickness;
    marker.color.a = 0.7;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.2;
    marker.frame_locked = true;
    return marker;
  }
    static visualization_msgs::Marker u_marker(const PaddleLimitsConfig& paddle_limits_config)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = paddle_limits_config.parent_frame_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = paddle_limits_config.u_height;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = paddle_limits_config.width;
    marker.scale.y = paddle_limits_config.width;
    marker.scale.z = paddle_limits_config.thickness;
    marker.color.a = 0.7;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.2;
    marker.frame_locked = true;
    return marker;
  }
};

}  // namespace ball_simulator

#endif
