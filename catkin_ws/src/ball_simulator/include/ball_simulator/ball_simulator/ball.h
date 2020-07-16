#ifndef BALL_SIMULATOR_BALL_H
#define BALL_SIMULATOR_BALL_H

#include <ball_simulator/BallSimulatorConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <string>

namespace ball_simulator
{
struct BallConfig
{
  BallConfig(ros::NodeHandle nh) : parent_frame_id("world"), frame_id("ball"), mass(1.0), radius(0.05), stiffness(15000)
  {
    nh.param("parent_frame_id", parent_frame_id, parent_frame_id);
    nh.param("ball_frame_id", frame_id, frame_id);
    nh.param("ball_mass", mass, mass);
    nh.param("radius", radius, radius);
    nh.param("stiffness", stiffness, stiffness);
  }

  void reconfigure(ball_simulator::BallSimulatorConfig& config)
  {
    if (config.ball_mass <= 0.0)
    {
      ROS_WARN("ball mass cannot be null or negative");
      config.ball_mass = mass;
    }
    mass = config.ball_mass;
    radius = config.radius;
  }

  std::string parent_frame_id;
  std::string frame_id;
  double mass;
  double radius;
  double stiffness;
};

struct BallState
{
  BallState()
  {
    reset();
  }

  bool reset()
  {
    y = 0.0;
    height = 1.0;
    velocity = 0.0;
    return true;
  }

  double y;
  double height;
  double velocity;
};

struct Ball
{
public:
  using Config = BallConfig;
  using State = BallState;

public:
  Ball(const Config& config) : config(config)
  {
  }

public:
  State state;
  const Config& config;
};

class BallPublisher
{
public:
  BallPublisher(ros::NodeHandle nh)
  {
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("ball_pose", 1);
    velocity_pub_ = nh.advertise<visualization_msgs::Marker>("ball_velocity_marker", 1, true);
  }

  static geometry_msgs::PoseStamped ballPose(const Ball::Config& config, ros::Time stamp, tf::Vector3 position,
                                             tf::Quaternion orientation)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = config.parent_frame_id;
    pose.pose.position.x = position[0];
    pose.pose.position.y = position[1];
    pose.pose.position.z = position[2];
    pose.pose.orientation.x = orientation[0];
    pose.pose.orientation.y = orientation[1];
    pose.pose.orientation.z = orientation[2];
    pose.pose.orientation.w = orientation[3];
    return pose;
  }

  static visualization_msgs::Marker marker(const Ball::Config& config)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = config.frame_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = config.radius * 2.0;
    marker.scale.y = config.radius * 2.0;
    marker.scale.z = config.radius * 2.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.frame_locked = true;
    return marker;
  }

  void publish(const Ball& ball, const ros::Time& stamp)
  {
    auto state = ball.state;
    auto config = ball.config;
    tf::Transform transform;
    tf::Vector3 pose(state.y, 0.0, state.height);
    transform.setOrigin(pose);
    tf::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(orientation);
    pose_pub_.publish(ballPose(config, stamp, pose, orientation));
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, stamp, config.parent_frame_id, config.frame_id));

    visualization_msgs::Marker marker;
    marker.header.frame_id = config.frame_id;
    marker.header.stamp = stamp;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.6;
    marker.scale.x = 0.02;
    marker.scale.y = 0.07;
    marker.scale.z = 0.09;
    geometry_msgs::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = state.velocity / 5.0;
    geometry_msgs::Point end;
    end.x = 0.0;
    end.y = 0.0;
    end.z = 0.0;
    marker.points.push_back(end);
    marker.points.push_back(start);
    velocity_pub_.publish(marker);
  }

private:
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;
};
}  // namespace ball_simulator

#endif
