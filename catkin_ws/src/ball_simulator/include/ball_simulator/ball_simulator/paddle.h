#ifndef BALL_SIMULATOR_PADDLE_H
#define BALL_SIMULATOR_PADDLE_H

#include <ball_simulator/BallSimulatorConfig.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <string>

namespace ball_simulator
{
struct PaddleConfig
{
  PaddleConfig(ros::NodeHandle nh)
    : parent_frame_id("world"), frame_id("paddle"), mass(1.0), stiffness(700), frequency(0.5), amplitude(0.2), scale(1.0), initial_height(0.32)
  {
    nh.param("parent_frame_id", parent_frame_id, parent_frame_id);
    nh.param("paddle_frame_id", frame_id, frame_id);
    nh.param("paddle_mass", mass, mass);
    nh.param("stiffness", stiffness, stiffness);
    nh.param("paddle_frequency", frequency, frequency);
    nh.param("paddle_amplitude", amplitude, amplitude);
    nh.param("scale", scale, scale);
    nh.param("h0", initial_height, initial_height);
  }

  void reconfigure(ball_simulator::BallSimulatorConfig& config)
  {
    mass = config.paddle_mass;
    frequency = config.paddle_frequency;
    amplitude = config.paddle_amplitude;
    scale = config.scale;
  }

  double getScale() 
    {
      return scale;
    }

  std::string parent_frame_id;
  std::string frame_id;
  double mass;
  double stiffness;
  double frequency;
  double amplitude;
  double scale;
  double initial_height; 
};

class Paddle
{
public:
  using Config = PaddleConfig;

public:
  Paddle(const Config& config) : config_(config), impulse_(0.0)
  {
  }

  virtual double height(double stamp) const = 0;

  virtual double velocity(double stamp) const = 0;

  const Config& config() const
  {
    return config_;
  }

  void setImpulse(double impulse)
  {
    impulse_ = impulse;
  }

  double impulse() const
  {
    return impulse_;
  }

private:
  const Config& config_;
  double impulse_;
};

class SinePaddle : public Paddle
{
public:
  SinePaddle(const Config& config) : Paddle(config), start_time_(ros::Time::now().toSec())
  {
  }

  double height(double stamp) const override
  {
    const double elapsed_time = stamp - start_time_;
    return config().amplitude * std::sin(elapsed_time * config().frequency * 2.0 * M_PI);
  }

  double velocity(double stamp) const override
  {
    const double elapsed_time = stamp - start_time_;
    return config().amplitude * config().frequency * 2.0 * M_PI *
           std::cos(elapsed_time * config().frequency * 2.0 * M_PI);
  }

private:
  double start_time_;
};

class RobotPaddle : public Paddle
{
public:
  RobotPaddle(const Config& config, ros::NodeHandle nh) : Paddle(config), frame_id_("arm")
  {
    double averaging_interval = 0.01;
    nh.param("arm_frame_id", frame_id_, frame_id_);
    nh.param("averaging_interval", averaging_interval, averaging_interval);
    averaging_interval_ = ros::Duration(averaging_interval);
  }

  double height(double stamp) const override
  {
    tf::StampedTransform transform;

    tf_listener_.waitForTransform(frame_id_, config().parent_frame_id, ros::Time(stamp), ros::Duration(0.03), ros::Duration(0.001));

    try
    {
      tf_listener_.lookupTransform(config().parent_frame_id, frame_id_, ros::Time(stamp), transform);
      return (transform.getOrigin().getZ() - config().initial_height) * config().scale;
    }
    catch (const tf::TransformException& e)
    {
      ROS_WARN_THROTTLE(0.2, "cannot compute %s coordinates: %s", frame_id_.c_str(), e.what());
      return 0.0;
    }
  }

  double velocity(double stamp) const override
  {
    geometry_msgs::Twist twist;
    double velocity = 0.0;
    std::string error_msg;

    tf_listener_.waitForTransform(config().parent_frame_id, frame_id_, ros::Time(stamp), ros::Duration(0.03), ros::Duration(0.001));

    try
    {
      lookup_twist(frame_id_, config().parent_frame_id, config().parent_frame_id, tf::Point(0, 0, 0), frame_id_,
                   ros::Time(stamp), ros::Duration(averaging_interval_), twist, &tf_listener_);
      return twist.linear.z * config().scale;
    }
    catch (const tf::TransformException& e)
    {
      ROS_WARN_THROTTLE(0.2, "cannot compute %s velocity: %s", frame_id_.c_str(), e.what());
    }

    return velocity;
  }

  void lookup_twist(const std::string& tracking_frame, const std::string& observation_frame,
                    const std::string& reference_frame, const tf::Point& reference_point,
                    const std::string& reference_point_frame, const ros::Time& time,
                    const ros::Duration& averaging_interval, geometry_msgs::Twist& twist,
                    const tf::TransformListener* listener) const
  {
    ros::Time latest_time, target_time;
    listener->getLatestCommonTime(observation_frame, tracking_frame, latest_time, NULL);

    if (ros::Time() == time)
      target_time = latest_time;
    else
      target_time = time;

    ros::Time end_time = std::min(target_time + averaging_interval * 0.5, latest_time);

    ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) -
                           averaging_interval;  // don't collide with zero
    ros::Duration corrected_averaging_interval =
        end_time - start_time;  // correct for the possiblity that start time was truncated above.
    tf::StampedTransform start, end;
    listener->lookupTransform(observation_frame, tracking_frame, start_time, start);
    listener->lookupTransform(observation_frame, tracking_frame, end_time, end);

    tf::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
    tf::Quaternion quat_temp;
    temp.getRotation(quat_temp);
    tf::Vector3 o = start.getBasis() * quat_temp.getAxis();
    tfScalar ang = quat_temp.getAngle();

    double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
    double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
    double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();

    tf::Vector3 twist_vel((delta_x) / corrected_averaging_interval.toSec(),
                          (delta_y) / corrected_averaging_interval.toSec(),
                          (delta_z) / corrected_averaging_interval.toSec());
    tf::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());

    twist.linear.x = twist_vel.x();
    twist.linear.y = twist_vel.y();
    twist.linear.z = twist_vel.z();
    twist.angular.x = twist_rot.x();
    twist.angular.y = twist_rot.y();
    twist.angular.z = twist_rot.z();
  }

private:
  tf::TransformListener tf_listener_;
  std::string frame_id_;
  ros::Duration averaging_interval_;
};

class PaddlePublisher
{
public:
  PaddlePublisher(ros::NodeHandle nh)
  {
    impulse_pub_ = nh.advertise<std_msgs::Float64>("impulse", 1);
    velocity_pub_ = nh.advertise<visualization_msgs::Marker>("paddle_velocity_marker", 1, true);
  }

  static visualization_msgs::Marker marker(const Paddle::Config& config, double width = 0.02)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = config.frame_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = -0.5 * width;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = width;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.frame_locked = true;
    return marker;
  }

  void publish(const Paddle& paddle, const ros::Time& stamp)
  {
    auto config = paddle.config();

    tf::Transform transform;
    tf::Vector3 pose(0.0, 0.0, paddle.height(stamp.toSec()));
    transform.setOrigin(pose);
    tf::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(orientation);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, stamp, config.parent_frame_id, config.frame_id));

    std_msgs::Float64 impulse_msg;
    double imp = paddle.impulse();
    if (imp != 0)
    {
        impulse_msg.data = imp;
        impulse_pub_.publish(impulse_msg);
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = config.frame_id;
    marker.header.stamp = stamp;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.6;
    marker.scale.x = 0.02;
    marker.scale.y = 0.07;
    marker.scale.z = 0.09;
    geometry_msgs::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = paddle.velocity(stamp.toSec()) / 5.0;
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
  ros::Publisher impulse_pub_;
  ros::Publisher velocity_pub_;
};

}  // namespace ball_simulator

#endif
