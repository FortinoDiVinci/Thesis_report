#ifndef BALL_SIMULATOR_MESSAGE_AGGREGATOR_H
#define BALL_SIMULATOR_MESSAGE_AGGREGATOR_H

#include <ball_simulator/logger/log_entry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <string>

namespace ball_simulator
{
class MessageAggregator
{
public:
  MessageAggregator(const std::string& world_frame_id, const std::string& arm_frame_id,
                    const std::string& virtual_paddle_frame_id, const std::string& ball_frame_id);

  void onRealPaddlePose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void onVirtualPaddlePose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void onBallPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void onForceFeedback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  void onArmJointStates(const sensor_msgs::JointState::ConstPtr& msg);

  void logFrameHeightField(LogEntry& entry, const ros::Time& stamp, const std::string& frame_id);

  std::string produceLogEntry();

protected:
  template <typename T>
  void addField(std::stringstream& stream, const T& field);

  void addEmptyField(std::stringstream& stream);

private:
  geometry_msgs::PoseStamped::ConstPtr real_paddle_pose;
  geometry_msgs::PoseStamped::ConstPtr virtual_paddle_pose;
  geometry_msgs::PoseStamped::ConstPtr ball_pose;
  geometry_msgs::WrenchStamped::ConstPtr force_feedback;
  sensor_msgs::JointState::ConstPtr arm_joint_states;
  tf::TransformListener tf_listener;
  std::string world_frame_id;
  std::string arm_frame_id;
  std::string virtual_paddle_frame_id;
  std::string ball_frame_id;
};
}  // namespace ball_simulator

#endif