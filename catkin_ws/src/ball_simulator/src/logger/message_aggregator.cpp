#include <ball_simulator/logger/message_aggregator.h>

namespace ball_simulator {

MessageAggregator::MessageAggregator(
    const std::string& world_frame_id,
    const std::string& arm_frame_id,
    const std::string& virtual_paddle_frame_id,
    const std::string& ball_frame_id)
    : world_frame_id(world_frame_id),
      arm_frame_id(arm_frame_id),
      virtual_paddle_frame_id(virtual_paddle_frame_id),
      ball_frame_id(ball_frame_id)
{}

void MessageAggregator::onRealPaddlePose(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    real_paddle_pose = msg;
}

void MessageAggregator::onVirtualPaddlePose(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    virtual_paddle_pose = msg;
}

void MessageAggregator::onBallPose(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ball_pose = msg;
}

void MessageAggregator::onForceFeedback(
    const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    force_feedback = msg;
}

void MessageAggregator::onArmJointStates(
    const sensor_msgs::JointState::ConstPtr& msg)
{
    arm_joint_states = msg;
}

void MessageAggregator::logFrameHeightField(
    LogEntry& entry, const ros::Time& stamp, const std::string& frame_id)
{
    tf::StampedTransform transform;

    try
    {
        tf_listener.lookupTransform(frame_id, world_frame_id, stamp, transform);
        entry << transform.getOrigin().getZ();
    }
    catch (const tf::TransformException& e)
    {
        ROS_WARN_THROTTLE(
            2.0, "cannot compute %s coordinates: %s", frame_id.c_str(),
            e.what());
        entry << LogEntry::EmptyField();
    }
}

std::string MessageAggregator::produceLogEntry()
{
    LogEntry entry;
    ros::Time stamp = ros::Time::now() - ros::Duration(0.1);
    entry << stamp;

    logFrameHeightField(entry, stamp, arm_frame_id);
    logFrameHeightField(entry, stamp, virtual_paddle_frame_id);
    logFrameHeightField(entry, stamp, ball_frame_id);

    if (real_paddle_pose)
    {
        entry << real_paddle_pose->pose.position.z;
    }
    else
    {
        entry << LogEntry::EmptyField();
        ROS_WARN_THROTTLE(2.0, "no real paddle pose received yet");
    }

    if (force_feedback)
    {
        entry << force_feedback->wrench.force.z;
    }
    else
    {
        entry << LogEntry::EmptyField();
        ROS_WARN_THROTTLE(2.0, "no force feedback received yet");
    }

    if (arm_joint_states)
    {
        for (int i = 0; i < arm_joint_states->name.size(); ++i)
        {
            if (i < arm_joint_states->position.size())
            {
                entry << arm_joint_states->position[i];
            }
            else
            {
                entry << LogEntry::EmptyField();
            }
            if (i < arm_joint_states->velocity.size())
            {
                entry << arm_joint_states->velocity[i];
            }
            else
            {
                entry << LogEntry::EmptyField();
            }
            if (i < arm_joint_states->effort.size())
            {
                entry << arm_joint_states->effort[i];
            }
            else
            {
                entry << LogEntry::EmptyField();
            }
        }
    }
    else
    {
        ROS_WARN_THROTTLE(2.0, "no arm joint states received yet");
    }

    return entry.str();
}

template <typename T>
void MessageAggregator::addField(std::stringstream& stream, const T& field)
{
    stream << field << ", ";
}

void MessageAggregator::addEmptyField(std::stringstream& stream)
{
    stream << ", ";
}

} // namespace ball_simulator