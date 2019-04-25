#include <ball_simulator/logger/log_file.h>
#include <ball_simulator/logger/message_aggregator.h>
#include <std_srvs/Trigger.h>

using namespace ball_simulator;

class Logger
{
public:
  Logger(ros::NodeHandle nh)
    : start_logging_(nh.advertiseService("start_logging", &Logger::startLogging, this))
    , stop_logging_(nh.advertiseService("stop_logging", &Logger::stopLogging, this))
    , logging_started_(false)
  {
  }

  bool startLogging(ball_simulator::StartLogging::Request& request, ball_simulator::StartLogging::Response& response)
  {
    try
    {
      log_file_.open(request.log_file_path);
      response.success = true;
      logging_started_ = true;
    }
    catch (const std::ofstream::failure& e)
    {
      response.success = false;
      response.message = e.what();
      logging_started_ = false;
    }

    return true;
  }

  bool stopLogging(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
  {
    logging_started_ = false;
    return true;
  }

  void spin(ball_simulator::MessageAggregator& message_aggregator)
  {
    if (logging_started_)
    {
      log_file_.append(message_aggregator.produceLogEntry());
    }
  }

protected:
  LogFile log_file_;
  ros::ServiceServer start_logging_;
  ros::ServiceServer stop_logging_;
  bool logging_started_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "logger");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");

  double frequency = 10.0;
  std::string world_frame_id = "world";
  std::string arm_frame_id = "arm";
  std::string virtual_paddle_frame_id = "paddle";
  std::string ball_frame_id = "ball";
  pn.param("rate", frequency, frequency);
  pn.param("world_frame", world_frame_id, world_frame_id);
  pn.param("arm_frame", arm_frame_id, arm_frame_id);
  pn.param("virtual_paddle_frame", virtual_paddle_frame_id, virtual_paddle_frame_id);
  pn.param("ball_frame", ball_frame_id, ball_frame_id);

  MessageAggregator message_aggregator(world_frame_id, arm_frame_id, virtual_paddle_frame_id, ball_frame_id);

  ros::Subscriber real_paddle_pose_sub =
      nh.subscribe("real_paddle_pose", 1, &MessageAggregator::onRealPaddlePose, &message_aggregator);
  ros::Subscriber force_feedback_sub =
      nh.subscribe("force_feedback", 1, &MessageAggregator::onForceFeedback, &message_aggregator);
  ros::Subscriber arm_joint_states_sub =
      nh.subscribe("arm_joint_states", 1, &MessageAggregator::onArmJointStates, &message_aggregator);

  Logger logger(nh);

  if (!ros::Time::isValid())
  {
    ROS_WARN("waiting for valid time");
    ros::Time::waitForValid();
    ROS_WARN("time is now valid");
  }

  ros::Rate rate(frequency);

  while (nh.ok())
  {
    ros::spinOnce();
    logger.spin(message_aggregator);
    rate.sleep();
  }
}