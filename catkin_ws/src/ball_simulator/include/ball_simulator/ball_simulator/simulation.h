#ifndef BALL_SIMULATOR_SIMULATION_H
#define BALL_SIMULATOR_SIMULATION_H

#include <ball_simulator/BallSimulatorConfig.h>
#include <ball_simulator/ball_simulator/ball.h>
#include <ball_simulator/ball_simulator/paddle.h>
#include <ball_simulator/ball_simulator/ramp.h>
#include <ball_simulator/ball_simulator/target.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

namespace ball_simulator
{
struct SimulationConfig
{
  SimulationConfig(ros::NodeHandle nh) : restitution(0.6), gravity(9.81), enable_sine_paddle(false)
  {
    nh.param("restitution", restitution, restitution);
    nh.param("gravity", gravity, gravity);
    nh.param("enable_sine_paddle", enable_sine_paddle, enable_sine_paddle);
  }

  void reconfigure(ball_simulator::BallSimulatorConfig& config)
  {
    restitution = config.restitution;
    gravity = config.gravity;
    enable_sine_paddle = config.enable_sine_paddle;
  }

  double restitution;
  double gravity;
  bool enable_sine_paddle;
};

struct SimulationState
{
public:
  SimulationState()
  {
    reset();
  }

  bool reset()
  {
    first_step = true;
    collision_height = 1.0;
    bounce_velocity = 0.0;
    contact_at_last_step = false;
    stopped = false;
    collision_time = 0.0;
    rolling = true;
    start_time = ros::Time::now();
    return true;
  }

  bool stop()
  {
    stopped = true;
    return true;
  }

public:
  bool first_step;
  double collision_height;
  double bounce_velocity;
  bool contact_at_last_step;
  bool stopped;
  double collision_time;
  bool rolling;
  ros::Time start_time;
};

struct Simulation
{
public:
  using Config = SimulationConfig;
  using State = SimulationState;

public:
  Simulation(const Config& config) : config(config)
  {
  }

public:
  State state;
  const Config& config;
};

class SimulationPublisher
{
public:
  SimulationPublisher(ros::NodeHandle nh) : ball_publisher(nh), paddle_publisher(nh)
  {
  }

  void publish(const Ball& ball, const Paddle& paddle, const ros::Time& stamp)
  {
    ball_publisher.publish(ball, stamp);
    paddle_publisher.publish(paddle, stamp);
  }

private:
  BallPublisher ball_publisher;
  PaddlePublisher paddle_publisher;
};

class SimulationReconfiguration
{
public:
  SimulationReconfiguration(ros::NodeHandle nh)
    : simulation_config(nh), ball_config(nh), paddle_config(nh), ramp_config(nh), target_config(nh)
  {
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1, true);

    dynamic_reconfigure::Server<ball_simulator::BallSimulatorConfig>::CallbackType reconfigure_callback =
        boost::bind(&SimulationReconfiguration::reconfigure, this, _1, _2);
    reconfiguration_server.setCallback(reconfigure_callback);
  }

  void reconfigure(ball_simulator::BallSimulatorConfig& config, uint32_t level)
  {
    simulation_config.reconfigure(config);
    ball_config.reconfigure(config);
    paddle_config.reconfigure(config);
    target_config.reconfigure(config);

    marker_pub.publish(markers());
  }

  const Simulation::Config& simulationConfig() const
  {
    return simulation_config;
  }

  const Ball::Config& ballConfig() const
  {
    return ball_config;
  }

  const Paddle::Config& paddleConfig() const
  {
    return paddle_config;
  }

  const RampConfig& rampConfig() const
  {
    return ramp_config;
  }

  visualization_msgs::MarkerArray markers() const
  {
    int id = 0;
    visualization_msgs::MarkerArray markers;
    auto ball_marker = BallPublisher::marker(ball_config);
    ball_marker.id = id++;
    markers.markers.push_back(ball_marker);
    auto paddle_marker = PaddlePublisher::marker(paddle_config);
    paddle_marker.id = id++;
    markers.markers.push_back(paddle_marker);
    visualization_msgs::Marker ramp_marker = RampPublisher::marker(ramp_config);
    ramp_marker.id = id++;
    markers.markers.push_back(ramp_marker);
    visualization_msgs::Marker target_marker = TargetPublisher::marker(target_config);
    target_marker.id = id++;
    markers.markers.push_back(target_marker);
    return markers;
  }

private:
  dynamic_reconfigure::Server<ball_simulator::BallSimulatorConfig> reconfiguration_server;
  SimulationConfig simulation_config;
  Ball::Config ball_config;
  Paddle::Config paddle_config;
  RampConfig ramp_config;
  TargetConfig target_config;
  ros::Publisher marker_pub;
};

class SimulationServices
{
public:
  SimulationServices(ros::NodeHandle nh, Simulation::State& simulation_state, Ball::State& ball_state)
    : simulation_state(simulation_state), ball_state(ball_state)
  {
    reset_service = nh.advertiseService("reset", &SimulationServices::reset, this);
    stop_service = nh.advertiseService("stop", &SimulationServices::stop, this);
  }

  bool reset(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
  {
    response.success = simulation_state.reset() && ball_state.reset();
    return true;
  }

  bool stop(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
  {
    response.success = simulation_state.stop();
    return true;
  }

private:
  Simulation::State& simulation_state;
  Ball::State& ball_state;
  ros::ServiceServer reset_service;
  ros::ServiceServer stop_service;
};
}  // namespace ball_simulator

#endif