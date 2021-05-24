#include <ball_simulator/ball_simulator/simulation.h>
#include <ros/ros.h>

using namespace ball_simulator;

double fallDuration(const Simulation& simulation, double relative_collision_height)
{
  double fall_duration;
  auto state = simulation.state;
  auto config = simulation.config;

  if (state.bounce_velocity * state.bounce_velocity + 2.0 * config.gravity * relative_collision_height <= 0.0)
  {
    fall_duration = 0.0;
  }
  else
  {
    fall_duration = (state.bounce_velocity + sqrt(state.bounce_velocity * state.bounce_velocity +
                                                  2.0 * config.gravity * relative_collision_height)) /
                    config.gravity;
  }

  return fall_duration;
}

struct RampRollStepResult
{
  Ball::State ball_state;
  bool ramp_end_reached = false;
};

/**
 * This function computes the new state of the ball rolling down the ramp for a new timestamp
 */
RampRollStepResult ramp_roll_step(const ros::Time& stamp, const Simulation& simulation, const Ball::Config& ball_config,
                                  const RampConfig& ramp_config)
{
  RampRollStepResult result;
  auto config = simulation.config;
  auto& state = result.ball_state;
  const double elapsed_time = (stamp - simulation.state.start_time).toSec();

  // we assume the moment of inertia of the ball is 2/3mr²
  const double velocity = 0.6 * config.gravity * std::sin(ramp_config.slope_angle) * elapsed_time;
  const double distance = velocity * elapsed_time;
  double ball_drop = 0.0;

  if (distance >= ramp_config.length - ball_config.radius)
  {
    const double d = std::max(0.0, ball_config.radius - std::max(0.0, ramp_config.length - distance));
    ball_drop = ball_config.radius * (1.0 - std::sqrt(1.0 - d * d / (ball_config.radius * ball_config.radius)));
  }

  state.y =
      (distance - ramp_config.length) * cos(ramp_config.slope_angle) + ball_config.radius + ramp_config.end_offset;
  state.height = ramp_config.end_height - (distance - ramp_config.length) * sin(ramp_config.slope_angle) +
                 ball_config.radius + 0.5 * ramp_config.thickness - ball_drop;
  state.velocity = -velocity * sin(ramp_config.slope_angle);

  result.ramp_end_reached = (distance >= ramp_config.length);

  return result;
}

struct BouncingStepResult
{
  Ball::State ball_state;
  double impulse = 0.0;
};

/**
 * This function computes the new state of the ball bouncing on the paddle for a new timestamp
 */
BouncingStepResult bouncing_step(const ros::Time& stamp, Simulation& simulation, const Ball::Config& ball_config, const Paddle& paddle, const TargetConfig& target_config)
{
  auto& state = simulation.state;
  auto config = simulation.config;
  auto paddle_config = paddle.config();
  BouncingStepResult result;
  auto& ball_state = result.ball_state;
  auto& total_impulse = result.impulse;

  const double current_time = stamp.toSec();

  if (state.first_step)
  {
    state.first_step = false;
    state.collision_time = current_time;
  }

  double elapsed_time = current_time - state.collision_time;
  bool all_bounce_processed = false;
  double potential_collision_height;

  const double future_paddle_velocity = paddle.velocity(current_time);
  const double paddle_velocity_at_collision = paddle.velocity(state.collision_time);
  const double paddle_velocity_increase = future_paddle_velocity - paddle_velocity_at_collision;
  const double ball_velocity_increase = -config.gravity * elapsed_time;

  // ball just sticks to the paddle if they were in contact and paddle is falling slower than gravity
  if (state.contact_at_last_step && paddle_velocity_increase >= ball_velocity_increase)
  {
    potential_collision_height = paddle.height(current_time) + ball_config.radius;
    all_bounce_processed = true;
    state.collision_height = potential_collision_height;
    state.bounce_velocity = 0.0;
  }

  int collision_count = 0;

  // process all bounces occuring between the last step and now
  while (!all_bounce_processed)
  {
    potential_collision_height = paddle.height(state.collision_time + elapsed_time) + ball_config.radius;

    const double next_relative_collision_height = state.collision_height - potential_collision_height;

    double fall_duration = fallDuration(simulation, next_relative_collision_height);

    if (fall_duration <= 1E-4 && next_relative_collision_height <= 1E-5)
    {
      all_bounce_processed = true;
    }

    if (elapsed_time >= fall_duration)
    {
      ++collision_count;
      state.contact_at_last_step = false;
      state.collision_height = potential_collision_height;
      state.collision_time += fall_duration;
      const double ball_velocity_at_collision = state.bounce_velocity - config.gravity * fall_duration;
      const double paddle_velocity_at_collision = paddle.velocity(state.collision_time);

      const double bounce_velocity = ( paddle_config.mass * (1.0 + config.restitution) * paddle_velocity_at_collision + ball_velocity_at_collision * (ball_config.mass - paddle_config.mass * config.restitution) ) / (ball_config.mass + paddle_config.mass);

      //const double collision_impulse = (ball_config.mass * paddle_config.mass) * ( (1.0 + config.restitution) * paddle_velocity_at_collision - config.restitution * ball_velocity_at_collision) / (ball_config.mass + paddle_config.mass);

      state.bounce_velocity =
          std::max(0.0, std::max(paddle_velocity_at_collision, bounce_velocity));
      elapsed_time = current_time - state.collision_time;

      //const double collision_impulse = ball_config.mass * (state.bounce_velocity - ball_velocity_at_collision) / elapsed_time; // m*a   
      const double collision_impulse = -(ball_velocity_at_collision - paddle_velocity_at_collision) * (1 + config.restitution) * sqrt(ball_config.mass * 650) / 3.14 / sqrt(1 + ball_config.mass/paddle_config.mass); // TODO correct hardcoded values..  

      total_impulse += collision_impulse;
      
      //ROS_INFO_STREAM("ball post_velocity: " << bounce_velocity << "\n" <<
      //                "ball pre_velocity: "  << ball_velocity_at_collision << "\n" <<
      //                "ball mass: " << ball_config.mass << "\n" <<
      //                "arm pre_velocity: "  << paddle_velocity_at_collision << "\n" <<
      //                "arm mass: " << paddle_config.mass << "\n" <<
      //                "collision_impulse: " << collision_impulse << "\n" <<
      //                "time elasped: " << elapsed_time);

    }
    else
    {
      all_bounce_processed = true;
    }
  }

  const double future_ball_height =
      state.collision_height + (state.bounce_velocity - 0.5 * config.gravity * elapsed_time) * elapsed_time;

  const double future_ball_velocity = state.bounce_velocity - config.gravity * elapsed_time;

// bouncing error estimation
  const double ball_apex = 0.5*state.bounce_velocity*state.bounce_velocity/config.gravity + state.collision_height;    
  ball_state.bouncing_error = ball_apex - target_config.height;

  // if there is no more bounce and ball is below paddle, ball must rest on top of paddle instead
  if (future_ball_height <= potential_collision_height)
  {
    state.collision_time = current_time;
    state.collision_height = potential_collision_height;
    ball_state.height = potential_collision_height;
    ball_state.velocity = std::max(future_paddle_velocity, future_ball_velocity);
    state.bounce_velocity = std::max(0.0, ball_state.velocity);
    state.contact_at_last_step = true;
  }
  else
  {
    ball_state.height = future_ball_height;
    ball_state.velocity = future_ball_velocity;
    state.contact_at_last_step = false;
  }

  return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_simulator");

  ros::NodeHandle nh;
  ros::NodeHandle pn("~");

  double frequency = 200.0;
  pn.param("rate", frequency, frequency);

  if (!ros::Time::isValid())
  {
    ROS_WARN("waiting for valid time");
    ros::Time::waitForValid();
    ROS_WARN("time is now valid");
  }

  SimulationReconfiguration configuration(pn);
  Ball ball(configuration.ballConfig());
  SinePaddle sine_paddle(configuration.paddleConfig());
  RobotPaddle robot_paddle(configuration.paddleConfig(), pn);
  Simulation simulation(configuration.simulationConfig());
  SimulationServices services(nh, simulation.state, ball.state);
  SimulationPublisher simulation_publisher(nh);

  ros::Rate rate(frequency);

  simulation.state.stop();

  while (nh.ok())
  {
    rate.sleep();
    ros::spinOnce();

    ros::Time stamp = ros::Time::now();

    Paddle* paddle = (simulation.config.enable_sine_paddle) ? static_cast<Paddle*>(&sine_paddle) : static_cast<Paddle*>(&robot_paddle);

    //ROS_INFO_STREAM_THROTTLE(1, "target height: " << configuration.targetConfig().height);

    if (!simulation.state.stopped)
    {
      if (simulation.state.rolling)
      {
        auto result = ramp_roll_step(stamp, simulation, ball.config, configuration.rampConfig());
        ball.state = result.ball_state;
        simulation.state.collision_height = ball.state.height;
        simulation.state.bounce_velocity = ball.state.velocity;
        simulation.state.rolling = !result.ramp_end_reached;
      }
      else
      {
        auto result = bouncing_step(stamp, simulation, ball.config, *paddle, configuration.targetConfig());
        ball.state = result.ball_state;
        paddle->setImpulse(result.impulse);
      }
    }
    simulation_publisher.publish(ball, *paddle, stamp);
  }
}
