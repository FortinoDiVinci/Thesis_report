#ifndef BALL_SIMULATOR_CONFIGURATION_H
#define BALL_SIMULATOR_CONFIGURATION_H

#include <ball_simulator/BallSimulatorConfig.h>

namespace ball_simulator
{
class Configuration
{
public:
  virtual void reconfigure(ball_simulator::BallSimulatorConfig& config) {}
};
}

#endif