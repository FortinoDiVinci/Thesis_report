#ifndef BALL_SIMULATOR_LOG_FILE_H
#define BALL_SIMULATOR_LOG_FILE_H

#include <ball_simulator/StartLogging.h>
#include <ros/ros.h>
#include <fstream>
#include <memory>

namespace ball_simulator
{
class LogFile
{
public:
  LogFile();

  void open(const std::string& new_log_file_path);

  void append(const std::string& entry);

private:
  std::unique_ptr<std::ofstream> log_file_;
  std::string log_file_path_;
};

}  // namespace ball_simulator

#endif