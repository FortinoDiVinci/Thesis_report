#include <ball_simulator/logger/log_file.h>

namespace ball_simulator
{
LogFile::LogFile()
{
}

void LogFile::open(const std::string& new_log_file_path)
{
  try
  {
    log_file_ = std::unique_ptr<std::ofstream>(new std::ofstream(new_log_file_path, std::ofstream::out));
    log_file_path_ = new_log_file_path;
  }
  catch (const std::ofstream::failure& e)
  {
    ROS_WARN("log cannot be recorded to %s: %s", new_log_file_path.c_str(), e.what());
  }
}

void LogFile::append(const std::string& entry)
{
  try
  {
    if (log_file_)
    {
      *log_file_ << entry << std::endl;
    }
  }
  catch (const std::ofstream::failure& e)
  {
    ROS_WARN("cannot write to log file %s, %s", log_file_path_.c_str(), e.what());
  }
}

}  // namespace ball_simulator
