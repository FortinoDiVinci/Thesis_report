#ifndef BALL_SIMULATOR_LOGGER_H
#define BALL_SIMULATOR_LOGGER_H

#include <sstream>
#include <string>

namespace ball_simulator {

class LogEntry
{
public:
    LogEntry(const std::string& delimeter = ",") : delimeter(delimeter) {}

    inline std::string str() const
    {
        return stream.str();
    }

    struct EmptyField
    {};

private:
    template <typename T>
    friend LogEntry& operator<<(LogEntry& entry, const T& field);
    friend LogEntry& operator<<(LogEntry& entry, const EmptyField& field);

private:
    std::stringstream stream;
    std::string delimeter;
};

template <typename T>
LogEntry& operator<<(LogEntry& entry, const T& field)
{
    if (entry.stream.str().empty())
    {
        entry.stream << field;
    }
    else
    {
        entry.stream << entry.delimeter << field;
    }

    return entry;
}

} // namespace ball_simulator

#endif