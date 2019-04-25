#include <ball_simulator/logger/log_entry.h>

namespace ball_simulator {

LogEntry& operator<<(LogEntry& entry, const LogEntry::EmptyField& field)
{
    entry.stream << entry.delimeter;
    return entry;
}

} // namespace ball_simulator