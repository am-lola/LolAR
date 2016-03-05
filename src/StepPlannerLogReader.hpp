#ifndef _STEPPLANNER_LOG_READER_H_
#define _STEPPLANNER_LOG_READER_H_

#include <string>
#include <vector>

#include "StepPlannerLogEntry.hpp"

class StepPlannerLogReader
{
public:
  StepPlannerLogReader(std::string logFile);

  std::vector<StepPlannerLogEntry> Entries() { return _log; }

private:

  std::vector<StepPlannerLogEntry> _log;

  void ParseStepLog(std::string filename);
};

#endif // _STEPPLANNER_LOG_READER_H_
