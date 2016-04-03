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

  StepPlannerLogEntry Entry(unsigned int idx) { return _log.at(idx); }

private:

  std::vector<StepPlannerLogEntry> _log;

  // reads the given stepplanner log file and populates _log with all entries
  void ParseStepLog(std::string filename);

  // # of entries in each log line
  const size_t _log_width             = 40;
  const size_t _max_footsteps         =  8; // max number of footsteps per frame (any others are discarded)

  // offsets into each log line of data we need
  const size_t _stamp_idx             =  0;

  const size_t _footstep_foot_idx     = 13;
  const size_t _footstep_coord_idx    =  8;
  const size_t _footstep_coord_size   =  3;
  const size_t _footstep_rotation_idx = 11; /// TODO: rotation will eventually have 3 components

  const size_t _obstacle_id_idx       = 26;
  const size_t _obstacle_type_idx     = 25;
  const size_t _obstacle_clear_idx    = 29;
  const size_t _obstacle_radius_idx   = 28;
  const size_t _obstacle_coord_idx    = 31;
  const size_t _obstacle_coord_size   =  9;
};

#endif // _STEPPLANNER_LOG_READER_H_
