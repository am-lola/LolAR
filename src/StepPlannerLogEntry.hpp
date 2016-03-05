#ifndef _STEPPLANNER_LOG_ENTRY_H_
#define _STEPPLANNER_LOG_ENTRY_H_

#include <vector>

#include "Obstacle.hpp"
#include "Footstep.hpp"

struct StepPlannerLogEntry
{
  unsigned long _stamp;
  std::vector<Footstep> _footsteps;
  std::vector<Obstacle> _obstacles;

  void Clear()
  {
    _stamp = 0;
    _footsteps.clear();
    _obstacles.clear();
  }
};

#endif // _STEPPLANNER_LOG_ENTRY_H_
