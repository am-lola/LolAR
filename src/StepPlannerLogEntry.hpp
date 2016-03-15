#ifndef _STEPPLANNER_LOG_ENTRY_H_
#define _STEPPLANNER_LOG_ENTRY_H_

#include <vector>
#include <string>

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

  operator std::string() const
  {
    std::string result;
    result += "<" + std::to_string(_stamp) + "> : \n";
    result += "Footstep Positions:\n";
    for (auto step : _footsteps)
    {
      result += "\t";
      result += step;
      result += "\n";
    }
    result += "Obstacles:\n";
    for (auto obstacle : _obstacles)
    {
      result += "\t";
      result += obstacle;
      result += "\n";
    }

    return result;
  }

};

#endif // _STEPPLANNER_LOG_ENTRY_H_
