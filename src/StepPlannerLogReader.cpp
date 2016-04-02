#include "StepPlannerLogReader.hpp"

#include <fstream>
#include <sstream>
#include <utility>
#include <iostream>

StepPlannerLogReader::StepPlannerLogReader(std::string logFile)
{
  ParseStepLog(logFile);
}

void StepPlannerLogReader::ParseStepLog(std::string filename)
{
  if (_log.size() > 0)
  {
    _log.clear();
  }

  std::ifstream infile(filename);
  std::string line;
  std::string splitLine[_log_width];

  std::string splitLine_header[_log_width];

  StepPlannerLogEntry current_entry;
  current_entry._stamp = 0;

  while (std::getline(infile, line))
  {
    if ( line.find("#") == 0 )  // skip commented lines
    {
      std::stringstream ss(line);
      int i = 0;
      while ( ss.good() && i < _log_width )
      {
        ss >> splitLine_header[i];
        i++;
      }
      continue;
    }

    std::stringstream ss(line);
    int i = 0;
    while ( ss.good() && i < _log_width )
    {
      ss >> splitLine[i];
      i++;
    }

    // skip entries with a timestamp of 0
    if ( splitLine[0] == "0" )
    {
      continue;
    }
    else if ( std::stoul(splitLine[_stamp_idx]) != current_entry._stamp )
    {
      // add current entry if it's valid
      if (current_entry._stamp > 0)
      {
        _log.push_back(current_entry);
      }

      current_entry.Clear();
      current_entry._stamp = std::stoul(splitLine[_stamp_idx]);
    }

    // stop parsing footsteps if we already have the max. for this frame
    // (any additional log entries will contain invalid footstep data)
    if (current_entry._footsteps.size() < _max_footsteps)
    {
      Footstep newFootstep;
      newFootstep._foot = static_cast<Foot>(std::stoi(splitLine[_footstep_foot_idx]));
      newFootstep._phi  = std::stod(splitLine[_footstep_rotation_idx]);
      for (int i = _footstep_coord_idx; i < _footstep_coord_idx + _footstep_coord_size; i++)
      {
        newFootstep._position.push_back(std::stod(splitLine[i]));
      }
      current_entry._footsteps.push_back(newFootstep);
    }

    // obstacle data
    Obstacle newObstacle;
    newObstacle._id     = std::stol(splitLine[_obstacle_id_idx]);
    newObstacle._type   = static_cast<ObstacleType>(std::stoi(splitLine[_obstacle_type_idx]));
    newObstacle._radius = std::stod(splitLine[_obstacle_radius_idx]);

    // obstacle coordinates
    for (int i = _obstacle_coord_idx; i < _obstacle_coord_idx + _obstacle_coord_size; i += 3)
    {
      newObstacle._coords.push_back({
        {
          std::stod(splitLine[i]),
          std::stod(splitLine[i+1]),
          std::stod(splitLine[i+2])
        }
      });
    }

    // obstacle ids < 0 should be ignored
    if (newObstacle._id >= 0)
    {
      current_entry._obstacles.push_back(newObstacle);
    }

  }

  _log.push_back(current_entry);

}
