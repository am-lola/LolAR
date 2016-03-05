#include "StepPlannerLogReader.hpp"

#include <fstream>
#include <sstream>
#include <utility>

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
  std::string splitLine[36];

  StepPlannerLogEntry current_entry;
  current_entry._stamp = 0;

  while (std::getline(infile, line))
  {
    if ( line.find("#") == 0 )  // skip commented lines
      continue;

    std::stringstream ss(line);
    int i = 0;
    while ( ss.good() && i < 36 )
    {
      ss >> splitLine[i];
      i++;
    }

    // skip entries with a timestamp of 0
    if ( splitLine[0] == "0" )
      continue;
    else if ( std::stoul(splitLine[0]) != current_entry._stamp )
    {
      _log.push_back(current_entry);
      current_entry.Clear();
      current_entry._stamp = std::stoul(splitLine[0]);
    }

    Footstep newFootstep;
    newFootstep._foot = static_cast<Foot>(std::stoi(splitLine[13]));
    newFootstep._phi  = std::stod(splitLine[4]);
    for (int i = 8; i <=10; i++)
    {
      newFootstep._position.push_back(std::stod(splitLine[i]));
    }
    current_entry._footsteps.push_back(newFootstep);

    // obstacle data is in entries 21 - 35
    Obstacle newObstacle;
    newObstacle._type   = static_cast<ObstacleType>(std::stoi(splitLine[21]));
    newObstacle._id     = std::stoul(splitLine[22]);
    newObstacle._radius = std::stod(splitLine[24]);

    // obstacle coordinates
    for (int i = 27; i <= 35; i += 3)
    {
      newObstacle._coords.push_back({
        {
          std::stod(splitLine[i]),
          std::stod(splitLine[i+1]),
          std::stod(splitLine[i+2])
        }
      });
    }
    current_entry._obstacles.push_back(newObstacle);
  }

  _log.push_back(current_entry);

}
