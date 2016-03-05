#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_

#include <vector>

enum ObstacleType
{
  Sphere = 0,
  Capsule = 1
};

struct Obstacle
{
  ObstacleType _type;
  unsigned long _id;
  double _radius;
  std::vector<std::vector<double>> _coords;
};

#endif // _OBSTACLE_H_
