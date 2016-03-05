#ifndef _FOOTSTEP_H_
#define _FOOTSTEP_H_

#include <vector>

enum Foot
{
  Right = 0,
  Left  = 1
};

struct Footstep
{
  Foot _foot;
  std::vector<double> _position;
  double _phi;
};

#endif // _FOOTSTEP_H_
