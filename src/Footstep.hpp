#ifndef _FOOTSTEP_H_
#define _FOOTSTEP_H_

#include <vector>
#include <string>

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
  uint64_t _stamp;

  operator std::string() const
  {
    std::string result;
    result += std::to_string(_stamp) + " : ";
    result += _foot == Left ? "Left:  " : "Right: ";
    result += "(";
    for (size_t i = 0; i < _position.size(); i++)
    {
      result += std::to_string(_position[i]);

      if (i < _position.size() - 1)
      {
        result += ", ";
      }
    }
    result += "), " + std::to_string(_phi);
    return result;
  }
};

#endif // _FOOTSTEP_H_
