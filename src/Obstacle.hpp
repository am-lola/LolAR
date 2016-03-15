#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_

#include <vector>
#include <string>

enum ObstacleType
{
  Sphere = 0,
  Capsule = 1
};

struct Obstacle
{
  ObstacleType _type;
  long _id;
  double _radius;
  std::vector<std::vector<double>> _coords;

  operator std::string() const
  {
    std::string result;

    result += "<" + std::to_string(_id) + ">: ";

    switch (_type)
    {
      case Sphere:
        result += "Sphere: ";
        break;
      case Capsule:
        result += "Capsule: ";
        break;
      default:
        result += "UNKNOWN: ";
        break;
    }

    for (auto coord : _coords)
    {
      result += "(";
      for (int i = 0; i < coord.size(); i++)
      {
        result += std::to_string(coord[i]);
        if (i < coord.size() - 1)
        {
          result += ", ";
        }
      }
      result += "), ";
    }

    result += "Radius: " + std::to_string(_radius);

    return result;
  }
};

#endif // _OBSTACLE_H_
