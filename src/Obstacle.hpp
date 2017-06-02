#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_

#include <vector>
#include <string>
#include <iostream>

#include <iface_vision_msg.hpp>

enum ObstacleType
{
  Sphere = 0,
  Capsule = 1,
  Triangle = 2
};

struct Obstacle
{
  ObstacleType _type;
  long _id;
  double _radius;
  std::vector<std::vector<double>> _coords;

  Obstacle(){}

  Obstacle(am2b_iface::ObstacleMessage* msg)
  {
    _type   = (ObstacleType)msg->type;
    _id     = msg->model_id;
    _radius = (double)msg->radius;

    for (size_t i = 0; i < 3; i++)
    {
      _coords.push_back({});
      for (size_t j = 0; j < 3; j++)
      {
        _coords[i].push_back((double)msg->coeffs[i*3+j]);
      }
    }
  }

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
      for (size_t i = 0; i < coord.size(); i++)
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
