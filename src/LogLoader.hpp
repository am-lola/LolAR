#ifndef _LOG_LOADER_H
#define _LOG_LOADER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <fstream>
#include <vector>

#include "Obstacle.hpp"
#include "Footstep.hpp"

///////////////////////////////////////////////////////////
/// A single event which occurred during recording`
/// e.g. an obstacle added or point cloud received
///////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;

class LogEvent_
{
public:
  double time() { return _timestamp; }

  friend bool operator< (const LogEvent_& l, const LogEvent_& r)
  {
    return l._timestamp < r._timestamp;
  }
  friend bool operator> (const LogEvent_& lhs, const LogEvent_& rhs){ return rhs < lhs; }
  friend bool operator<=(const LogEvent_& lhs, const LogEvent_& rhs){ return !(lhs > rhs); }
  friend bool operator>=(const LogEvent_& lhs, const LogEvent_& rhs){ return !(lhs < rhs); }

protected:
  double _timestamp; // relative time the event occurred at
};

template <typename PointT>
class PointCloudEvent : public LogEvent_
{
public:
  PointCloudEvent(double time, std::string filename)
  {
    this->_timestamp = time;
    pcl::io::loadPCDFile<PointT>(filename, _cloud);
  }

private:
  pcl::PointCloud<PointT> _cloud;
};

class RGBImageEvent : public LogEvent_
{
public:
  RGBImageEvent(double time, std::string filename)
  {
    this->_timestamp = time;
  }

private:
  char* _image;
  unsigned int _width;
  unsigned int _height;
};

template <typename T>
class LogEvent : public LogEvent_
{
public:
  LogEvent(double time, T data)
  {
    this->_timestamp = time;
    this->_data = data;
  }

private:
  T _data;
};

///////////////////////////////////////////////////////////
/// Loads recorded data from a directory
/// for easy playback and analysis
///////////////////////////////////////////////////////////

class LogLoader
{
public:
  LogLoader(std::string dir) : _log_dir(dir)
  {
    std::ifstream metafile(_log_dir + "/metalog.txt");
    double time;
    std::string file;
    while (metafile >> time >> file)
    {
      if (file.find(".pcd") != std::string::npos)
      {
        _log.push_back(PointCloudEvent<PointT>(time, file));
      }
      else if (file.find(".png") != std::string::npos)
      {
        _log.push_back(RGBImageEvent(time, file));
      }
    }
  }

  LogEvent_& nextEntry()
  {
    if (_current_entry < _log.size()-1) {
      return _log[_current_entry+1];
    }
    else
    {
      return _log[_current_entry];
    }
  }

  double nextEventTime()
  {
    return nextEntry().time();
  }

private:
  std::string _log_dir;
  std::vector<LogEvent_> _log;
  unsigned int _current_entry;
};

#endif // _LOG_LOADER_H
