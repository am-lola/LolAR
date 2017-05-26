#ifndef _ARUCO_TRACKER_H_
#define _ARUCO_TRACKER_H_

#include <pcl/io/openni_grabber.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "CameraIntrinsics.hpp"

class ArucoTracker
{
public:
  ArucoTracker()
  {
    _distortionCoefficients = cv::Mat(1,5,CV_32FC1);
    _cameraMatrix = cv::Mat(3,3,CV_32FC1,0.0);

    _distortionCoefficients.at<float>(0,0) =  2.6451622333009589e-01;
    _distortionCoefficients.at<float>(0,1) = -8.3990749424620825e-01;
    _distortionCoefficients.at<float>(0,2) = -1.9922302173693159e-03;
    _distortionCoefficients.at<float>(0,3) =  1.4371995932897616e-03;
    _distortionCoefficients.at<float>(0,4) =  9.1192465078713847e-01;

    // default camera intrinsic matrix
    _cameraMatrix.at<float>(0,0) = 5.25;
    _cameraMatrix.at<float>(1,1) = 5.25;
    _cameraMatrix.at<float>(2,2) = 1.0;
    _cameraMatrix.at<float>(0,2) = 3.0;
    _cameraMatrix.at<float>(1,2) = 2.0;

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    _markerBoard = cv::aruco::GridBoard::create(1, 2, 0.05, 0.012, dictionary);
  }

  ArucoTracker(CameraIntrinsics camera_params)
  {
    _distortionCoefficients = cv::Mat(1, camera_params.distortion.size(), CV_32FC1);
    _cameraMatrix = cv::Mat(3, 3, CV_32FC1, 0.0);

    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        _cameraMatrix.at<float>(i, j) = camera_params.intrinsics[i][j];
      }
    }

    for (size_t i = 0; i < camera_params.distortion.size(); i++)
    {
      _distortionCoefficients.at<float>(0, i) = camera_params.distortion.at(i);
    }

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    _markerBoard = cv::aruco::GridBoard::create(1, 2, 0.05, 0.012, dictionary);
  }

  ArucoTracker(CameraIntrinsics camera_params, cv::Ptr<cv::aruco::Board> markerBoard)
  {
    _markerBoard = markerBoard;

    _distortionCoefficients = cv::Mat(1, camera_params.distortion.size(), CV_32FC1);
    _cameraMatrix = cv::Mat(3, 3, CV_32FC1, 0.0);

    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        _cameraMatrix.at<float>(i, j) = camera_params.intrinsics[i][j];
      }
    }

    for (size_t i = 0; i < camera_params.distortion.size(); i++)
    {
      _distortionCoefficients.at<float>(0, i) = camera_params.distortion.at(i);
    }
  }

  void Track(const boost::shared_ptr<openni_wrapper::Image>& image)
  {
    cv::Mat m = cv::Mat(image->getHeight(), image->getWidth(), CV_8UC3);
    image->fillRGB(m.cols,m.rows,m.data,m.step);
    cvtColor(m, m, CV_RGB2BGR);

    Track(m);
  }

  void Track(cv::Mat image)
  {
    cv::Mat imageCopy = image.clone();  // used for debug output

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(image, _markerBoard->dictionary, corners, ids);

    //if at least one marker detected
    if (ids.size() > 0)
    {
      cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

      cv::Vec3d rvec, tvec;
      int valid = cv::aruco::estimatePoseBoard(corners, ids, _markerBoard, _cameraMatrix, _distortionCoefficients, rvec, tvec);

      if(valid > 1) // only process pose if at least two markers were seen
      {
        cv::aruco::drawAxis(imageCopy, _cameraMatrix, _distortionCoefficients, rvec, tvec, 0.1);
        _translation[0] = tvec[0];
        _translation[1] = tvec[1];
        _translation[2] = tvec[2];

        for (int i = 0; i < 3; i++)
        {
          _rotation[i] = rvec[i];
        }

        // save image location of Marker
        cv::Point2f centroid(0, 0);
        int total = 0;
        for (auto rect : corners)
        {
          for (auto point : rect)
          {
            centroid.x += point.x;
            centroid.y += point.y;
            total++;
          }
        }
        centroid.x /= (float)total;
        centroid.y /= (float)total;
        _imageCenter[0] = centroid.x;
        _imageCenter[1] = centroid.y;

        // save marker centers
        cv::Point2f center1(0, 0);
        total = 0;
        for (auto point : corners[0])
        {
            center1.x += point.x;
            center1.y += point.y;
            total++;
        }
        center1.x /= total;
        center1.y /= total;
        cv::Point2f center2(0, 0);
        total = 0;
        for (auto point : corners[1])
        {
            center2.x += point.x;
            center2.y += point.y;
            total++;
        }
        center2.x /= total;
        center2.y /= total;

        if (center2.x < center1.x)
        {
            _img_x_axis[2] = center1.x;
            _img_x_axis[3] = center1.y;
            _img_x_axis[0] = center2.x;
            _img_x_axis[1] = center2.y;
        }
        else
        {
            _img_x_axis[0] = center1.x;
            _img_x_axis[1] = center1.y;
            _img_x_axis[2] = center2.x;
            _img_x_axis[3] = center2.y;
        }
        _hasPose = true;
      }
    }

    cv::imshow("Marker Detection", imageCopy);
    cv::waitKey(10);
  }

  bool HasPose() { return _hasPose; }

  double* LastTranslation() { return _translation; }
  double* LastRotation() { return _rotation; }
  double* LastImagePos() { return _imageCenter; }
  double* LastAxis() { 
      return _img_x_axis;
  }
  

private:
  bool _hasPose = false;

  double _translation[3];
  double _rotation[3];
  double _imageCenter[2];
  double _img_x_axis[4];

  cv::Ptr<cv::aruco::Board> _markerBoard; // marker board to detect
  cv::Mat _cameraMatrix;           // camera intrinsic parameters
  cv::Mat _distortionCoefficients; // lens distortion parameters
};

#endif // _ARUCO_TRACKER_H_
