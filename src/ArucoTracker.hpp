#ifndef _ARUCO_TRACKER_H_
#define _ARUCO_TRACKER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

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

    _cameraMatrix.at<float>(0,0) = 5.2921508098293293e+02;
    _cameraMatrix.at<float>(1,1) = 5.2556393630057437e+02;
    _cameraMatrix.at<float>(2,2) = 1.0;
    _cameraMatrix.at<float>(0,2) = 3.2894272028759258e+02;
    _cameraMatrix.at<float>(1,2) = 2.6748068171871557e+02;
  }

  ArucoTracker(std::vector<float> distortion, float camera[3][3])
  {
    _distortionCoefficients = cv::Mat(1, distortion.size(), CV_32FC1);
    _cameraMatrix = cv::Mat(3, 3, CV_32FC1, 0.0);

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        _cameraMatrix.at<float>(i, j) = camera[i][j];
      }
    }

    for (int i = 0; i < distortion.size(); i++)
    {
      _distortionCoefficients.at<float>(0, i) = distortion.at(i);
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
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    auto gridboard = cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, dictionary);
    auto board = gridboard.staticCast<cv::aruco::Board>();

    cv::Mat imageCopy = image.clone();

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);

    //if at least one marker detected
    if (ids.size() > 0)
    {
      cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

      cv::Vec3d rvec, tvec;
      int valid = cv::aruco::estimatePoseBoard(corners, ids, board, _cameraMatrix, _distortionCoefficients, rvec, tvec);

      if(valid > 0)
      {
        cv::aruco::drawAxis(imageCopy, _cameraMatrix, _distortionCoefficients, rvec, tvec, 0.1);
        _translation[0] = tvec[0];
        _translation[1] =-tvec[1];
        _translation[2] = tvec[2];

        for (int i = 0; i < 3; i++)
        {
          _rotation[i] = rvec[i];
        }


        _hasPose = true;
      }
    }

    cv::imshow("out", imageCopy);
    cv::waitKey(10);
  }

  bool HasPose() { return _hasPose; }

  double* LastTranslation() { return _translation; }
  double* LastRotation() { return _rotation; }

private:
  bool _hasPose = false;

  double _translation[3];
  double _rotation[3];

  cv::Mat _cameraMatrix; // camera intrinsic parameters
  cv::Mat _distortionCoefficients; // lens distortion parameters
};

#endif // _ARUCO_TRACKER_H_
