#ifndef _FLOOR_DETECTOR_H
#define _FLOOR_DETECTOR_H

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

template <typename PointT>
class FloorDetector
{
public:
  FloorDetector()
  {
    _expectedNormal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    _maxDeviation = 0.2;

    _planeModel.setAxis(_expectedNormal);
    _planeModel.setEpsAngle(_maxDeviation);
  }

  FloorDetector(Eigen::Vector3f expected_normal, double deviation)
  {
    _expectedNormal = expected_normal;
    _maxDeviation   = deviation;

    _planeModel.setAxis(expected_normal);
    _planeModel.setEpsAngle(deviation);
  }

  bool FindFloor(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::PointIndices::Ptr inliers)
  {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT>seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() != 0)
    {
      _floorNormal = Eigen::Vector3f(
                    coefficients->values[0] / coefficients->values[3],
                    coefficients->values[1] / coefficients->values[3],
                    coefficients->values[2] / coefficients->values[3]
        );
      std::cout << "New floor normal: ["
                << _floorNormal[0] << ","
                << _floorNormal[1] << ","
                << _floorNormal[2] << "]"
                << std::endl;
      return true;
    }

    return false;
  }

  // Returns the computed normal of the floor
  Eigen::Vector3f GetNormal() { return _floorNormal; }

  // Returns the expected ("true") normal of the floor
  Eigen::Vector3f GetExpectedNormal() { return _expectedNormal; }

private:
  Eigen::Vector3f _expectedNormal; // expected normal vector to the floor
  double          _maxDeviation;   // maximum deviation from expected normal to allow for detection

  Eigen::Vector3f _floorNormal;    // actual (detected) normal of the floor
  double          _floorHeight;    // vertical distance from sensor to floor

  pcl::SACSegmentation<PointT> _planeModel;
};

#endif // _FLOOR_DETECTOR_H
