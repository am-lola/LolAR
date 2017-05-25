#ifndef _CAMERA_POSE_ESTIMATOR_H_
#define _CAMERA_POSE_ESTIMATOR_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <glm/gtx/euler_angles.hpp>

#include <iostream>

#include "utils.hpp"
#include "ArucoTracker.hpp"
#include "FloorDetector.hpp"

template <typename PointT>
class CameraPoseEstimator
{
public:
  CameraPoseEstimator(ArucoTracker tracker, FloorDetector<PointT> floorDetector)
  {
    _markerTracker = tracker;
    _floorDetector = floorDetector;
    _visualizer.Start("Camera Pose Estimation");

    // just for testing at the moment...
    double camera_up[3]       = { 0.0, -1.0, 0.0 };
    double camera_forward[3]  = { 0.0,  0.0, 1.0 };
    double camera_position[3] = { 0.0,  0.0, 0.0 };
    _visualizer.SetCameraPose(camera_position, camera_forward, camera_up);
  }

  CameraPoseEstimator(ArucoTracker tracker, FloorDetector<PointT> floorDetector, Eigen::Vector3f initial_pos, Eigen::Vector3f initial_rot)
  {
    _markerTracker = tracker;
    _floorDetector = floorDetector;
    _visualizer.Start("Camera Pose Estimation");

    _cameraPosition = initial_pos;
    _cameraRotation = initial_rot;

    // set our visualizer's camera to the initial values
    double camera_position[3] = { _cameraPosition[0], _cameraPosition[1], _cameraPosition[2] };
    double camera_rotation[3][3];
    GetRotationMatrix(camera_rotation);
    _visualizer.SetCameraPose(camera_position, camera_rotation);
  }

  ~CameraPoseEstimator()
  {
    _visualizer.Stop();
  }

  // Gets the latest position estimate of the camera
  // @position Buffer to store the position in. Must be of length 3.
  void GetPosition(double* position)
  {
    for (size_t i = 0; i < 3; i++)
    {
      position[i] = _cameraPosition[i];
    }
  }

  // Gets the euler angles for the latest estimate of the camera's orientation
  // @rotation Buffer to store the rotation in. Must be of length 3.
  void GetOrientation(double* rotation)
  {
    for (size_t i = 0; i < 3; i++)
    {
      rotation[i] = _cameraRotation[i];
    }
  }

  // Gets a 3x3 rotation matrix for the camera's current estimated orientation
  // @rotation Buffer to store the rotation matrix in. Must be 3x3 array.
  void GetRotationMatrix(double rotation[3][3])
  {
    Eigen::AngleAxisd rollAngle(-_cameraRotation[2], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(-_cameraRotation[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(-_cameraRotation[1], Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();

    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        rotation[i][j] = rotationMatrix(i, j);
      }
    }
  }

  // Sets the transformation between the marker origin and world origin
  // Should most likely come from robot's kinematics
  void SetMarkerTransform(Eigen::Affine3f transform)
  {
    _markerToWorld = transform;
  }

  // Gets a transformation which can be used to rotate/translate point clouds & other objects
  // relative to the camera from the world coordinate frame
  Eigen::Affine3f GetTransform()
  {
    Eigen::Affine3f transform = _markerToWorld;

    // apply camera transformation
    Eigen::AngleAxisf rollAngle(-_cameraRotation[0], Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(-_cameraRotation[1], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(-_cameraRotation[2], Eigen::Vector3f::UnitZ());

    transform.rotate(Eigen::Quaternionf(rollAngle * pitchAngle * yawAngle));
    transform.pretranslate(_cameraPosition);

    return transform;
  }

  // Updates camera parameters from a pointcloud generated from the camera's perspective
  // From the pointcloud we get: Pitch, Roll, and Height (Z)
  void Update(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    _lastCloud = cloud;

    // display whole pointcloud
    static ar::PointCloudData wholeCloud(ar::PCL_PointXYZ, ar::Color(0.5, 0.5, 0.5, 0.5));
    const PointT* data = &cloud->points[0];
    wholeCloud.pointData = reinterpret_cast<const void*>(data);
    wholeCloud.numPoints = cloud->size();
    static ar::mesh_handle wholeCloud_handle = _visualizer.Add(wholeCloud);
    _visualizer.Update(wholeCloud_handle, wholeCloud);

    // Update height of camera based on floor
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (_floorDetector.FindFloor(cloud, inliers))
    {
      // extract just the points which we found on the floor
      typename pcl::PointCloud<PointT>::Ptr floorcloud(new pcl::PointCloud<PointT>);
      pcl::ExtractIndices<pcl::PointXYZ> extractor;
      extractor.setInputCloud(cloud);
      extractor.setIndices(inliers);
      extractor.setNegative(false);
      extractor.filter(*floorcloud);

      // find correct orientation of the floor & transformation from actual
      auto floor_normal = _floorDetector.GetNormal();
      auto true_normal  = _floorDetector.GetExpectedNormal();
      auto rot_quat = Eigen::Quaternionf::FromTwoVectors(floor_normal, true_normal);

      // get pitch and roll values from rotation
      auto rot_euler = rot_quat.toRotationMatrix().eulerAngles(0, 1, 2);
      _cameraRotation[0] = M_PI - rot_euler[0]; // flipped because camera will have the opposite rotation
      _cameraRotation[2] = M_PI - rot_euler[2];

      // transform floor to match its real-world orientation
      Eigen::Affine3f rotation(rot_quat);
      pcl::transformPointCloud(*floorcloud, *floorcloud, rotation);

      // Estimate centroid of the floor
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*floorcloud, centroid);

      _cameraPosition[2] = centroid[1]; // Z is up in world coordinates, Y is up in sensor coordinates

      // draw transformed floor cloud
      static ar::PointCloudData floorCloudData = ar::PointCloudData(ar::PCL_PointXYZ, ar::Color(1,1,0));
      const PointT* floorCloud_data = &floorcloud->points[0];
      floorCloudData.pointData = reinterpret_cast<const void*>(floorCloud_data);
      floorCloudData.numPoints = floorcloud->size();
      static ar::mesh_handle floorCloud_handle = _visualizer.Add(floorCloudData);
      _visualizer.Update(floorCloud_handle, floorCloudData);
    }

    std::cout << "New Camera Position: [" << _cameraPosition[0] << ", " << _cameraPosition[1] << ", " << _cameraPosition[2] << "]" << std::endl;
    std::cout << "New Camera Orientation: [" << DEGREES(_cameraRotation[0]) << ", " << DEGREES(_cameraRotation[1]) << ", " << DEGREES(_cameraRotation[2]) << "]" << std::endl;
  }

  void Update(cv::Mat& image)
  {
    _markerTracker.Track(image);

    if(_markerTracker.HasPose())
    {
      // if we have good pointcloud data, estimate marker's true position from there
      if (_lastCloud->height > 1)
      {
        double* image_location = _markerTracker.LastImagePos();
        UpdateFromMarkerPos(glm::floor(image_location[0]), glm::floor(image_location[1]));
      }
    }
  }

  // Updates camera parameters from an RGB image captured from the camera's perspective
  // From the image we get: Yaw, X, Y
  void Update(const boost::shared_ptr<openni_wrapper::Image>& image)
  {
    _markerTracker.Track(image);

    if(_markerTracker.HasPose())
    {
      // if we have good pointcloud data, estimate marker's true position from there
      if (_lastCloud->height > 1)
      {
        double* image_location = _markerTracker.LastImagePos();
        UpdateFromMarkerPos(glm::floor(image_location[0]), glm::floor(image_location[1]));
      }
    }
  }

private:
  FloorDetector<PointT> _floorDetector;
  ArucoTracker  _markerTracker;

  Eigen::Affine3f _markerToWorld;  // transform of marker center to world origin
  Eigen::Vector3f _cameraPosition; // camera position in world coordinates
  Eigen::Vector3f _cameraRotation; // euler angles for camera's orientation (in world coords)

  typename pcl::PointCloud<PointT>::ConstPtr _lastCloud; // most recent pointcloud received

  ar::ARVisualizer _visualizer;

  // Updates camera parameters from a detected marker in an RGB image corresponded with points in a recent pointcloud
  // Updates: Yaw, X, Y
  // @markerX x-coord of marker center in image
  // @markerY y-coord of marker center in image
  void UpdateFromMarkerPos(int markerX, int markerY)
  {
    auto point = _lastCloud->at(markerX, markerY);
    if (point.x + point.y + point.z != 0 &&
        !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
    {
      // marker's center
      double truePosition[3] = {point.x, point.y, point.z};
      ar::Sphere truePos(truePosition, 0.02, ar::Color(1,0,0));
      static ar::mesh_handle marker_origin_handle = _visualizer.Add(truePos);
      _visualizer.Update(marker_origin_handle, truePos);

      // extract a portion of the cloud around the detected marker
      typename pcl::PointCloud<PointT>::Ptr markercloud(new pcl::PointCloud<PointT>);

      // Fill in the cloud data /// TODO: scale # of points taken with distance from sensor
      markercloud->width  = 65; 
      markercloud->height = 65; 
      markercloud->points.resize(markercloud->width * markercloud->height);
      for (size_t i = 0; i < markercloud->width; ++i)
      {
        for (size_t j = 0; j < markercloud->height; ++j)
        {
          markercloud->at(i,j) = _lastCloud->at(markerX + i-32, markerY + j-32);
        }
      }

      // find plane near detected Marker
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.01);
      seg.setInputCloud (markercloud);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size() != 0)
      {
        // if we found a plane, extract the points which fit it and display them
        typename pcl::PointCloud<PointT>::Ptr planecloud(new pcl::PointCloud<PointT>);
        pcl::ExtractIndices<pcl::PointXYZ> extractor;
        extractor.setInputCloud(markercloud);
        extractor.setIndices(inliers);
        extractor.setNegative(false);
        extractor.filter(*planecloud);

        const PointT* plane_data = &planecloud->points[0];
        static ar::PointCloudData markerPlane(ar::PCL_PointXYZ, ar::Color(1, 0, 0));
        static ar::mesh_handle markerPlane_handle = _visualizer.Add(markerPlane);
        markerPlane.pointData = reinterpret_cast<const void*>(plane_data);
        markerPlane.numPoints = planecloud->size();
        _visualizer.Update(markerPlane_handle, markerPlane);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (markercloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch (0.03);
        ne.compute (*cloud_normals);

        // compute avg normal
        double avg_normal[3] = {0, 0, 1};
        size_t total_normals = cloud_normals->width * cloud_normals->height;
        for (size_t i = 0; i < cloud_normals->width; ++i)
        {
           for (size_t j = 0; j < cloud_normals->height; ++j)
           {
              // make sure normal is pointing the right way
              Eigen::Vector4f n_flipped(cloud_normals->at(i,j).data_c[0],
                                        cloud_normals->at(i,j).data_c[1],
                                        cloud_normals->at(i,j).data_c[2],
                                        cloud_normals->at(i,j).data_c[3]);
              flipNormalTowardsViewpoint (markercloud->at(i,j), 0.0f, 0.0f, 0.0f, n_flipped);
              if (std::isnan(n_flipped.x()) ||
                  std::isnan(n_flipped.y()) ||
                  std::isnan(n_flipped.z()))
              {
                total_normals--;
                continue;
              }
              avg_normal[0] += n_flipped.x();
              avg_normal[1] += n_flipped.y();
              avg_normal[2] += n_flipped.z();
           }
        }
        avg_normal[0] /= (double)(total_normals);
        avg_normal[1] /= (double)(total_normals);
        avg_normal[2] /= (double)(total_normals);

        // visualize marker board
        static ar::Quad board = ar::Quad(truePosition, avg_normal, 0.4, 0.3, ar::Color(0.8,0.8,0));
        static ar::mesh_handle board_handle = _visualizer.Add(board);
        static ar::Transform boardTransform = ar::Transform();
        boardTransform.translation[0] = truePosition[0];
        boardTransform.translation[1] = truePosition[1];
        boardTransform.translation[2] = truePosition[2];

        // visualize marker normal
        static ar::LinePath n_line = ar::LinePath(0.005f, ar::Color(0.5, 0.52, 0.8));
        static double n_line_end[3];
        Eigen::Vector3f nnormal = Eigen::Vector3f(truePosition[0], truePosition[1], truePosition[2]) +
              Eigen::Vector3f(avg_normal[0], avg_normal[1], avg_normal[2]).normalized()*0.2f;

        n_line_end[0] = nnormal.x();
        n_line_end[1] = nnormal.y();
        n_line_end[2] = nnormal.z();
        n_line.points = {
            truePosition[0],
            truePosition[1],
            truePosition[2],
            n_line_end[0],
            n_line_end[1],
            n_line_end[2]
        };
        static ar::mesh_handle normal_line_handle = _visualizer.Add(n_line);
        _visualizer.Update(normal_line_handle, n_line);

        auto boardRotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(avg_normal[0], avg_normal[1], avg_normal[2])).toRotationMatrix();
        for (size_t i = 0; i < 3; i++)
        {
          for (size_t j = 0; j < 3; j++)
          {
            boardTransform.rotation[i][j] = boardRotation(i, j);
          }
        }
        _visualizer.Update(board_handle, boardTransform, true);

        // find camera's position, relative to marker     /// TODO: add offset from marker to Lola's world origin
        _cameraPosition[0] =  truePosition[2]; // Z in sensor-coords is X in world coords
        _cameraPosition[1] = -truePosition[0]; // X in sensor-coords is Y in world coords

        // find camera's yaw rotation, relative to marker /// TODO: add offset from marker to Lola's world origin
        Eigen::Vector3f board_normal_adjusted = Eigen::Vector3f(avg_normal[0], 0.0f, avg_normal[2]).normalized();
        _cameraRotation[1] = M_PI - acos(board_normal_adjusted.dot(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
        // ensure sign of the rotation is correct -- assumes rotation is about the -Y axis, with +Z aligning to a rotation of 0 degrees
        if (Eigen::Vector3f(0.0f, -1.0f, 0.0f).dot(board_normal_adjusted.cross(Eigen::Vector3f(0.0f, 0.0f, 1.0f))) < 0)
        {
          _cameraRotation[1] = -_cameraRotation[1];
        }
      }
    }
  }
};

#endif // _CAMERA_POSE_ESTIMATOR_H_
