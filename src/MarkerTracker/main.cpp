#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <glm/gtx/euler_angles.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <utility>

#include <am2b-arvis/ARVisualizer.hpp>

#include "utils.hpp"
#include "ArucoTracker.hpp"

typedef pcl::PointXYZ PointT;

ar::ARVisualizer vizPoints;
ar::ARVisualizer vizImages;
ar::PointCloudData pointCloud(ar::PCL_PointXYZ, ar::Color(0.5,0.5,0.5,0.5));
ar::PointCloudData markerPlane(ar::PCL_PointXYZ, ar::Color(0, 1, 0));
ar::mesh_handle cloud_handle_pts;
ar::mesh_handle cloud_handle_img;
ar::mesh_handle cloud_srf;

ar::mesh_handle marker_origin_img;
ar::mesh_handle marker_board_img;
ar::mesh_handle marker_origin_pts;
ar::mesh_handle marker_board_pts;
double board_center[3] = {0, 0, 0};
double board_normal[3] = {0, 0, -1};
double board_width = 0.07;
double board_height = 0.1;

pcl::Grabber* interface;
pcl::PointCloud<PointT>::ConstPtr lastCloud;

// camera intrinsic parameters for Kinect RGB sensor
double camera_matrix[3][3] = {
  5.2921508098293293e+02, 0.0, 3.2894272028759258e+02,
  0.0, 5.2556393630057437e+02, 2.6748068171871557e+02,
  0.0, 0.0, 1.0
};

// default camera intrinsic parameters
// double camera_matrix[3][3] = {
//   5.25, 0.0, 3.0,
//   0.0, 5.25, 2.0,
//   0.0, 0.0, 1.0
// };

std::vector<double> camera_distortion = {
  2.6451622333009589e-01,
 -8.3990749424620825e-01,
 -1.9922302173693159e-03,
  1.4371995932897616e-03,
  9.1192465078713847e-01
};

ArucoTracker tracker;

// initial camera configuration
double camera_up[3]       = { 0.0, -1.0, 0.0 }; // only correct when robot is not connected
double camera_forward[3]  = { 0.0,  0.0, 1.0 };
double camera_position[3] = { 0.0,  0.0, 0.0 };


/*
  This callback is called from PCL every time a new RGB image is available
*/
void image_callback (const boost::shared_ptr<openni_wrapper::Image>& image)
{
  // only send data if the visualizer has started
  if (vizImages.IsRunning())
  {
    // allocate space to hold image data
    static unsigned char* img_data = new unsigned char[image->getWidth() * image->getHeight() * 3];

    cv::Mat tempImage;
    cv::Mat bgrImage;
    cv::Mat flippedImage;
    tempImage = cv::Mat(image->getHeight(), image->getWidth(), CV_8UC3);
    image->fillRGB(image->getWidth(), image->getHeight(), tempImage.data,
                  tempImage.step);

    cv::cvtColor(tempImage, bgrImage, cv::COLOR_RGB2BGR);
    cv::flip(bgrImage, flippedImage, 1);

    // incoming image is flipped on the X axis
    Mat2Arr(bgrImage, img_data);

    // send image data to visualizer
    // (img_data is safe to delete after this; visualizer makes its own copy )
    vizImages.NotifyNewVideoFrame(image->getWidth(), image->getHeight(), img_data);

    tracker.Track(image);
    if (tracker.HasPose())
    {
      double* pose = tracker.LastTranslation();
      board_center[0] = pose[0]; // + 0.15;
      board_center[1] = pose[1]; // + 0.15;
      board_center[2] = pose[2];

      ar::Transform board_transform(board_center);
      glm::vec3 euler_angles = glm::vec3(-tracker.LastRotation()[2], -tracker.LastRotation()[0], -tracker.LastRotation()[1]);
      glm::mat3 rotation = glm::orientate3(euler_angles);
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          board_transform.rotation[i][j] = rotation[i][j];
        }
      }
      vizImages.Update(marker_origin_img, ar::Sphere(pose, 0.025, ar::Color(1, 1, 0)));
      // vizImages.Update(marker_board_img, board_transform, true);
      vizPoints.Update(marker_origin_pts, ar::Sphere(pose, 0.025, ar::Color(1, 1, 0)));
      // vizPoints.Update(marker_board_pts, board_transform, true);

      double dist = glm::length(glm::vec3(board_center[0], board_center[1], board_center[2]));
      std::cout << "Distance: " << dist << std::endl;

      if (lastCloud->height > 1)
      {
        double* image_location = tracker.LastImagePos();
        if (lastCloud->height > image_location[1] && lastCloud->width > image_location[0])
        {
          auto point = lastCloud->at(glm::floor(image_location[0]), glm::floor(image_location[1]));
          if (point.x + point.y + point.z != 0 &&
              !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
          {
            std::cout << "TRUE Distance: " << glm::length(glm::vec3(point.x, point.y, point.z)) << std::endl;
            std::cout << "Delta: " << dist - glm::length(glm::vec3(point.x, point.y, point.z)) << std::endl;
            double truePosition[3] = {point.x, point.y, point.z};
            board_center[0] = truePosition[0];
            board_center[1] = truePosition[1];
            board_center[2] = truePosition[2];
            ar::Sphere truePos(truePosition, 0.02, ar::Color(1,0,0));
            static ar::mesh_handle truePos_handle = vizPoints.Add(truePos);
            vizPoints.Update(truePos_handle, truePos);
          }
        }

        pcl::PointCloud<PointT>::Ptr subcloud(new pcl::PointCloud<PointT>);

        // Fill in the cloud data
        subcloud->width  = 65;
        subcloud->height = 65;
        subcloud->points.resize (subcloud->width * subcloud->height);

        // Generate the data
        for (size_t i = 0; i < subcloud->width; ++i)
        {
          for (size_t j = 0; j < subcloud->height; ++j)
          {
            subcloud->at(i,j) = lastCloud->at(image_location[0] + i-32, image_location[1] + j-32);
          }
        }

        // find plane near detected Marker
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (subcloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
          PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        }
        else
        {
          // if we found a plane, extract the points which fit it and display them
          pcl::PointCloud<PointT>::Ptr planecloud(new pcl::PointCloud<PointT>);
          pcl::ExtractIndices<pcl::PointXYZ> extractor;
          extractor.setInputCloud (subcloud);
          extractor.setIndices (inliers);
          extractor.setNegative (false);
          extractor.filter (*planecloud);

          const PointT* plane_data = &planecloud->points[0];
          markerPlane.pointData = reinterpret_cast<const void*>(plane_data);
          markerPlane.numPoints = planecloud->size();
          vizPoints.Update(cloud_srf, markerPlane);

          std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                              << coefficients->values[1] << " "
                                              << coefficients->values[2] << " "
                                              << coefficients->values[3] << std::endl;
          board_normal[0] = coefficients->values[0] / coefficients->values[3];
          board_normal[1] = coefficients->values[1] / coefficients->values[3];
          board_normal[2] = coefficients->values[2] / coefficients->values[3];
          ar::Quad board = ar::Quad(board_center, board_normal, 0.4, 0.3, ar::Color(0.8,0.8,0));
          vizPoints.Update(marker_board_pts, board);
          vizImages.Update(marker_board_img, board);
        }
      }
    }
  }
}

void cloud_cb (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  // only send data if the visualizer has started
  if (vizPoints.IsRunning())
  {
    const PointT* data = &cloud->points[0];
    pointCloud.pointData = reinterpret_cast<const void*>(data);
    pointCloud.numPoints = cloud->size();
    vizPoints.Update(cloud_handle_pts, pointCloud); // give the visualizer the new points

  }

  lastCloud = cloud;
}

int main(int argc, char* argv[])
{
  std::cout << "Opening sensor" << std::endl;
  interface = new pcl::OpenNIGrabber("", pcl::OpenNIGrabber::OpenNI_VGA_30Hz);

  // initialize marker tracker
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
  auto marker = cv::aruco::GridBoard::create(1,2, 0.195, 0.01, dictionary);
  // auto marker = cv::aruco::GridBoard::create(2,2, 0.145, 0.01, dictionary);
  tracker = ArucoTracker(camera_matrix, camera_distortion, marker);

  // subscribe to grabber's pointcloud callback
  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f_points = boost::bind(&cloud_cb, _1);
  interface->registerCallback(f_points);
  // subscribe to grabber's image callback so we get RGB data
  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> f_rgb = boost::bind (&image_callback, _1);
  interface->registerCallback (f_rgb);
  interface->start();

  // Start RGB data visualizer
  vizImages.Start("AR View");
  vizImages.SetCameraIntrinsics(camera_matrix);
  vizImages.SetCameraPose(camera_position, camera_forward, camera_up);

  // Start pointcloud data visualizer
  vizPoints.Start("PointCloud View");
  vizPoints.SetCameraPose(camera_position, camera_forward, camera_up);

  // add an empty point cloud to visualizer, which we will upate each frame
  cloud_handle_pts = vizPoints.Add(pointCloud);
  cloud_handle_img = vizImages.Add(pointCloud);
  cloud_srf        = vizPoints.Add(markerPlane);

  marker_origin_img = vizImages.Add(ar::Sphere(0, 0, 0, 0.025, ar::Color(1, 1, 0)));
  marker_board_img  = vizImages.Add(ar::Quad(board_center, board_normal, board_width, board_height, ar::Color(0, 1, 0)));
  marker_origin_pts = vizPoints.Add(ar::Sphere(0, 0, 0, 0.025, ar::Color(1, 1, 0)));
  marker_board_pts  = vizPoints.Add(ar::Quad(board_center, board_normal, board_width, board_height, ar::Color(0, 1, 0)));

  // wait for user to exit
  std::cout << "Press enter when ready to exit..." << std::endl;
  std::cin.get();
  std::cout << "Shutting down..." << std::endl;

  vizPoints.Stop();
  vizImages.Stop();

  return 0;
}
