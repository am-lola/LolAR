#include <pcl/common/transforms.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <utility>

#include <am2b-arvis/ARVisualizer.hpp>

#include "StepPlannerLogReader.hpp"
#include "utils.hpp"

ar::ARVisualizer vizPoints;
ar::ARVisualizer vizImages;
ar::PointCloudData pointCloud(ar::PCL_PointXYZ);
ar::mesh_handle cloud_handle_pts;
ar::mesh_handle cloud_handle_img;

pcl::Grabber* interface;

// camera intrinsic parameters for RGB sensor
// double camera_matrix[3][3] = {
//   5.2921508098293293e+02, 0.0, 3.2894272028759258e+02,
//   0.0, 5.2556393630057437e+02, 2.6748068171871557e+02,
//   0.0, 0.0, 1.0
// };

// default camera intrinsic parameters
double camera_matrix[3][3] = {
  5.25, 0.0, 3.0,
  0.0, 5.25, 2.0,
  0.0, 0.0, 1.0
};

// initial camera configuration
double camera_up[3]       = { 0.0,   0.0, 1.0 }; // only correct when robot is not connected
double camera_forward[3]  = {-1.0,  0.08,-0.3 };
double camera_position[3] = {-1.2,  -0.5, 6.3 };
double arcam_position[3]  = { 6.3, -1.45, 1.8 };


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
    Mat2Arr(flippedImage, img_data);

    // send image data to visualizer
    // (img_data is safe to delete after this; visualizer makes its own copy )
    vizImages.NotifyNewVideoFrame(image->getWidth(), image->getHeight(), img_data);
  }
}

typedef pcl::PointXYZ PointT;

void cloud_cb (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  // only send data if the visualizer has started
  if (vizPoints.IsRunning())
  {
    Eigen::Affine3f cloud_transform = Eigen::Affine3f::Identity();
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(
      Eigen::Vector3f( 0.0f,-1.0f, 0.0f), // y_dir
      Eigen::Vector3f(-1.0f, 0.0f, 0.0f), // z_dir
      Eigen::Vector3f(camera_position[0], camera_position[1], camera_position[2]),
      cloud_transform
    );

    Eigen::Affine3f cloud_transform_2 = Eigen::Affine3f::Identity();
    cloud_transform_2.rotate(Eigen::AngleAxis<float>(0.5f*M_PI, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, cloud_transform);
    pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, cloud_transform_2);

    const PointT* data = &transformed_cloud->points[0];
    pointCloud.pointData = reinterpret_cast<const void*>(data);
    pointCloud.numPoints = cloud->size();
    vizPoints.Update(cloud_handle_pts, pointCloud); // give the visualizer the new points
  }
}

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    std::cout << "ERROR: You must provide an input filename!" << std::endl;
    return -1;
  }

  std::string oni_input = argv[1];
  std::string step_plan = argv[2];


  interface = new pcl::ONIGrabber(oni_input, true, true);

  // subscribe to grabber's pointcloud callback
  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f_points = boost::bind(&cloud_cb, _1);
  interface->registerCallback(f_points);
  // subscribe to grabber's image callback so we get RGB data
  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> f_rgb = boost::bind (&image_callback, _1);
  interface->registerCallback (f_rgb);
  interface->start();

  vizImages.Start("AR View");
  vizImages.SetCameraPose(arcam_position, camera_forward, camera_up);
  vizImages.SetCameraIntrinsics(camera_matrix);

  vizPoints.Start("PointCloud View");
  vizPoints.SetCameraPose(arcam_position, camera_forward, camera_up);

  // add an empty point cloud to visualizer, which we will upate each frame
  cloud_handle_pts = vizPoints.Add(pointCloud);
  cloud_handle_img = vizImages.Add(pointCloud);

  StepPlannerLogReader log(step_plan);

  std::cout << "Loaded " << log.Entries().size() << " log entries" << std::endl;

  for (auto entry : log.Entries())
  {
    static double c = 0.0;
    c += 1.0;

    for (size_t i = 0; i < entry._footsteps.size(); i++)
    {
      auto footstep = entry._footsteps[i];
      double quadNormal[3] = {0, 0, 1};
      ar::Quad newQuad(
        footstep._position[0], footstep._position[1], footstep._position[2],
        quadNormal,
        0.15, 0.15,
        ar::Color(c / (double)log.Entries().size(), footstep._foot == Left ? 1 : 0, 0.9, 0.5)
      );
      vizImages.Add(newQuad);
      vizPoints.Add(newQuad);

      // find previous footstep from the same leg to draw a path
      if (i > 0)
      {
        for (int j = i-1; j >= 0; j--)
        {
          // draw an arc connecting each pair of footsteps
          /// TODO: get the true path data from the step planner
          if (entry._footsteps[j]._foot == footstep._foot)
          {
            Footstep& prevstep = entry._footsteps[j];
            ar::LinePath footpath(0.004f, ar::Color(1, 1, 1));
            int nVerts = 16; // number of points for each path
            // set the highest point of each step path based on XY distance between steps
            double step_height = lerp(0, 0.25, (footstep._position[0] - prevstep._position[0]) * (footstep._position[0] - prevstep._position[0]) +
                                               (footstep._position[1] - prevstep._position[1]) * (footstep._position[1] - prevstep._position[1]));
            for (size_t k = 0; k <= nVerts; k++)
            {
              footpath.points.push_back(lerp(prevstep._position[0], footstep._position[0], (double)k / (double)nVerts));
              footpath.points.push_back(lerp(prevstep._position[1], footstep._position[1], (double)k / (double)nVerts));
              if (k < nVerts / 2)
                footpath.points.push_back(easeInOut(prevstep._position[2], prevstep._position[2] + step_height, (double)k / (double)(nVerts / 2)));
              else
                footpath.points.push_back(easeInOut(prevstep._position[2] + step_height, footstep._position[2], (double)(k-nVerts/2) / (double)(nVerts / 2)));
            }

            vizPoints.Add(footpath);
            vizImages.Add(footpath);
            break;
          }
        }
      }
    }

    for (auto obstacle : entry._obstacles)
    {
      switch (obstacle._type)
      {
        case Sphere:
        {
          ar::Sphere newSphere(
            obstacle._coords[0][0], obstacle._coords[0][1], obstacle._coords[0][2], // center
            obstacle._radius, // radius
            ar::Color(0, 0.4, 0.8, 0.5)
          );
          vizImages.Add(newSphere);
          vizPoints.Add(newSphere);
          break;
        }
        case Capsule:
        {
          ar::Capsule newCapsule(
            obstacle._coords[0][0], obstacle._coords[0][1], obstacle._coords[0][2], // center1
            obstacle._coords[1][0], obstacle._coords[1][1], obstacle._coords[1][2],// center2
            obstacle._radius, // radius
            ar::Color(0.6, 0.2, 0.6, 0.5)
          );
          vizImages.Add(newCapsule);
          vizPoints.Add(newCapsule);
          break;
        }
        default:
        {
          std::cout << "Unexpected obstacle type: " << obstacle._type << std::endl;
        }
      }
    }
  }

  // Add a sphere to mark the origin in both windows
  vizPoints.Add(ar::Sphere(0,0,0,0.05, ar::Color(1.0,0,0)));
  vizImages.Add(ar::Sphere(0,0,0,0.05, ar::Color(1.0,0,0)));

  // Add a sphere to mark the external camera's position
  vizPoints.Add(ar::Sphere(arcam_position, 0.05, ar::Color(0,1,0)));

  // wait for user to exit
  std::cout << "Press enter when ready to exit..." << std::endl;
  std::cin.get();
  std::cout << "Shutting down..." << std::endl;

  vizPoints.Stop();
  vizImages.Stop();

  return 0;
}
