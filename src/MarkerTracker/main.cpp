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

#include <ARVisualizer.hpp>

#include "utils.hpp"
#include "CameraPoseEstimator.hpp"
#include "CameraIntrinsics.hpp"
#include "ArucoTracker.hpp"
#include "FloorDetector.hpp"

typedef pcl::PointXYZ PointT;

ar::ARVisualizer vizImages;

pcl::Grabber* interface;

// camera intrinsic parameters for Kinect RGB sensor
double camera_matrix[3][3] = {
  5.2921508098293293e+02, 0.0, 3.2894272028759258e+02,
  0.0, 5.2556393630057437e+02, 2.6748068171871557e+02,
  0.0, 0.0, 1.0
};

std::vector<double> camera_distortion = {
  2.6451622333009589e-01,
 -8.3990749424620825e-01,
 -1.9922302173693159e-03,
  1.4371995932897616e-03,
  9.1192465078713847e-01
};

CameraIntrinsics camera_params(camera_matrix, camera_distortion);
CameraPoseEstimator<PointT>* cameraPoseEstimator;

// initial camera configuration
double camera_up[3]       = { 0.0, -1.0, 0.0 }; // only correct when robot is not connected
double camera_forward[3]  = { 0.0,  0.0, 1.0 };
double camera_position[3] = { 0.0,  0.0, 0.0 };


/*
  This callback is called from PCL every time a new RGB image is available
*/
void image_callback (const boost::shared_ptr<openni_wrapper::Image>& image)
{
  cameraPoseEstimator->Update(image);

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
    vizImages.NotifyNewVideoFrame(image->getWidth(), image->getHeight(), img_data);
  }
}

/*
  This callback is called from PCL every time a new point cloud is available
*/
void cloud_cb (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    cameraPoseEstimator->Update(cloud);
}

int main(int argc, char* argv[])
{
  std::cout << "Opening sensor" << std::endl;
  interface = new pcl::OpenNIGrabber("", pcl::OpenNIGrabber::OpenNI_VGA_30Hz);

  // initialize marker tracker
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  auto marker = cv::aruco::GridBoard::create(1,2, 0.205, 0.008, dictionary);
  // auto marker = cv::aruco::GridBoard::create(2,2, 0.145, 0.01, dictionary);

  auto tracker = ArucoTracker(camera_params, marker);
  auto floorDetector = FloorDetector<PointT>(Eigen::Vector3f(0.0, -1.0, 0.0), 0.78);

  cameraPoseEstimator = new CameraPoseEstimator<PointT>(tracker, floorDetector);

  // subscribe to grabber's pointcloud callback
  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f_points = boost::bind(&cloud_cb, _1);
  interface->registerCallback(f_points);

  // subscribe to grabber's image callback so we get RGB data
  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> f_rgb = boost::bind (&image_callback, _1);
  interface->registerCallback (f_rgb);
  interface->start();

  // Start RGB data visualizer
  vizImages.Start("AR View");
  vizImages.SetCameraIntrinsics(camera_params.intrinsics);
  vizImages.SetCameraPose(camera_position, camera_forward, camera_up);

  // render box on marker
  double obox_center[3] = {0, 1.2, 0};
  ar::Box obox(obox_center, 0.25, 0.1, 0.1, ar::Color(0.2, 0.8, 1.0));
  auto obox_h = vizImages.Add(obox);
  ar::Transform obox_t;
  // render box on origin
  double box_center[3] = {0, 0, 0};
  ar::Box box(box_center, 0.25, 0.1, 0.1, ar::Color(1.0, 0.8, 0.2));
  auto box_h = vizImages.Add(box);
  ar::Transform box_t;

  cameraPoseEstimator->SetMarkerTransform(Eigen::Translation3f(Eigen::Vector3f(0.0f, -1.2f, 0.0F)) * Eigen::Affine3f::Identity());
  double cam_pos[3] = {0.0, 0.0, 0.0};
  double cam_rot_mat[3][3];
  while(1)
  {
    cameraPoseEstimator->GetPosition(cam_pos);
    cameraPoseEstimator->GetRotationMatrix(cam_rot_mat);
//    vizImages.SetCameraPose(cam_pos, cam_rot_mat);
    auto transform = cameraPoseEstimator->GetTransform();
    auto obox_trans = transform*Eigen::Vector3f(obox_center[0], obox_center[1], obox_center[2]);
    obox_t.translation[0] = obox_trans[0];obox_t.translation[1] = obox_trans[1];obox_t.translation[2] = obox_trans[2];
    cameraPoseEstimator->GetRotationMatrix(obox_t.rotation);

    auto box_trans = transform*Eigen::Vector3f(box_center[0], box_center[1], box_center[2]);
    box_t.translation[0] = box_trans[0];box_t.translation[1] = box_trans[1];box_t.translation[2] = box_trans[2];
    cameraPoseEstimator->GetRotationMatrix(box_t.rotation);

    vizImages.Update(obox_h, obox_t, true);
    vizImages.Update(box_h, box_t, true);
  }

  vizImages.Stop();
  interface->stop();

  return 0;
}
