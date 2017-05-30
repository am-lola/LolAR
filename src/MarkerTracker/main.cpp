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

  double box_center_0[3] = {0,   0,   0};
  double box_center_x[3] = {0.2, 0,   0};
  double box_center_y[3] = {0,   0.2, 0};
  double box_center_z[3] = {0,   0, 0.2};
  double box_center_m[3] = {0,   0,   0};

  ar::Box box_0(box_center_0, 0.15, 0.1, 0.1, ar::Color(1.0, 0.8, 0.2));
  ar::Box box_x(box_center_x, 0.15, 0.1, 0.1, ar::Color(1.0, 0.0, 0.0));
  ar::Box box_y(box_center_y, 0.15, 0.1, 0.1, ar::Color(0.0, 1.0, 0.0));
  ar::Box box_z(box_center_z, 0.15, 0.1, 0.1, ar::Color(0.0, 0.0, 1.0));
  ar::Box box_m(box_center_m, 0.15, 0.1, 0.1, ar::Color(0.2, 1.0, 0.8));

  ar::Box box_0_(box_center_0, 0.15, 0.1, 0.1, ar::Color(1.0, 1.0, 0.5));
  ar::Box box_x_(box_center_x, 0.15, 0.1, 0.1, ar::Color(1.0, 0.5, 0.5));
  ar::Box box_y_(box_center_y, 0.15, 0.1, 0.1, ar::Color(0.5, 1.0, 0.5));
  ar::Box box_z_(box_center_z, 0.15, 0.1, 0.1, ar::Color(0.5, 0.5, 1.0));
  ar::Box box_m_(box_center_m, 0.15, 0.1, 0.1, ar::Color(0.8, 1.0, 1.0));

  auto o_h = vizImages.Add(box_0);
  auto x_h = vizImages.Add(box_x);
  auto y_h = vizImages.Add(box_y);
  auto z_h = vizImages.Add(box_z);
  auto m_h = vizImages.Add(box_m);

  auto o_h_ = vizImages.Add(box_0_);
  auto x_h_ = vizImages.Add(box_x_);
  auto y_h_ = vizImages.Add(box_y_);
  auto z_h_ = vizImages.Add(box_z_);
  auto m_h_ = vizImages.Add(box_m_);

  ar::Transform t_0;
  ar::Transform t_x;
  ar::Transform t_y;
  ar::Transform t_z;
  ar::Transform t_m;

  ar::Transform t_0_;
  ar::Transform t_x_;
  ar::Transform t_y_;
  ar::Transform t_z_;
  ar::Transform t_m_;

  Eigen::Vector3f marker_pos(0.0f, -1.2f, 0.0f);
  Eigen::AngleAxisf marker_rot(M_PI/4.0f, Eigen::Vector3f::UnitZ());
  auto marker_rot_mat = marker_rot.matrix();

  cameraPoseEstimator->SetMarkerTransform(Eigen::Translation3f(marker_pos) * Eigen::Affine3f(marker_rot));
  double cam_pos[3] = {0.0, 0.0, 0.0};
  double cam_rot_mat[3][3];
  while(1)
  {
    cameraPoseEstimator->GetPosition(cam_pos);
    cameraPoseEstimator->GetRotationMatrix(cam_rot_mat);
    vizImages.SetCameraPose(cam_pos, cam_rot_mat);

    std::cout << "----------------" << std::endl;
    for (size_t i = 0; i < 3; i++)
    {
      std::cout << "[" << cam_rot_mat[i][0] << " " << cam_rot_mat[i][1] << " " << cam_rot_mat[i][2] << "]" << std::endl;
    }
    std::cout << "----------------" << std::endl;
    // // get current camera transform
    // auto transform = cameraPoseEstimator->GetTransform();
    // auto cam2markerTransform = cameraPoseEstimator->GetCam2MarkerTransform();
    //
    // // update boxes translation
    // auto box_0_t = transform * Eigen::Vector3f(box_center_0[0], box_center_0[1], box_center_0[2]);
    // t_0.translation[0] = box_0_t[0];t_0.translation[1] = box_0_t[1];t_0.translation[2] = box_0_t[2];
    // auto box_x_t = transform * Eigen::Vector3f(box_center_x[0], box_center_x[1], box_center_x[2]);
    // t_x.translation[0] = box_x_t[0];t_x.translation[1] = box_x_t[1];t_x.translation[2] = box_x_t[2];
    // auto box_y_t = transform * Eigen::Vector3f(box_center_y[0], box_center_y[1], box_center_y[2]);
    // t_y.translation[0] = box_y_t[0];t_y.translation[1] = box_y_t[1];t_y.translation[2] = box_y_t[2];
    // auto box_z_t = transform * Eigen::Vector3f(box_center_z[0], box_center_z[1], box_center_z[2]);
    // t_z.translation[0] = box_z_t[0];t_z.translation[1] = box_z_t[1];t_z.translation[2] = box_z_t[2];
    // auto box_m_t = transform * Eigen::Vector3f(-marker_pos[0], -marker_pos[1], -marker_pos[2]);
    // t_m.translation[0] = box_m_t[0];t_m.translation[1] = box_m_t[1];t_m.translation[2] = box_m_t[2];
    //
    // auto box_0_t_ = cam2markerTransform * Eigen::Vector3f(box_center_0[0], box_center_0[1], box_center_0[2]);
    // t_0_.translation[0] = box_0_t_[0];t_0_.translation[1] = box_0_t_[1];t_0_.translation[2] = box_0_t_[2];
    // auto box_x_t_ = cam2markerTransform * Eigen::Vector3f(box_center_x[0], box_center_x[1], box_center_x[2]);
    // t_x_.translation[0] = box_x_t_[0];t_x_.translation[1] = box_x_t_[1];t_x_.translation[2] = box_x_t_[2];
    // auto box_y_t_ = cam2markerTransform * Eigen::Vector3f(box_center_y[0], box_center_y[1], box_center_y[2]);
    // t_y_.translation[0] = box_y_t_[0];t_y_.translation[1] = box_y_t_[1];t_y_.translation[2] = box_y_t_[2];
    // auto box_z_t_ = cam2markerTransform * Eigen::Vector3f(box_center_z[0], box_center_z[1], box_center_z[2]);
    // t_z_.translation[0] = box_z_t_[0];t_z_.translation[1] = box_z_t_[1];t_z_.translation[2] = box_z_t_[2];
    // auto box_m_t_ = cam2markerTransform * Eigen::Vector3f(-marker_pos[0], -marker_pos[1], -marker_pos[2]);
    // t_m_.translation[0] = box_m_t_[0];t_m_.translation[1] = box_m_t_[1];t_m_.translation[2] = box_m_t_[2];
    //
    // // update boxes orientation
    // cameraPoseEstimator->GetRotationMatrix(t_0.rotation);
    // cameraPoseEstimator->GetRotationMatrix(t_x.rotation);
    // cameraPoseEstimator->GetRotationMatrix(t_y.rotation);
    // cameraPoseEstimator->GetRotationMatrix(t_z.rotation);
    //
    // cameraPoseEstimator->GetCam2MarkerRotationMatrix(t_0_.rotation);
    // cameraPoseEstimator->GetCam2MarkerRotationMatrix(t_x_.rotation);
    // cameraPoseEstimator->GetCam2MarkerRotationMatrix(t_y_.rotation);
    // cameraPoseEstimator->GetCam2MarkerRotationMatrix(t_z_.rotation);
    //
    //
    // for(size_t i = 0; i < 3; i++)
    // {
    //   for(size_t j = 0; j < 3; j++)
    //   {
    //     t_m.rotation[i][j] = marker_rot_mat(i,j);
    //     t_m_.rotation[i][j] = marker_rot_mat(i,j);
    //   }
    // }
    //
    // // update boxes in visualizer
    // vizImages.Update(o_h, t_0, true);
    // vizImages.Update(x_h, t_x, true);
    // vizImages.Update(y_h, t_y, true);
    // vizImages.Update(z_h, t_z, true);
    // vizImages.Update(m_h, t_m, true);
    // vizImages.Update(o_h_, t_0_, true);
    // vizImages.Update(x_h_, t_x_, true);
    // vizImages.Update(y_h_, t_y_, true);
    // vizImages.Update(z_h_, t_z_, true);
    // vizImages.Update(m_h_, t_m_, true);
  }

  vizImages.Stop();
  interface->stop();

  return 0;
}
