#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <glm/gtx/euler_angles.hpp>

#include <map>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <utility>

#include <ARVisualizer.hpp>

#include <tclap/CmdLine.h>

#include "Obstacle.hpp"
#include "VisionListener.hpp"
#include "PoseListener.hpp"
#include "utils.hpp"
#include "CameraPoseEstimator.hpp"
#include "CameraIntrinsics.hpp"
#include "ArucoTracker.hpp"
#include "FloorDetector.hpp"

typedef pcl::PointXYZ PointT;

bool recording = false; /// TODO: Don't keep this here...
std::string log_dir;
double time_elapsed = 0.0f;

ar::ARVisualizer vizImages;
std::map<int, ar::mesh_handle> obstacle_id_map;
std::map<int, ar::mesh_handle> surface_id_map;
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

struct ParsedParams
{
    unsigned int visionPort = 0; // port to receive vision messages on
    unsigned int posePort   = 0; // port to receive pose data on
    bool record = false;          // whether or not to record a log
};

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

    if (recording)
    {
        static unsigned int img_idx = 0;
        std::stringstream filename;
        filename << "image_" << std::setfill('0') << std::setw(4) << img_idx << ".png";
        cv::imwrite(log_dir + "/" + filename.str(), bgrImage);
        img_idx++;

        std::ofstream metalog;
        metalog.open(log_dir + "/metalog.txt", std::ofstream::app);
        metalog << time_elapsed << "\t" << filename.str() << std::endl;
        metalog.close();
    }
  }
}

/*
  This callback is called from PCL every time a new point cloud is available
*/
void cloud_cb (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    cameraPoseEstimator->Update(cloud);

    if (recording)
    {
        static unsigned int cloud_idx = 0;
        std::stringstream filename;
        filename << "cloud_" << std::setfill('0') << std::setw(4) << cloud_idx << ".pcd";
        pcl::io::savePCDFileBinary(log_dir + "/" + filename.str(), *cloud);
        cloud_idx++;

        std::ofstream metalog;
        metalog.open(log_dir + "/metalog.txt", std::ofstream::app);
        metalog << time_elapsed << "\t" << filename.str() << std::endl;
        metalog.close();
    }
}

/*
 * This callback is called when we receive a new robot pose
*/
void pose_cb (HR_Pose_Red* new_pose, CameraPoseEstimator<PointT>* cameraPoseEstimator)
{
  std::cout << "Received new robot pose!" << std::endl;
    /// TODO: Set marker->world transform according to new pose data
//  cameraPoseEstimator->SetMarkerTransform(Eigen::Translation3f(marker_pos) * Eigen::Affine3f(marker_rot));

  if (recording)
  {
    std::ofstream tf_out;
    tf_out.open(log_dir + "/params.txt", std::ofstream::app);
    tf_out << time_elapsed << "\t";
    for (size_t i = 0; i < 3; ++i)
    {
        tf_out << new_pose->t_wr_cl[i] << "\t";
    }
    for (size_t i = 0; i < 9; ++i)
    {
        tf_out << new_pose->R_wr_cl[i] << "\t";
    }
    for (size_t i = 0; i < 3; ++i)
    {
        tf_out << new_pose->t_stance_odo[i] << "\t";
    }
    tf_out << new_pose->phi_z_odo << "\t";
    tf_out << new_pose->stance << "\t";
    tf_out << new_pose->stamp;
    tf_out << std::endl;
    tf_out.close();
  }
}

/*
 * Adds a new obstacle to a visualization instance
 */
void addObstacle(Obstacle obstacle, ar::ARVisualizer* viz)
{
  ar::mesh_handle viz_id;
  switch (obstacle._type)
  {
    case Sphere:
    {
      ar::Sphere newSphere(
        obstacle._coords[0][0], obstacle._coords[0][1], obstacle._coords[0][2], // center
        obstacle._radius,
        ar::Color(0, 0.4, 0.8, 0.8)
      );
      viz_id = viz->Add(newSphere);
      break;
    }
    case Capsule:
    {
      ar::Capsule newCapsule(
        obstacle._coords[0][0], obstacle._coords[0][1], obstacle._coords[0][2], // center1
        obstacle._coords[1][0], obstacle._coords[1][1], obstacle._coords[2][2],
        obstacle._radius,
        ar::Color(0.6, 0.2, 0.6, 0.9)
      );
      viz_id = viz->Add(newCapsule);
      break;
    }
    default:
    {
      std::cout << "Unexpected obstacle type: " << obstacle._type << std::endl;
    }
  }
  obstacle_id_map.insert(std::pair<int, ar::mesh_handle>(obstacle._id, viz_id));
}

/*
 * Updates an existing obstacle
 */
void updateObstacle(Obstacle obstacle, ar::ARVisualizer* viz)
{
  switch (obstacle._type)
  {
    case Sphere:
    {
      ar::Sphere newSphere(
        obstacle._coords[0][0], obstacle._coords[0][1], obstacle._coords[0][2], // center
        obstacle._radius,
        ar::Color(0, 0.4, 0.8, 0.8)
      );
      viz->Update(obstacle_id_map[obstacle._id], newSphere);
      break;
    }
    case Capsule:
    {
	  ar::Capsule newCapsule(
        obstacle._coords[0][0], obstacle._coords[0][1], obstacle._coords[0][2], // center1
        obstacle._coords[1][0], obstacle._coords[1][1], obstacle._coords[1][2], // center2
        obstacle._radius, // radius
        ar::Color(0.6, 0.2, 0.6, 0.9)
      );
      viz->Update(obstacle_id_map[obstacle._id], newCapsule);
      break;
    }
    default:
    {
      std::cout << "Unexpected obstacle type: " << obstacle._type << std::endl;
    }
  }
}

/*
 * Removes an existing obstacle from visualization
 */
void deleteObstacle(int obstacle_id, ar::ARVisualizer* viz)
{
  viz->Remove(obstacle_id_map[obstacle_id]);
  obstacle_id_map.erase(obstacle_id);
}

/*
 * This callback is called when an obstacle is detected, changed, or removed
*/
void obstacle_cb (am2b_iface::ObstacleMessage* message, ar::ARVisualizer* viz)
{
  std::cout << "Received obstacle: " << std::endl;
  std::cout << *message << std::endl;

  if (message->action == am2b_iface::SET_SSV)
  {
    addObstacle(Obstacle(message), viz);
  }
  else if (message->action == am2b_iface::MODIFY_SSV)
  {
    updateObstacle(Obstacle(message), viz);
  }
  else if (message->action == am2b_iface::REMOVE_SSV_ONLY_PART)
  {
    deleteObstacle(message->model_id, viz);
  }
  else if (message->action == am2b_iface::REMOVE_SSV_WHOLE_SEGMENT)
  {
    deleteObstacle(message->model_id, viz);
  }
  else
  {
    std::cout << "[vision] Received unknown obstacle action: " << message->action << std::endl;
  }

  if (recording)
  {
    std::ofstream obstaclelog;
    obstaclelog.open(log_dir + "/vision_data.txt", std::ofstream::app);
    obstaclelog << time_elapsed << "\tobstacle { " << *message << " }" << std::endl;
    obstaclelog.close();
  }
}

/*
 * Adds a new surface polygon to the given visualization
 */
 void addSurface(float* vertices, unsigned int count, unsigned int id, ar::ARVisualizer* viz)
 {
   ar::Polygon new_surface(0, 0); /// HACK: work around ARVisualizer api...
   new_surface.points = vertices;
   new_surface.numPoints = count;
   new_surface.color = ar::Color(0, 1, 0);
   ar::mesh_handle new_id = viz->Add(new_surface);
   surface_id_map.insert(std::pair<int, ar::mesh_handle>(id, new_id));
 }

 /*
  * Updates an existing surface
  */
void updateSurface(float* vertices, unsigned int count, unsigned int id, ar::ARVisualizer* viz)
{
  ar::Polygon new_surface(0, 0); /// HACK: work around ARVisualizer api...
  new_surface.points = vertices;
  new_surface.numPoints = count;
  new_surface.color = ar::Color(0, 1, 0);
  viz->Update(surface_id_map[id], new_surface);
}

/*
 * Deletes a surface
 */
 void deleteSurface(int surface_id, ar::ARVisualizer* viz)
 {
   viz->Remove(surface_id_map[surface_id]);
   surface_id_map.erase(surface_id);
 }

/*
 * This callback is called when a surface is detected, changed, or removed
 */
void surface_cb (am2b_iface::SurfaceMessage* message, ar::ARVisualizer* viz)
{
  std::cout << "Received surface" << std::endl;

  if (message->action == am2b_iface::SET_SURFACE)
  {
      addSurface(message->vertices, 8, message->id, viz);
  }
  else if (message->action == am2b_iface::MODIFY_SURFACE)
  {
      updateSurface(message->vertices, 8, message->id, viz);
  }
  else if (message->action == am2b_iface::REMOVE_SURFACE)
  {
      deleteSurface(message->id, viz);
  }
  else
  {
      std::cout << "[vision] Received unknown surface action: " << message->action << std::endl;
  }

  if (recording)
  {
    std::ofstream obstaclelog;
    obstaclelog.open(log_dir + "/vision_data.txt", std::ofstream::app);
    obstaclelog << time_elapsed << "\tsurface { " << *message << " }" << std::endl;
    obstaclelog.close();
  }
}

/*
 * This callback is called when a new RGB image is received from lepp
 */
void lepp_image_cb (am2b_iface::RGBMessage* message)
{
  if (recording)
  {
    /// TODO: Convert message->pixels to cv::Mat and save to disk
  }
}

/*
 * This callback is called when a pointcloud is received from lepp
 */
void lepp_cloud_cb (am2b_iface::PointCloudMessage* message)
{
  if (recording)
  {
      /// TODO: Convert message->data to pcl cloud and save to disk
  }
}

bool parse_args(int argc, char* argv[],  ParsedParams* params)
{
  try {
    TCLAP::CmdLine cmd("Lola AR View", ' ', "0.1");
    TCLAP::ValueArg<unsigned int> visionPort("l", "lepp",
                                             "Port to listen on for vision data from lepp",
                                             false, 0, "unsigned int");
    TCLAP::ValueArg<unsigned int> posePort("p", "pose",
                                             "Port to listen on for robot pose data",
                                             false, 0, "unsigned int");
    TCLAP::SwitchArg record("r", "record",
                            "If set, all rendered data will also be saved to disk.",
                            cmd, false);
    cmd.add(visionPort);
    cmd.add(posePort);
    cmd.parse(argc, argv);

    params->visionPort = visionPort.getValue();
    params->posePort   = posePort.getValue();
    params->record     = record.getValue();
  }
  catch (TCLAP::ArgException& e) {
      std::cerr << "Error in argument " << e.argId() << ": " << e.error() << std::endl;
      return false;
  }
  return true;
}

int main(int argc, char* argv[])
{
  ParsedParams params;
  if (!parse_args(argc, argv, &params))
  {
    return 0;
  }

  auto start_time = std::chrono::system_clock::now();
  recording = params.record;
  if (recording)
  {
    log_dir = makeStampedDirectory("recording_");
    std::ofstream tf_out(log_dir + "/params.txt");
    if (tf_out.is_open())
    {
      tf_out << "# local_time\tt_wr_cl[0]\tt_wr_cl[1]\tt_wr_cl[2]\t"
             << "R_wr_cl[0][0]\tR_wr_cl[0][1]\tR_wr_cl[0][2]\t"
             << "R_wr_cl[1][0]\tR_wr_cl[1][1]\tR_wr_cl[1][2]\t"
             << "R_wr_cl[2][0]\tR_wr_cl[2][1]\tR_wr_cl[2][2]\t"
             << "t_stance_odo[0]\tt_stance_odo[1]\tt_stance_odo[2]\t"
             << "phi_z_odo\tstance\tstamp"
             << std::endl;
      tf_out.close();

      std::ofstream metalog(log_dir + "/metalog.txt");
      if (metalog.is_open())
      {
        metalog << "# this file tracks relative creation times of other log files" << std::endl;
      }
      metalog.close();

      std::ofstream obstaclelog(log_dir + "/vision_data.txt");
      if (obstaclelog.is_open())
      {
        obstaclelog << "# This file tracks obstacle events and the local time they are received at" << std::endl;
      }
      obstaclelog.close();
    }
  }

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

  PoseListener pl(params.posePort, true);
  if (params.posePort > 0)
  {
    pl.onError([](std::string err)->void{std::cout << "ERROR [pose]: " << err << std::endl;});
    pl.onNewPose(std::bind(&pose_cb, std::placeholders::_1, cameraPoseEstimator));
    pl.listen();
  }

  VisionListener vl(params.visionPort, true);
  if (params.visionPort > 0)
  {
    vl.onConnect([](std::string host)->void{std::cout << "[vision] Connected to: " << host << std::endl;});
    vl.onDisconnect([](std::string host)->void{std::cout << "[vision] Disconnected from: " << host << std::endl;});
    vl.onObstacleMessage(std::bind(&obstacle_cb, std::placeholders::_1, &vizImages));
    vl.onSurfaceMessage(std::bind(&surface_cb, std::placeholders::_1, &vizImages));
    vl.onRGBMessage(&lepp_image_cb);
    vl.onPointCloudMessage(&lepp_cloud_cb);
    vl.listen();
  }

  cameraPoseEstimator->SetMarkerTransform(Eigen::Translation3f(marker_pos) * Eigen::Affine3f(marker_rot));
  double cam_pos[3] = {0.0, 0.0, 0.0};
  double cam_rot_mat[3][3];
  while(1)
  {
    std::chrono::duration<double> diff = (std::chrono::system_clock::now() - start_time);
    time_elapsed = diff.count();
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
