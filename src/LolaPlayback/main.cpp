#include <pcl/common/transforms.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/image_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include <dirent.h>
#include <getopt.h>
#include <sys/stat.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <utility>

#include <ARVisualizer.hpp>

#include "StepPlannerLogReader.hpp"
#include "CameraPoseEstimator.hpp"
#include "CameraIntrinsics.hpp"
#include "FloorDetector.hpp"
#include "utils.hpp"

typedef pcl::PointXYZ PointT;

ar::ARVisualizer vizPoints;
ar::ARVisualizer vizImages;
ar::PointCloudData pointCloud(ar::PCL_PointXYZ);
ar::mesh_handle cloud_handle_pts;
ar::mesh_handle cloud_handle_img;
ar::mesh_handle camera_marker_handle;

/// TODO: This shouldn't have to be global...
std::map<unsigned int, std::vector<ar::mesh_handle>> img_objs;
std::map<unsigned int, std::vector<ar::mesh_handle>> pts_objs;

unsigned int current_timestep = 0;

std::vector<pcl::Grabber*> interfaces;

struct PlaybackParams
{
  std::string oniFile;
  std::string pcdDir;
  std::string stepLog;
  std::string poseLog;
};

// camera intrinsic parameters for Kinect RGB sensor
double camera_matrix[3][3] = {
  5.2921508098293293e+02, 0.0, 3.2894272028759258e+02,
  0.0, 5.2556393630057437e+02, 2.6748068171871557e+02,
  0.0, 0.0, 1.0
};

// distortion coefficients from Kinect RGB sensor
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
double camera_up[3]       = { 0.0,   0.0, 1.0 }; // only correct when robot is not connected
double camera_forward[3]  = {-1.0,  0.08,-0.3 };
double camera_position[3] = {-1.2,  -0.5, 6.3 };
double arcam_position[3]  = { 6.3, -1.45, 1.8 };


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
    Mat2Arr(flippedImage, img_data);

    // send image data to visualizer
    // (img_data is safe to delete after this; visualizer makes its own copy )
    vizImages.NotifyNewVideoFrame(image->getWidth(), image->getHeight(), img_data);
  }
}

void image_callback(cv::Mat& image)
{
  cameraPoseEstimator->Update(image);

  if (vizImages.IsRunning())
  {
    static unsigned char* img_data = new unsigned char[image.size().width * image.size().height * image.channels()];

    if(Mat2Arr(image, img_data))
    {
      vizImages.NotifyNewVideoFrame(image.size().width, image.size().height, img_data);
    }
  }
}


void cloud_cb (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  cameraPoseEstimator->Update(cloud);

  // only send data if the visualizer has started
  if (vizPoints.IsRunning())
  {
    Eigen::Affine3f cloud_transform = cameraPoseEstimator->GetTransform();
    pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT> ());
    pcl::transformPointCloud(*cloud, *transformed_cloud, cloud_transform);

    // give the visualizer the new points
    const PointT* data = &transformed_cloud->points[0];
    pointCloud.pointData = reinterpret_cast<const void*>(data);
    pointCloud.numPoints = transformed_cloud->size();
    vizPoints.Update(cloud_handle_pts, pointCloud);

    // move our camera marker to match the new estimated position
    static double camera_position[3] = {0, 0, 0};
    cameraPoseEstimator->GetPosition(camera_position);
    vizPoints.Update(camera_marker_handle, ar::Sphere(camera_position, 0.05, ar::Color(0,1,0)));
  }
}

bool parse_args(int argc, char* argv[], PlaybackParams* params)
{
  int c;

  if (argc < 2)
  {
    std::cout << "ERROR: You must provide at least one input!" << std::endl;
    return false;
  }

  while (1)
  {
    static struct option long_options[] =
      {
        {"oni",     required_argument, 0, 'n'},
        {"pcd",     required_argument, 0, 'p'},
        {"steplog", required_argument, 0, 's'},
        {"poselog", required_argument, 0, 'o'},
        {"help",    no_argument,       0, 'h'},
        {0, 0, 0, 0}
      };

      int option_index = 0;
      c = getopt_long (argc, argv, "n:p:s:o:h",
                       long_options, &option_index);

      /* Detect the end of the options. */
      if (c == -1)
        break;

      switch (c)
      {
        case 'n':
          params->oniFile = optarg;
          break;

        case 'p':
          params->pcdDir = optarg;
          // ensure directory path ends with '/' for convenience
          if (params->pcdDir.back() != '/')
          {
            params->pcdDir += "/";
          }
          break;

        case 's':
          params->stepLog = optarg;
          break;

        case 'o':
          params->poseLog = optarg;
          break;

        case 'h':
          return false;
          break;

        default:
          abort ();
      }
  }

  /* Print any remaining command line arguments (not options). */
  if (optind < argc)
  {
    std::cout << "Unrecognized arguments: ";
    while (optind < argc)
      std::cout << argv[optind++] << ", ";
    std::cout << std::endl;
    return false;
  }

  return true;
}

void printHelp(std::string name)
{
  std::cout << "Usage:" << std::endl;
  std::cout << "\t" << name << " [--oni ONI_input_file | --pcd PCD_input_dir] [--steplog stepplanner_log_file] [--poselog pose_log_file] [--help]" << std::endl;
  std::cout << std::endl;
  std::cout << "\t" << "      oni: Loads the given ONI file for playback of both pointcloud & rgb data" << std::endl;
  std::cout << "\t" << "      pcd: Loads all pcd and jpg files in the given directory for playback" << std::endl;
  std::cout << "\t" << "  steplog: Log file generated by stepplanner, for displaying footstep & obstacle data" << std::endl;
  std::cout << "\t" << "  poselog: Log of robot kinematic data, for displaying robot details & complete camera estimation" << std::endl;
  std::cout << "\t" << "     help: Display this message" << std::endl;
}

// hides all objects related to the given timestep
void HideTimeStep(unsigned int timestep)
{
  for (auto id : pts_objs[timestep])
  {
    vizPoints.SetVisibility(id, false);
  }
  for (auto id : img_objs[timestep])
  {
    vizImages.SetVisibility(id, false);
  }
}

// shows all objects related to the given timestep
void ShowTimeStep(unsigned int timestep)
{
  for (auto id : pts_objs[timestep])
  {
    vizPoints.SetVisibility(id, true);
  }
  for (auto id : img_objs[timestep])
  {
    vizImages.SetVisibility(id, true);
  }
}

void initStepLog(StepPlannerLogReader& log)
{
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
        ar::Color(c / (double)log.Entries().size(), footstep._foot == Left ? 1 : 0, 0.9, 0.9)
      );
      img_objs[entry._stamp].push_back(vizImages.Add(newQuad));
      vizImages.SetVisibility(img_objs[entry._stamp].back(), false);
      pts_objs[entry._stamp].push_back(vizPoints.Add(newQuad));
      vizPoints.SetVisibility(pts_objs[entry._stamp].back(), false);

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
            unsigned int nVerts = 16; // number of points for each path
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

            img_objs[entry._stamp].push_back(vizImages.Add(footpath));
            vizImages.SetVisibility(img_objs[entry._stamp].back(), false);
            pts_objs[entry._stamp].push_back(vizPoints.Add(footpath));
            vizPoints.SetVisibility(pts_objs[entry._stamp].back(), false);
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
            ar::Color(0, 0.4, 0.8, 0.9)
          );
          img_objs[entry._stamp].push_back(vizImages.Add(newSphere));
          vizImages.SetVisibility(img_objs[entry._stamp].back(), false);
          pts_objs[entry._stamp].push_back(vizPoints.Add(newSphere));
          vizPoints.SetVisibility(pts_objs[entry._stamp].back(), false);
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
          img_objs[entry._stamp].push_back(vizImages.Add(newCapsule));
          vizImages.SetVisibility(img_objs[entry._stamp].back(), false);
          pts_objs[entry._stamp].push_back(vizPoints.Add(newCapsule));
          vizPoints.SetVisibility(pts_objs[entry._stamp].back(), false);
          break;
        }
        default:
        {
          std::cout << "Unexpected obstacle type: " << obstacle._type << std::endl;
        }
      }
    }
  }
}

void initONI(std::string oni_file, std::vector<pcl::Grabber*>& interface_list)
{
  if (!checkPath(oni_file))
  {
    std::cout << "ONI file '" << oni_file << "' does not exist! Exiting..." << std::endl;
    abort();
  }

  pcl::Grabber* grabber = new pcl::ONIGrabber(oni_file, true, false);

  // subscribe to grabber's pointcloud callback
  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f_points = boost::bind(&cloud_cb, _1);
  grabber->registerCallback(f_points);
  // subscribe to grabber's image callback so we get RGB data
  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> f_rgb = boost::bind (static_cast<void (&)(const boost::shared_ptr<openni_wrapper::Image>&)> (&image_callback), _1);
  grabber->registerCallback (f_rgb);
  grabber->start();
  interface_list.push_back(grabber);
}

void initPCD(std::string pcd_dir, std::vector<pcl::Grabber*>& interface_list)
{
  if (!checkPath(pcd_dir))
  {
    std::cout << "Directory '" << pcd_dir << "' does not exist! Exiting..." << std::endl;
    abort();
  }

  std::vector<std::string> pcdFiles;

  // find all files ending with .pcd
  struct dirent** filenames;
  int res = scandir(pcd_dir.c_str(),
                    &filenames,
                    0,
                    alphasort);
  if (res < 0)
  {
      std::cout << "Error reading directory: " << pcd_dir << std::endl;
      abort();
  }
  else
  {
    while (res--)
    {
      std::string entry = std::string(filenames[res]->d_name);
      if (entry.substr(entry.find_last_of(".")+1) == "pcd")
      {
        pcdFiles.push_back(pcd_dir + entry);
      }
      free(filenames[res]);
    }
    free(filenames);
  }

  // make sure pcd files are in order (assumes uniform naming convention: "cloud_XXX.pcd")
  std::sort(pcdFiles.begin(), pcdFiles.end(), [](std::string a, std::string b) {
        unsigned long a_frame = std::stoul(a.substr(a.find_last_of("_")+1, a.find_last_of(".") - a.find_last_of("_")-1));
        unsigned long b_frame = std::stoul(b.substr(b.find_last_of("_")+1, b.find_last_of(".") - b.find_last_of("_")-1));
        return a_frame < b_frame;
    });

  pcl::Grabber* pcdGrabber = new pcl::PCDGrabber<PointT>(pcdFiles, 0, true);

  // subscribe to grabber's pointcloud callback
  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f_points = boost::bind(&cloud_cb, _1);
  pcdGrabber->registerCallback(f_points);
  // subscribe to grabber's image callback so we get RGB data
  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> f_rgb = boost::bind (static_cast<void (&)(const boost::shared_ptr<openni_wrapper::Image>&)> (&image_callback), _1);
  pcdGrabber->registerCallback (f_rgb);
  pcdGrabber->start();

  interfaces.push_back(pcdGrabber);
}

void initVisualizers()
{
  vizImages.Start("AR View");
  vizImages.SetCameraPose(arcam_position, camera_forward, camera_up);
  vizImages.SetCameraIntrinsics(camera_matrix);

  vizPoints.Start("PointCloud View");
  vizPoints.SetCameraPose(arcam_position, camera_forward, camera_up);

  // add an empty point cloud to visualizer, which we will upate each frame
  cloud_handle_pts = vizPoints.Add(pointCloud);
  cloud_handle_img = vizImages.Add(pointCloud);

  // Add some lines to mark the world axes at the origin
  ar::LinePath xAxis(0.004f, ar::Color(1,0,0));
  ar::LinePath yAxis(0.004f, ar::Color(0,1,0));
  ar::LinePath zAxis(0.004f, ar::Color(0,0,1));
  xAxis.points.push_back(0);   xAxis.points.push_back(0);   xAxis.points.push_back(0);
  xAxis.points.push_back(0.25);xAxis.points.push_back(0);   xAxis.points.push_back(0);
  yAxis.points.push_back(0);   yAxis.points.push_back(0);   yAxis.points.push_back(0);
  yAxis.points.push_back(0);   yAxis.points.push_back(0.25);yAxis.points.push_back(0);
  zAxis.points.push_back(0);   zAxis.points.push_back(0);   zAxis.points.push_back(0);
  zAxis.points.push_back(0);   zAxis.points.push_back(0);   zAxis.points.push_back(0.25);

  vizPoints.Add(xAxis); vizPoints.Add(yAxis); vizPoints.Add(zAxis);
  vizImages.Add(xAxis); vizImages.Add(yAxis); vizImages.Add(zAxis);

  // Add a sphere to mark the external camera's position
  camera_marker_handle = vizPoints.Add(ar::Sphere(arcam_position, 0.05, ar::Color(0,1,0)));
}

void cleanup()
{
  vizPoints.Stop();
  vizImages.Stop();

  for(auto interface : interfaces)
  {
    interface->stop();
  }
}

int main(int argc, char* argv[])
{
  PlaybackParams params;
  if (!parse_args(argc, argv, &params))
  {
    printHelp(argv[0]);
    return 0;
  }

  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
  auto marker = cv::aruco::GridBoard::create(1,2, 0.195, 0.01, dictionary);
  auto tracker = ArucoTracker(camera_params, marker);
  auto floorDetector = FloorDetector<PointT>(Eigen::Vector3f(0.0, -1.0, 0.0), 0.78);

  cameraPoseEstimator = new CameraPoseEstimator<PointT>(tracker, floorDetector);

  initVisualizers();
  auto ctrl_wnd = vizPoints.AddUIWindow("Control Panel", 200, 400);
  cv::VideoCapture* rgbImages = nullptr;

  // load either oni file or pcd, not both
  if (params.oniFile.size() > 0)
  {
    std::cout << "Loading ONI file from: " << params.oniFile << std::endl;
    initONI(params.oniFile, interfaces);
  }
  else if (params.pcdDir.size() > 0)
  {
    std::cout << "Loading PCD data from: " << params.pcdDir << std::endl;
    initPCD(params.pcdDir, interfaces);
    rgbImages = new cv::VideoCapture(params.pcdDir + "image_%04d.jpg");
  }

  StepPlannerLogReader* log = nullptr;
  if (params.stepLog.size() > 0)
  {
    std::cout << "Loading step planner log from file: " << params.stepLog << std::endl;
    log = new StepPlannerLogReader(params.stepLog);
    std::cout << "Loaded " << log->Entries().size() << " log entries" << std::endl;
    std::cout << "Preparing geometry for rendering...";
    initStepLog(*log);
    std::cout << "done!" << std::endl;

    // make sure first timestep is visible
    ShowTimeStep(log->Entry(0)._stamp);
  }


  auto timestep_slider = log ? ctrl_wnd->AddSliderInt("Timestep", 0, log->Entries().size()-1, 0) : 0;
  auto allsteps_chkbx  = log ? ctrl_wnd->AddCheckBox("Show All Steps", false) : 0;
                         ctrl_wnd->AddSeparator();
  auto exit_btn        = ctrl_wnd->AddButton("Exit");


  int frame_time = 33; // ~33ms for ~30fps
  // main event loop
  while (!ctrl_wnd->GetButtonState(exit_btn))
  {
    std::this_thread::sleep_for((std::chrono::milliseconds)frame_time); // todo: allow UI updates to happen faster than image/point/log frame updates
    if (log)
    {
      static bool showAll = false;

      bool allsteps_state = ctrl_wnd->GetCheckBoxState(allsteps_chkbx);
      if (!showAll && allsteps_state)
      {
        showAll = true;
        for (auto& entry : log->Entries())
        {
          ShowTimeStep(entry._stamp);
        }
      }
      else if (showAll && !allsteps_state)
      {
        showAll = false;
        for (auto& entry : log->Entries())
        {
          if (entry._stamp != current_timestep)
          {
            HideTimeStep(entry._stamp);
          }
        }
      }
      else if (!showAll)
      {
        int new_timestep_idx = ctrl_wnd->GetSliderIntValue(timestep_slider);
        if (log->Entry(new_timestep_idx)._stamp != current_timestep)
        {
          HideTimeStep(current_timestep);
          current_timestep = log->Entry(new_timestep_idx)._stamp;
          std::cout << "Showing timestep: " << current_timestep << std::endl;
          ShowTimeStep(current_timestep);
        }
      }
    }

    // if we're reading RGB images from a directory, show the next one
    if (rgbImages)
    {
      static cv::Mat img;
      if (rgbImages->read(img))
      {
        image_callback(img);
      }
      else
      {
        rgbImages->set(CV_CAP_PROP_POS_FRAMES, 0);
      }
    }

    // if we're reading ONI or PCD files, show the next frame
    for (auto& interface : interfaces)
    {
      interface->start();
    }
  }

  std::cout << "Shutting down..." << std::endl;
  cleanup();
  return 0;
}
