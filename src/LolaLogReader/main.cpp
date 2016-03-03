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

#include <am2b-ar/ARVisualizer.hpp>

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
  Copies the raw data from the given cv::Mat to dst_array
  Based on the sample code from: docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html

  Returns True on success, False if the conversion could not be performed

  NOTE: dst_array must already be allocated with enough space to store the image!
*/
bool Mat2Arr(cv::Mat inputImage, unsigned char* dst_array)
{
  if (inputImage.depth() != CV_8U)
  {
    std::cout << "ERROR: Expected input image type: CV_8U! Got cv type: " << inputImage.depth() << std::endl;
    return false;
  }

  int channels = inputImage.channels();
  int nRows = inputImage.rows;
  int nCols = inputImage.cols * channels;

  if (inputImage.isContinuous())
  {
    nCols *= nRows;
    nRows = 1;
  }

  int p = 0;
  for (int i = 0; i < nRows; i++)
  {
    uint8_t* rowPtr = inputImage.ptr<uchar>(i);
    for (int j = 0; j < nCols; j += channels)
    {
      // note the order here, because the cv::Mat is BGR and we need RGB
      dst_array[p + 0] = rowPtr[j + 2]; // R
      dst_array[p + 1] = rowPtr[j + 1]; // G
      dst_array[p + 2] = rowPtr[j + 0]; // B

      p += 3;
    }
  }

  return true;
}

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

std::vector< std::pair< int, std::vector< double > > > parse_step_log(std::string filename)
{
  std::ifstream infile(filename);
  std::string line;
  std::string splitLine[36];

  std::vector< std::pair< int, std::vector< double > > > output;

  while (std::getline(infile, line))
  {
    if ( line.find("#") == 0 )  // skip commented lines
      continue;

    std::stringstream ss(line);
    int i = 0;
    while ( ss.good() && i < 36 )
    {
      ss >> splitLine[i];
      i++;
    }

    int obstacle_type = std::stoi(splitLine[21]);
    std::vector<double> obstacle_params;
    for (int i = 22; i <= 35; i++)
    {
      obstacle_params.push_back(std::stod(splitLine[i]));
    }

    std::vector<double> footstep_params;
    for (int i = 8; i <=10; i++)
    {
      footstep_params.push_back(std::stod(splitLine[i]));
    }

    output.push_back(std::make_pair(7, footstep_params));
    output.push_back(std::make_pair(obstacle_type, obstacle_params));
  }

  return output;
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
  // vizPoints.SetCameraPose(camera_position, camera_forward, camera_up);

  // add an empty point cloud to visualizer, which we will upate each frame
  cloud_handle_pts = vizPoints.Add(pointCloud);
  cloud_handle_img = vizImages.Add(pointCloud);

  auto log = parse_step_log(step_plan);
  for (auto entry : log)
  {
    switch (entry.first)
    {
      case 0: // Sphere
      {
        ar::Sphere newSphere(
          entry.second[5], entry.second[6], entry.second[7], // center
          entry.second[2], // radius
          ar::Color(0, 0.4, 0.8, 0.5)
        );
        vizImages.Add(newSphere);
        vizPoints.Add(newSphere);
        break;
      }
      case 1: // capsule
      {
        ar::Capsule newCapsule(
          entry.second[5], entry.second[6], entry.second[7], // center1
          entry.second[8], entry.second[9], entry.second[10], // center2
          entry.second[2], // radius
          ar::Color(0.6, 0.2, 0.6, 0.5)
        );
        vizImages.Add(newCapsule);
        vizPoints.Add(newCapsule);
        break;
      }
      case 7: // footstep
      {
        double quadNormal[3] = {0, 0, 1};
        ar::Quad newQuad(
          entry.second[0], entry.second[1], entry.second[2],
          quadNormal,
          0.15, 0.15,
          ar::Color(entry.second[0] / 4.0, 1, 0, 0.3)
        );
        vizImages.Add(newQuad);
        vizPoints.Add(newQuad);
        break;
      }
      default:
      {
        std::cout << "Unknown obstacle type: " << entry.first << std::endl;
        break;
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

