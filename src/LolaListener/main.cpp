#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netdb.h>
#include <unistd.h>
#include <signal.h>

#include <map>
#include <functional>
#include <algorithm>
#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <iface_msg.hpp>
#include <iface_sig_msg.hpp>
#include <iface_ps.hpp>
#include <iface_stepseq.hpp>
#include <iface_vis.h>
#include <iface_vision_msg.hpp>

#include <ARVisualizer.hpp>

#include <tclap/CmdLine.h>

#include "Obstacle.hpp"
#include "Footstep.hpp"
#include "sock_utils.hpp"
#include "PoseListener.hpp"
#include "VisionListener.hpp"
#include "FootstepListener.hpp"

using namespace std::placeholders;

/**
 * A client to collect and visualize data from LOLA
 *
 * Listens on the given port for any incoming data from the vision
 * system, and adds it to a local visualization.
 */

// maximum # of bytes expected in any incoming packet
#define BUFLEN 2048
ar::ARVisualizer viz;
ar::PointCloudData pointcloud(ar::PCL_PointXYZ);
ar::mesh_handle cloud_handle;
bool shuttingDown = false;
// maps lepp IDs to visualizer IDs
std::map<int, ar::mesh_handle> obstacle_id_map;
std::map<int, ar::mesh_handle> surface_id_map;

struct ParsedParams
{
  unsigned int obstaclePort = 0; // port number to listen on for obstacle data
  unsigned int footstepPort = 0; // port number to listen on for footstep data
  std::string  footstepHost = "";// host to connect to for footstep data
  unsigned int posePort     = 0; // port number to listen on for pose data
  float largeViewer = 1; // extend the viewer to include an area bigger than the vision message
  bool verbose = false;
};

// creates a directory with a name based on current timestamp
std::string static makeStampedDirectory(std::string prefix)
{
  auto now = std::time(nullptr);
  char buf[sizeof("YYYY-MM-DD_HHMMSS")];
  std::string dirname = prefix + std::string(buf, buf + std::strftime(buf, sizeof(buf), "%F_%H%M%S", std::gmtime(&now)));
#ifdef USE_BOOST_FILESYSTEM
  if (boost::filesystem::create_directories(dirname))
    return dirname;
  else
    throw std::runtime_error("Could not create directory: " + dirname);
#else
  const int dir_err = mkdir(dirname.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (-1 == dir_err)
  {
    throw std::runtime_error("Could not create directory: " + dirname + "\n\t\t" + strerror(errno));
  }
  return dirname;
#endif
}

bool parse_args(int argc, char* argv[], ParsedParams* params)
{
  if (argc <= 1)
  {
    std::cerr << "Error: You must specify at least one data source!" << std::endl;
    return false;
  }

  try {
    TCLAP::CmdLine cmd("Lola Listener", ' ', "0.1");
    TCLAP::ValueArg<unsigned int> obstaclePort("o", "obstacleport",
                                               "Port to listen on for obstacle data",
                                               false, 0, "unsigned int");
    TCLAP::ValueArg<std::string>  footstepHost("f", "footstephost",
                                               "Hostname to connect to for footstep data, in the form: ' ip:port '. With lola, -f 192.168.0.7:61448",
                                               false, "", "hostname:port  or  ip:port");
    TCLAP::ValueArg<unsigned int> posePort("p", "poseport",
                                                "Port to listen on for pose data. With lola_state_server, -p 53249",
                                                false, 0, "unsigned int");
    TCLAP::ValueArg<float> largeViewer("l", "largeviewer",
                                            "Factor to extend the viewer in order to include an area bigger than the vision message. Useful, for example, for showing footsteps in first person AR. Default: 1, Typical value: 2",
                                            false, 1, "float");
    TCLAP::SwitchArg verbose("v", "verbose",
                             "Verbose output", cmd, false);
    cmd.add(obstaclePort);
    cmd.add(footstepHost);
    cmd.add(posePort);
    cmd.add(largeViewer);

    cmd.parse(argc, argv);

    params->obstaclePort = obstaclePort.getValue();
    params->posePort = posePort.getValue();
    params->largeViewer = largeViewer.getValue();
    params->verbose = verbose.getValue();

    size_t delimpos;
    if ( (delimpos = footstepHost.getValue().find_last_of(":")) != std::string::npos )
    {
      params->footstepHost = footstepHost.getValue().substr(0, delimpos);
      params->footstepPort = std::stoul(footstepHost.getValue().substr(delimpos+1));
    }

  } catch (TCLAP::ArgException &e) {
    std::cerr << "Error in argument " << e.argId() << ": " << e.error() << std::endl;
    return false;
  }

  return true;
}

void renderFootstep(Footstep footstep)
{
  double quadNormal[3] = {0, 0, 1}; // should be parallel to ground
  ar::Color leftColor(0.6, 0.8, 0.4);
  ar::Color rightColor(0.0, 0.55, 1.0);
  ar::Quad newQuad(
    footstep._position[0], footstep._position[1], footstep._position[2],
    quadNormal,
    0.15, 0.15,
    footstep._foot == Left ? leftColor : rightColor
  );
  viz.Add(newQuad);
}

void renderObstacle(Obstacle obstacle)
{
  ar::mesh_handle viz_id;
  switch (obstacle._type)
  {
    case Sphere:
    {
      ar::Sphere newSphere(
        obstacle._coords[0][0], obstacle._coords[0][1], obstacle._coords[0][2], // center
        obstacle._radius, // radius
        ar::Color(0, 0.4, 0.8, 0.9)
      );
      viz_id = viz.Add(newSphere);
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
      viz_id = viz.Add(newCapsule);
      break;
    }
    default:
    {
      std::cout << "Unexpected obstacle type: " << obstacle._type << std::endl;
    }
  }
  obstacle_id_map.insert(std::pair<int, ar::mesh_handle>(obstacle._id, viz_id));
}

void updateObstacle(Obstacle obstacle)
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
      viz.Update(obstacle_id_map[obstacle._id], newSphere);
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
      viz.Update(obstacle_id_map[obstacle._id], newCapsule);
      break;
    }
    default:
    {
      std::cout << "Unexpected obstacle type: " << obstacle._type << std::endl;
    }
  }
}

void deleteObstacle(int obstacle_id)
{
  viz.Remove(obstacle_id_map[obstacle_id]);
  obstacle_id_map.erase(obstacle_id);
}

void renderSurface(float* vertices, unsigned int count, unsigned int id)
{
  ar::Polygon new_surface(0, 0); /// HACK: work around ARVisualizer api...
  new_surface.points = vertices;
  new_surface.numPoints = count;
  new_surface.color = ar::Color(0, 1, 0);
  ar::mesh_handle new_id = viz.Add(new_surface);
  surface_id_map.insert(std::pair<int, ar::mesh_handle>(id, new_id));
}

void updateSurface(float* vertices, unsigned int count, unsigned int id)
{
  ar::Polygon new_surface(0, 0); /// HACK: work around ARVisualizer api...
  new_surface.points = vertices;
  new_surface.numPoints = count;
  new_surface.color = ar::Color(0, 1, 0);
  viz.Update(surface_id_map[id], new_surface);
}

void deleteSurface(int surface_id)
{
  viz.Remove(surface_id_map[surface_id]);
  surface_id_map.erase(surface_id);
}

void onObstacleMsg(am2b_iface::ObstacleMessage* message, bool verbose)
{
    if (verbose)
    {
      std::cout << "Received obstacle: " << std::endl;
      std::cout << *message << std::endl;;
    }

    // new obstacle to add to visualization
    if (message->action == am2b_iface::SET_SSV)
    {
      renderObstacle(Obstacle(message));
    }
    else if (message->action == am2b_iface::MODIFY_SSV)
    {
      updateObstacle(Obstacle(message));
    }
    else if (message->action == am2b_iface::REMOVE_SSV_ONLY_PART)
    {
      deleteObstacle(message->model_id);
    }
    else if (message->action == am2b_iface::REMOVE_SSV_WHOLE_SEGMENT)
    {
      deleteObstacle(message->model_id);
    }
    else
    {
        std::cout << "ERROR: Unknown obstacle action: " << message->action << std::endl;
    }
}

void onSurfaceMsg(am2b_iface::SurfaceMessage* message, bool verbose=false)
{
    if (verbose)
    {
      std::cout << "Received Surface:" << std::endl;
      std::cout << "\tid: " << message->id << std::endl;
      std::cout << "\taction: 0x"
                << std::hex << message->action << std::dec << std::endl;
      std::cout << "\tnormal: [" << message->normal[0]
                                 << ", " << message->normal[1]
                                 << ", " << message->normal[2]
                                 << "]" << std::endl;
      std::cout << "\tVertices:" << std::endl;
      for (int i = 0; i < 8; i++)
      {
        std::cout << "\t\t[";
        std::cout << message->vertices[i*3]
                  << ", " << message->vertices[i*3 + 1]
                  << ", " << message->vertices[i*3 + 2];
        std::cout << "]" << std::endl;
      }
    }

    if (message->action == am2b_iface::SET_SURFACE)
    {
      std::cout << "Adding new surface: " << message->id << std::endl;
      renderSurface(message->vertices, 8, message->id);
    }
    else if (message->action == am2b_iface::MODIFY_SURFACE)
    {
      std::cout << "Updating surface: " << message->id << std::endl;
      updateSurface(message->vertices, 8, message->id);
    }
    else if (message->action == am2b_iface::REMOVE_SURFACE)
    {
      std::cout << "Deleting surface: " << message->id << std::endl;
      deleteSurface(message->id);
    }
    else
    {
      std::cout << "Unknown surface operation: 0x" << std::hex << message->action << std::dec << std::endl;
    }
}

void onRGBMsg(am2b_iface::RGBMessage* message, float largeViewer=1.0, bool verbose=false)
{
    if (verbose)
    {
      std::cout << "Received RGB Image:" << std::endl;
      std::cout << "\tWidth:  " << message->width << std::endl;
      std::cout << "\tHeight: " << message->height << std::endl;
    }
    if (largeViewer != 1)
    {
      viz.NotifyNewVideoFrame(message->width, message->height, message->pixels, largeViewer);
    }
    else
    {
      viz.NotifyNewVideoFrame(message->width, message->height, message->pixels);
    }
}

void onPointCloudMsg(am2b_iface::PointCloudMessage* message, bool verbose=false)
{
    if (verbose)
    {
      std::cout << "Received PointCloud" << std::endl;
      std::cout << "\t# of points:    " << message->count << std::endl;
      std::cout << "\tsize of points: " << message->format << " bytes" << std::endl;
    }
    pointcloud.pointData = reinterpret_cast<const void*>(message->data);
    pointcloud.numPoints = message->count;
    viz.Update(cloud_handle, pointcloud);
}

void onNewFootstep(Footstep step, bool verbose=false)
{
  if (verbose)
  {
    std::cout << "[footsteps] New footstep at: "
              << step._position[0] << ", "
              << step._position[1] << ", "
              << step._position[2] << std::endl;
  }

  renderFootstep(step);
}

void printvec(float* vec, unsigned int len, std::ostream& out)
{
  out << "[";
  for (unsigned int i = 0; i < len; i++)
  {
    out << vec[i];

    if (i < len-1)
      out << ", ";
  }
  out << "]";
}

void printmat(float* mat, unsigned int width, unsigned int height, std::string line_prefix, std::ostream& out)
{

  for (unsigned int i = 0; i < width; i++)
  {
    out << line_prefix << "[";
    for (unsigned int j = 0; j < height; j++)
    {
      out << mat[width*i+j];
      if (j < height-1)
        out << ", ";
    }
    out << "]";
    out << std::endl;
  }
}

// utility functions for processing pose data
/**
 * Puts a rotation matrix (around the z-axis) for the given angle in the given
 * matrix `matrix`.
 * It is assumed that the given matrix points to a matrix of dimensions 3x3.
 */

void rotationmatrix(float angle, double matrix[3][3]) {
  double s = sin(angle);
  double c = cos(angle);

  matrix[0][0] = c; matrix[0][1] = -s; matrix[0][2] = 0;
  matrix[1][0] = s; matrix[1][1] = c; matrix[1][2] = 0;
  matrix[2][0] = 0; matrix[2][1] = 0; matrix[2][2] = 1;
}

/**
 * Transposes the given matrix `matrix` and puts the transpose result into the
 * given `transpose` matrix.
 *
 * The matrices are assumed to be 3x3.
 */
void transpose(double matrix[][3], double transpose[][3]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      transpose[j][i] = matrix[i][j];
    }
  }
}
// In pseudo-code (if matrix operations were supported):
// translation = transpose(rotation_matrix) * (t_wr_cl + t_stance_odo)
void getTranslation(double rotation_matrix[3][3], float t_wr_cl[3], float t_stance_odo[3], double translation[3])
{
  double transposed_matrix[3][3];
  transpose(rotation_matrix, transposed_matrix);
  for (int i = 0; i < 3; ++i) {
    translation[i] = 0;
    for (int j = 0; j < 3; ++j) {
      translation[i] += transposed_matrix[i][j] * (t_wr_cl[j] + t_stance_odo[j]);
    }
  }
}

// In pseudo-code (if matrix operations were supported):
// A_odo_cam = transpose(R_wr_cl * rotation_matrix)
void getOrientation(float R_wr_cl[3][3], double rotation_matrix[3][3], double orientation[3][3])
{
  double A_odo_cam_no_trans[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      orientation[i][j] = 0;
      for (int k = 0; k < 3; ++k) {
        orientation[i][j] += R_wr_cl[i][k] * rotation_matrix[k][j];
      }
    }
  }
  // transpose(A_odo_cam_no_trans, orientation);
}

void onNewPose(HR_Pose_Red* new_pose, bool verbose=false)
{
    if (verbose)
    {
      std::cout << "New Pose:" << std::endl;
      std::cout << "\tVersion: " << new_pose->version << std::endl;
      std::cout << "\tTick Counter: " << new_pose->tick_counter << std::endl;
      std::cout << "\tStance: " << (unsigned int)(new_pose->stance) << std::endl;
      std::cout << "\tStamp:  " << new_pose->stamp << std::endl;
      std::cout << "\tt_wr_cl:" << std::endl;
      std::cout << "\t\t"; printvec(new_pose->t_wr_cl, 3, std::cout); std::cout << std::endl;
      std::cout << "\tR_wr_cl:" << std::endl;
      printmat(new_pose->R_wr_cl, 3, 3, "\t\t", std::cout);
      std::cout << "\tt_stance_odo: " << std::endl;;
      std::cout << "\t\t"; printvec(new_pose->t_stance_odo, 3, std::cout); std::cout << std::endl;
      std::cout << "\tphi_z_odo: " << new_pose->phi_z_odo << std::endl;
      std::cout << "------------------------------------------" << std::endl;
    }

    double rotation_matrix[3][3];
    rotationmatrix(new_pose->phi_z_odo, rotation_matrix);
    double cam_position[3];
    double cam_orienation[3][3];
    float R_wr_cl_mat[3][3] ={
                              {new_pose->R_wr_cl[0], new_pose->R_wr_cl[1], new_pose->R_wr_cl[2]},
                              {new_pose->R_wr_cl[3], new_pose->R_wr_cl[4], new_pose->R_wr_cl[5]},
                              {new_pose->R_wr_cl[6], new_pose->R_wr_cl[7], new_pose->R_wr_cl[8]}
                            };
    getTranslation(rotation_matrix, new_pose->t_wr_cl, new_pose->t_stance_odo, cam_position);
    getOrientation(R_wr_cl_mat, rotation_matrix, cam_orienation);
    viz.SetCameraPose(cam_position, cam_orienation);
}

void onSigInt(int s)
{
  shuttingDown = true;
  std::cout << std::endl << "=========================" << std::endl;
  std::cout << "Caught signal " << s << std::endl;;
  std::cout << "Shutting Down" << std::endl;
  std::cout << "=========================" << std::endl;

}

int main(int argc, char* argv[])
{
  ParsedParams params;

  if (!parse_args(argc, argv, &params))
  {
    return 0;
  }

  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = onSigInt;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  double cam_root_position[3] = {0, 0, 0};
  double cam_root_forward[3]  = {1, 0, 0};
  double cam_root_up[3]       = {0, 0, 1};

  double camera_matrix[3][3] = {
    5.2921508098293293e+02, 0.0, 3.2894272028759258e+02,
    0.0, 5.2556393630057437e+02, params.largeViewer * 2.6748068171871557e+02,
    0.0, 0.0, 1.0
  };

  viz.Start("Lola Listener");
  viz.SetCameraPose(cam_root_position, cam_root_forward, cam_root_up);
  viz.SetCameraIntrinsics(camera_matrix);
  cloud_handle = viz.Add(pointcloud);
  int footstep_socket = 0;

  PoseListener pl(params.posePort, params.verbose);
  if (params.posePort > 0)
  {
    pl.onError([](std::string err)->void{std::cout << "ERROR [pose]: " << err << std::endl;});
    pl.onNewPose(std::bind(&onNewPose, _1, params.verbose));
    pl.listen();
  }

  FootstepListener fl(params.footstepPort, params.footstepHost, params.verbose);
  if (params.footstepPort > 0)
  {
    fl.onError([](std::string err)->void{std::cout << "ERROR [footsteps]: " << err << std::endl;});
    fl.onNewStep(std::bind(&onNewFootstep, _1, params.verbose));
    fl.listen();
  }

  VisionListener vl(params.obstaclePort, params.verbose);
  if (params.obstaclePort > 0)
  {
    vl.onConnect([](std::string host)->void{std::cout << "[vision] Connected to: " << host << std::endl;});
    vl.onError([](std::string err)->void{std::cout << "ERROR [vision]: " << err << std::endl;});
    vl.onDisconnect([](std::string host)->void{std::cout << "[vision] Disconnected from: " << host << std::endl;});
    vl.onObstacleMessage(std::bind(&onObstacleMsg, _1, params.verbose));
    vl.onSurfaceMessage(std::bind(&onSurfaceMsg, _1, params.verbose));
    vl.onRGBMessage(std::bind(&onRGBMsg, _1, params.largeViewer, params.verbose));
    vl.onPointCloudMessage(std::bind(&onPointCloudMsg, _1, params.verbose));
    vl.listen();
  }

  while(!shuttingDown)
    std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "Closing open connections..." << std::endl;
  if (footstep_socket > 0)
    close(footstep_socket);
  if (vl.listening())
    vl.stop();
  if (pl.listening())
    pl.stop();

  std::cout << "Stopping visualizer..." << std::endl;
  viz.Stop();
  return 0;
}
