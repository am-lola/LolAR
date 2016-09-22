#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <signal.h>
#include <map>

#include <algorithm>
#include <thread>
#include <string>
#include <vector>
#include <iostream>

#include <iface_msg.hpp>
#include <iface_sig_msg.hpp>
#include <iface_ps.hpp>
#include <iface_stepseq.hpp>
#include <iface_vis.h>
#include <iface_vision_msg.hpp>

#include <am2b-arvis/ARVisualizer.hpp>

#include <tclap/CmdLine.h>

#include "Obstacle.hpp"
#include "Footstep.hpp"


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
  bool verbose = false;
};

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
                                               "Hostname to connect to for footstep data, in the form: ' ip:port '",
                                               false, "", "hostname:port  or  ip:port");
    TCLAP::ValueArg<unsigned int> posePort("p", "poseport",
                                               "Port to listen on for pose data",
                                               false, 0, "unsigned int");
    TCLAP::SwitchArg verbose("v", "verbose",
                             "Verbose output", cmd, false);
    cmd.add(obstaclePort);
    cmd.add(footstepHost);
    cmd.add(posePort);

    cmd.parse(argc, argv);

    params->obstaclePort = obstaclePort.getValue();
    params->posePort = posePort.getValue();
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

Obstacle getRealObstacle(am2b_iface::ObstacleMessage* ob)
{
  Obstacle real;
  real._type = (ObstacleType)ob->type;
  real._id = ob->model_id;
  real._radius = (double)ob->radius;

  for (size_t i = 0; i < 3; i++)
  {
    real._coords.push_back({});
    for (size_t j = 0; j < 3; j++)
    {
      real._coords[i].push_back((double)ob->coeffs[i*3+j]);
    }
  }

  return real;
}

void failWithError(const std::string s)
{
  perror(s.c_str());
  exit(1);
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

void process(am2b_iface::VisionMessage* message)
{

}

void readVisionMessagesFrom(int socket_remote, const sockaddr_in& si_other, bool verbose)
{
  std::vector<unsigned char> buf;
  buf.resize(BUFLEN);

  while (!shuttingDown)
  {
    ssize_t ifaceHeaderSize = sizeof(am2b_iface::MsgHeader);
    ssize_t visionHeaderSize = sizeof(am2b_iface::VisionMessageHeader);
    ssize_t total_received = 0;
    ssize_t total_expected = sizeof(ifaceHeaderSize);

    if (verbose)
    {
      std::cout << "Waiting for am2b_iface::msgHeader (" << ifaceHeaderSize << " bytes)..." << std::endl;
    }

    while (total_received < total_expected)
    {
      int recvd = 0;
      recvd = read(socket_remote, &buf[total_received], total_expected - total_received);

      if (recvd == 0) // connection died
        break;
      if (recvd == -1) // failed to read from socket
        failWithError("read() failed!");

      total_received += recvd;

      if (verbose)
      {
        std::printf("(iface header) Received %zu / %zu total bytes from %s:%d\n", total_received, total_expected, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
      }
    }

    if (total_received < ifaceHeaderSize)
      break;

    am2b_iface::MsgHeader* iface_header = (am2b_iface::MsgHeader*)buf.data();
    if (verbose)
    {
      std::cout << "Received am2b_iface::MsgHeader: id = 0x" << std::hex << iface_header->id << std::dec << ", len = " << iface_header->len << std::endl;
    }

    total_expected += iface_header->len;
    if (buf.size() < total_expected)
    {
      if (verbose)
      {
        std::cout << "Resizing receive buffer to " << total_expected << " bytes" << std::endl;
      }
      buf.resize(total_expected);
      iface_header = (am2b_iface::MsgHeader*)buf.data(); // update pointer after allocation
    }
    while (total_received < total_expected)
    {
      int recvd = 0;
      recvd = read(socket_remote, &buf[total_received], total_expected-total_received);

      if (recvd == 0) // connection died
        break;
      if (recvd == -1) // failed to read from socket
        failWithError("read() failed!");

      total_received += recvd;
      if (verbose)
      {
        std::printf("(message) Received %zu / %zu total bytes from %s:%d\n", total_received, total_expected, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
      }
    }
    if (total_received < total_expected)
      break;

    if (total_expected != total_received)
    {
      std::cout << "TOTAL RECVD: " << total_received << " : EXPECTED: " << sizeof(am2b_iface::MsgHeader) + iface_header->len << std::endl;

      break;
    }
    else
    {
      total_received = 0;
    }


    if (iface_header->id != am2b_iface::VISION_MESSAGE)
    {
      std::cout << "Received non-vision message type '0x" << std::hex << iface_header->id << std::dec << "' on obstacle port. Discarding it..." << std::endl;
      continue;
    }

    am2b_iface::VisionMessageHeader* visionHeader = (am2b_iface::VisionMessageHeader*)(buf.data() + sizeof(am2b_iface::MsgHeader));
    if (verbose)
    {
      std::cout << "Received VisionMessageHeader: " << *visionHeader << std::endl;
      std::cout << "Waiting to receive a total of " << visionHeader->len + sizeof(am2b_iface::VisionMessageHeader) << " bytes..." << std::endl;
    }

    switch (visionHeader->type)
    {
      case am2b_iface::Message_Type::Obstacle:
      {
        am2b_iface::ObstacleMessage* message = (am2b_iface::ObstacleMessage*)(buf.data() + sizeof(am2b_iface::VisionMessageHeader) + sizeof(am2b_iface::MsgHeader));
        if (verbose)
        {
          std::cout << "Received obstacle: " << std::endl;
          std::cout << *message << std::endl;;
        }

        // new obstacle to add to visualization
        if (message->action == am2b_iface::SET_SSV)
        {
          renderObstacle(getRealObstacle(message));
        }
        else if (message->action == am2b_iface::MODIFY_SSV)
        {
          updateObstacle(getRealObstacle(message));
        }
        else if (message->action == am2b_iface::REMOVE_SSV_ONLY_PART)
        {
          deleteObstacle(message->model_id);
        }
        else if (message->action == am2b_iface::REMOVE_SSV_WHOLE_SEGMENT)
        {
          deleteObstacle(message->model_id);
        }

        break;
      }
      case am2b_iface::Message_Type::Surface:
      {
        am2b_iface::SurfaceMessage* message = (am2b_iface::SurfaceMessage*)(buf.data() + sizeof(am2b_iface::VisionMessageHeader) + sizeof(am2b_iface::MsgHeader));
        if (verbose)
        {
          std::cout << "Received Surface:" << std::endl;
          std::cout << "\tid: " << message->id << std::endl;
          std::cout << "\taction: 0x" << std::hex << message->action << std::dec << std::endl;
          std::cout << "\tnormal: [" << message->normal[0] << ", " << message->normal[1] << ", " << message->normal[2] << "]" << std::endl;
          std::cout << "\tVertices:" << std::endl;
          for (int i = 0; i < 8; i++)
          {
            std::cout << "\t\t[";
            std::cout << message->vertices[i*3] << ", " << message->vertices[i*3 + 1] << ", " << message->vertices[i*3 + 2];
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
        break;
      }
      case am2b_iface::Message_Type::RGB_Image:
      {
        am2b_iface::RGBMessage* message = (am2b_iface::RGBMessage*)(buf.data() + sizeof(am2b_iface::VisionMessageHeader) + sizeof(am2b_iface::MsgHeader));
        message->pixels = (unsigned char*)((char*)message + sizeof(am2b_iface::RGBMessage)); // fix pointer
        if (verbose)
        {
          std::cout << "Received RGB Image:" << std::endl;
          std::cout << "\tWidth:  " << message->width << std::endl;
          std::cout << "\tHeight: " << message->height << std::endl;
        }
        viz.NotifyNewVideoFrame(message->width, message->height, message->pixels);
        break;
      }
      case am2b_iface::Message_Type::PointCloud:
      {
        am2b_iface::PointCloudMessage* message = (am2b_iface::PointCloudMessage*)(buf.data() + sizeof(am2b_iface::VisionMessageHeader) + sizeof(am2b_iface::MsgHeader));
        message->data = (unsigned char*)((char*)message + sizeof(am2b_iface::PointCloudMessage));
        if (verbose)
        {
          std::cout << "Received PointCloud" << std::endl;
          std::cout << "\t# of points:    " << message->count << std::endl;
          std::cout << "\tsize of points: " << message->format << " bytes" << std::endl;
        }
        pointcloud.pointData = reinterpret_cast<const void*>(message->data);
        pointcloud.numPoints = message->count;
        viz.Update(cloud_handle, pointcloud);
        break;
      }
      default:
      {
        std::cout << "UNKNOWN message type: " << visionHeader->type << "!!" << std::endl;
      }
    }

  }

  std::cout << std::endl << "-------------------------------------------------" << std::endl;
  std::cout << "Connection to client " << inet_ntoa(si_other.sin_addr) << ":" << ntohs(si_other.sin_port) << " terminated!" << std::endl;
  std::cout << "-------------------------------------------------" << std::endl << std::endl;
  close(socket_remote);
}

void readFootstepsFrom(int socket_remote, const std::string host_addr, bool verbose)
{
  unsigned char buf[BUFLEN];

  std::cout << "Attempting to subscribe to footstep data from " << host_addr << std::endl;
  am2b_iface::MsgId footstep_sub_id = __DOM_WPATT; //am2b_iface::STEPSEQ_AR_VIZUALIZATION;
  am2b_iface::MsgHeader footstep_sub = { am2b_iface::ps::SIG_PS_SUBSCRIBE, sizeof(am2b_iface::MsgId)};

  size_t sent = write(socket_remote, (unsigned char*)&footstep_sub, sizeof(footstep_sub));
  if (sent <= 0)
    std::cout << "Error sending subscribe request!" << std::endl;
  sent = write(socket_remote, (unsigned char*)&footstep_sub_id, sizeof(footstep_sub_id));
  if (sent <= 0)
    std::cout << "Error sending footstep sub ID!" << std::endl;

  std::cout << "Subscribed?" << std::endl;

  while (!shuttingDown)
  {
    std::fill(buf, buf+BUFLEN, 0);
    ssize_t total_received = 0;
    ssize_t header_size = sizeof(am2b_iface::MsgHeader);
    ssize_t step_size = sizeof(am2b_iface::struct_data_stepseq_ssv_log);
    if (verbose)
    {
      std::cout << "Waiting for am2b_iface::MsgHeader (" << header_size << " bytes)..." << std::endl;
    }

    while (total_received < header_size)
    {
      int recvd = 0;
      recvd = read(socket_remote, &buf[total_received], BUFLEN-total_received);

      if (recvd == 0) // connection died
        break;
      if (recvd == -1) // failed to read from socket
        failWithError("read() failed!");

      total_received += recvd;

      if (verbose)
      {
        std::printf("(footstep header) Received %zu total bytes from %s\n", total_received, host_addr.c_str());
      }
    }
    if (total_received < header_size)
      break;

    am2b_iface::MsgHeader* header = (am2b_iface::MsgHeader*)buf;
    while (total_received < header_size + header->len)
    {
      int recvd = 0;
      recvd = read(socket_remote, &buf[total_received], BUFLEN-total_received);

      if (recvd == 0) // connection died
        break;
      if (recvd == -1) // failed to read from socket
        failWithError("read() failed!");

      total_received += recvd;

      if (verbose)
      {
        std::printf("(footstep data) Received %zu total bytes from %s\n", total_received, host_addr.c_str());
      }
    }

    if (header->id != am2b_iface::STEPSEQ_AR_VIZUALIZATION)
    {
	if (header->id == am2b_iface::COM_EOK)
	{
	  std::cout << "Received COM_EOK! msg len: " << header->len << std::endl;
	}
	else
	{
	      std::cout << "Skipping message (type: 0x" << std::hex << header->id << std::dec << ", expecting: 0x" << std::hex << am2b_iface::STEPSEQ_AR_VIZUALIZATION << std::dec << ")" << std::endl;;
	}
      continue;
    }

    am2b_iface::struct_data_stepseq_ssv_log* message = (am2b_iface::struct_data_stepseq_ssv_log*)(buf + header_size);
    Footstep step;
    step._foot = (Foot)(message->stance);
    step._position.push_back(message->start_x);
    step._position.push_back(message->start_y);
    step._position.push_back(message->start_z);

    std::cout << "Adding footstep at: " << message->start_x << ", " << message->start_y << ", " << message->start_z << std::endl;
    renderFootstep(step);
  }

  std::cout << std::endl << "-------------------------------------------------" << std::endl;
  std::cout << "Connection to server " << host_addr << " terminated!" << std::endl;
  std::cout << "-------------------------------------------------" << std::endl << std::endl;
  close(socket_remote);
}

int create_server_socket(unsigned int port)
{
  struct sockaddr_in si_me;
  int s;

  // create & bind socket
  if ((s=socket(AF_INET, SOCK_STREAM, 0))==-1)
    failWithError("creating socket failed!");

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (sockaddr*)&si_me, sizeof(si_me))==-1)
    failWithError("bind failed!");

  if (listen(s, 5) != 0)
    failWithError("listen failed!");

  return s;
}

int create_client_socket(unsigned int port, std::string host)
{
  struct addrinfo hints, *res;
  struct sockaddr_in server_addr;
  int s;

  // get server info for connection
  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  int n = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res);
  if (n != 0)
    failWithError("Could not get host info for: " + host);

  std::cout << "Attempting to connect to: " << host << ":" << port << std::endl;

  // connect to server
  bool connection_success = false;
  for (auto rp = res; rp != NULL; rp = rp->ai_next) // check all addresses found by getaddrinfo
  {
    s = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (s < 0)
      continue; // could not open socket

    // attempt to connect
    if (connect(s, rp->ai_addr, rp->ai_addrlen) == 0)
    {
      std::cout << "Successfully connected to " << host << ":" << port << std::endl;
      connection_success = true;
      break;
    }
    else
    {
      perror("Could not connect");
    }

    close(s); // if connection failed, close socket and move on to the next address
  }

  if (!connection_success)
  {
    std::cout << "Connection to " << host << ":" << port << " failed! Exiting..." << std::endl;
    exit(1);
  }

  freeaddrinfo(res);
  return s;
}

socklen_t create_udp_socket(unsigned int port)
{
  struct sockaddr_in si_me;
  socklen_t s;
  int broadcast = 1;
  int reuseport = 1;

  // create & bind socket
  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    failWithError("Creating UDP socket failed!");

  if (setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) != 0)
    failWithError("Setting broadcast flag on UDP socket failed!");

  if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuseport, sizeof(reuseport)) != 0)
    failWithError("Setting Reuse Addr flag on UDP socket failed!");

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (sockaddr*)&si_me, sizeof(si_me)) == -1)
    failWithError("Binding UDP socket failed!");

  return s;
}

void listen(int sock_obstacles, bool verbose)
{
  struct sockaddr_in si_other;
  int s_other;
  socklen_t slen=sizeof(si_other);

  while(!shuttingDown)
  {
    std::cout << "Polling for new connections..." << std::endl;

    fd_set readfds; FD_ZERO(&readfds);
    int maxfd, fd;

    maxfd = -1;
    FD_SET(sock_obstacles, &readfds);
    maxfd = sock_obstacles;

    if (select(maxfd + 1, &readfds, NULL, NULL, NULL) < 0)
      continue;
    fd = -1;
    if (FD_ISSET(sock_obstacles, &readfds))
    {
      fd = sock_obstacles;
    }

    if (fd == -1)
      failWithError("Invalid socket returned from select!");

    s_other = accept(fd, (struct sockaddr* ) &si_other, &slen);
    if (s_other < 0)
      failWithError("accept failed!");

    std::cout << std::endl << "-------------------------------------------------" << std::endl;
    std::cout << "NEW Connection to " << inet_ntoa(si_other.sin_addr) << ":" << htons(si_other.sin_port) << std::endl;
    std::cout << "-------------------------------------------------" << std::endl << std::endl;

    // receive data from new connection
    if (fd == sock_obstacles)
    {
      std::thread servicer(readVisionMessagesFrom, s_other, si_other, verbose);
      servicer.detach();
    }
  }
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



void udp_pose_listen(socklen_t s, bool verbose)
{
  char buf[BUFLEN];
  sockaddr_in si_other;
  unsigned int slen = sizeof(sockaddr);

  while(!shuttingDown)
  {
    // clear buf
    memset(buf, 0, BUFLEN-1);

    // wait for message
    int nrecvd = recvfrom(s, buf, BUFLEN-1, 0, (sockaddr*)&si_other, &slen);

    if (verbose)
      std::cout << "Received " << nrecvd << " bytes from: " << inet_ntoa(si_other.sin_addr) << std::endl;

    HR_Pose_Red* new_pose = (HR_Pose_Red*)buf;

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
    0.0, 5.2556393630057437e+02, 2.6748068171871557e+02,
    0.0, 0.0, 1.0
  };

  viz.Start("Lola Listener");
  viz.SetCameraPose(cam_root_position, cam_root_forward, cam_root_up);
  viz.SetCameraIntrinsics(camera_matrix);
  cloud_handle = viz.Add(pointcloud);
  int footstep_socket = 0;
  int obstacle_socket = 0;
  int pose_socket     = 0;

  if (params.posePort > 0)
  {
    pose_socket = create_udp_socket(params.posePort);
    std::thread servicer(udp_pose_listen, pose_socket, params.verbose);
    servicer.detach();
  }

  if (params.footstepPort > 0)
  {
    footstep_socket = create_client_socket(params.footstepPort, params.footstepHost);
    std::thread servicer(readFootstepsFrom, footstep_socket, params.footstepHost + ":" + std::to_string(params.footstepPort), params.verbose);
    servicer.detach();
  }

  if (params.obstaclePort > 0)
  {
    obstacle_socket = create_server_socket(params.obstaclePort);
    listen(obstacle_socket, params.verbose);
  }

  while(!shuttingDown)
    std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "Closing open sockets..." << std::endl;
  if (footstep_socket > 0)
    close(footstep_socket);
  if (obstacle_socket > 0)
    close(obstacle_socket);
  if (pose_socket > 0)
    close(pose_socket);

  std::cout << "Stopping visualizer..." << std::endl;
  viz.Stop();
  return 0;
}
