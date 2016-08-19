#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <signal.h>
#include <map>

#include <getopt.h>
#include <algorithm>
#include <thread>
#include <string>
#include <vector>
#include <iostream>

#include <iface_vision_msg.hpp>
#include <am2b-arvis/ARVisualizer.hpp>

#include "Obstacle.hpp"
#include "Footstep.hpp"

// TODO: Remove this
struct struct_data_stepseq_ssv_log
  // :
  //     public struct_data_ssv,
  //     public struct_data_stepseq
  {

    // offsetof is not able to deal with inheritance
    //
    uint64_t stamp_gen;
    //----------------------------------------------------------------------
    // struct data stepseq base

    float L0;
    float L1;
    float B;
    float phiO;
    float dz_clear;
    float dz_step;
    float dy;
    float H;
    float T; // step time

    //----------------------------------------------------------------------
    // struct data stepseq

    // stance foot (?)
    float start_x;
    float start_y;
    float start_z;
    float start_phi_z;
    float phi_leg_rel;
    int32_t stance;


    //----------------------------------------------------------------------
    // Optimization results
    float dz_clear_ref;
    float dz_clear_opt;
    float dy_ref;
    float dy_opt;
    float H_cog_ref;
    float H_cog_opt;
    float V_old_opt;
    float V_new_opt;

    //----------------------------------------------------------------------
    // iface ssv
    //! id of segment
    int32_t id;
    //! id of ssv object (not used : 0)
    int32_t id_ssv;
    //! together with delete event: should only ssv obj be deleted or whole segment?
    int32_t remove_ssv_obj;
    //! type -> 0:PSS, 1:LSS, 2:TSS
    int32_t type;
    //! radius
    float radius;
    //! Which is the coresseponding surface?
    int32_t surface;
    //! vectors
    float p0[3];
    float p1[3];
    float p2[3];

  };

/**
 * A client to collect and visualize data from LOLA
 *
 * Listens on the given port for any incoming data from the vision
 * system, and adds it to a local visualization.
 */

// maximum # of bytes expected in any incoming packet
#define BUFLEN 2048
ar::ARVisualizer viz;
bool shuttingDown = false;
// maps lepp IDs to visualizer IDs
std::map<int, ar::mesh_handle> obstacle_id_map;

struct ParsedParams
{
  unsigned int obstaclePort = 0; // port number to listen on for obstacle data
  unsigned int footstepPort = 0; // port number to listen on for footstep data
  unsigned int posePort     = 0; // port number to listen on for pose data
  bool verbose = false;
};

bool parse_args(int argc, char* argv[], ParsedParams* params)
{
  int c;

  if (argc < 2)
  {
    std::cout << "ERROR: You must provide at least one argument!" << std::endl;
    return false;
  }

  while (1)
  {
    static struct option long_options[] =
      {
        {"obstacleport", required_argument, 0, 'o'},
        {"footstepport", required_argument, 0, 'f'},
        {"verbose", no_argument,       0, 'v'},
        {"help",    no_argument,       0, 'h'},
        {0}
      };

      int option_index = 0;
      c = getopt_long (argc, argv, "o:f:vh",
                       long_options, &option_index);

      /* Detect the end of the options. */
      if (c == -1)
        break;

      switch (c)
      {
        case 'o':
          params->obstaclePort = std::stoi(optarg);
          break;

        case 'f':
          params->footstepPort = std::stoi(optarg);

        case 'v':
          params->verbose = true;
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

  if (params->obstaclePort == 0)
  {
    std::cout << "You MUST specify a port to listen on!" << std::endl;
    return false;
  }

  return true;
}

void printHelp(std::string name)
{
  std::cout << "Usage:" << std::endl;
  std::cout << "\t" << name << " --port Port Number [--verbose] [--help]" << std::endl;
  std::cout << "\t" << name << " -p Port Number [-v] [-h]" << std::endl;
  std::cout << std::endl;
  std::cout << "\t" << "     port: UDP Port to listen on for data from the robot" << std::endl;
  std::cout << "\t" << "  verbose: Verbose output" << std::endl;
  std::cout << "\t" << "     help: Display this message" << std::endl;
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

void failWithError(const char *s)
{
  perror(s);
  exit(1);
}

void renderFootstep(Footstep footstep)
{
  double quadNormal[3] = {0, 1, 0}; // TODO: Orient toward +Z
  ar::Quad newQuad(
    footstep._position[0], footstep._position[1], footstep._position[2],
    quadNormal,
    0.15, 0.15,
    ar::Color(0.2, footstep._foot == Left ? 1 : 0, 1.0, 0.9)
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

void readObstaclesFrom(int socket_remote, const sockaddr_in& si_other, bool verbose)
{
  unsigned char buf[BUFLEN];
  while (!shuttingDown)
  {
    std::fill(buf, buf+BUFLEN, 0);
    ssize_t headerSize = sizeof(am2b_iface::VisionMessageHeader);
    ssize_t total_received = 0;

    if (verbose)
    {
      std::cout << "Waiting for VisionMessageHeader (" << headerSize << " bytes)..." << std::endl;
    }

    while (total_received < headerSize)
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
        std::printf("(header) Received %zu total bytes from %s:%d\n", total_received, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
      }
    }
    if (total_received < headerSize)
      break;

    am2b_iface::VisionMessageHeader* header = (am2b_iface::VisionMessageHeader*)buf;

    if (verbose)
    {
      std::cout << "Received VisionMessageHeader: " << *header << std::endl;
      std::cout << "Waiting to receive a total of " << header->len + sizeof(am2b_iface::VisionMessageHeader) << " bytes..." << std::endl;
    }

    while (total_received < header->len + sizeof(am2b_iface::VisionMessageHeader))
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
        std::printf("(message) Received %zu total bytes from %s:%d\n", total_received, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
      }
    }
    if (total_received < header->len + sizeof(am2b_iface::VisionMessageHeader))
      break;

    switch (header->type)
    {
      case am2b_iface::Message_Type::Obstacle:
      {
        am2b_iface::ObstacleMessage* message = (am2b_iface::ObstacleMessage*)(buf+sizeof(am2b_iface::VisionMessageHeader));
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
        am2b_iface::SurfaceMessage* message = (am2b_iface::SurfaceMessage*)(buf + sizeof(am2b_iface::VisionMessageHeader));
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
        break;
      }
      default:
      {
        std::cout << "UNKNOWN message type: " << header->type << "!!" << std::endl;
      }
    }

  }

  std::cout << std::endl << "-------------------------------------------------" << std::endl;
  std::cout << "Connection to client " << inet_ntoa(si_other.sin_addr) << ":" << ntohs(si_other.sin_port) << " terminated!" << std::endl;
  std::cout << "-------------------------------------------------" << std::endl << std::endl;
  close(socket_remote);
}

void readFootstepsFrom(int socket_remote, const sockaddr_in& si_other, bool verbose)
{
  unsigned char buf[BUFLEN];
  while (!shuttingDown)
  {
    std::fill(buf, buf+BUFLEN, 0);
    ssize_t total_received = 0;
    ssize_t step_size = sizeof(struct_data_stepseq_ssv_log);
    if (verbose)
    {
      std::cout << "Waiting for stepseq data (" << step_size << " bytes)..." << std::endl;
    }

    while (total_received < step_size)
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
        std::printf("(footstep) Received %zu total bytes from %s:%d\n", total_received, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
      }
    }
    if (total_received < step_size)
      break;

    struct_data_stepseq_ssv_log* message = (struct_data_stepseq_ssv_log*)(buf);
    Footstep step;
    step._foot = (Foot)(message->stance);
    step._position.push_back(message->start_x);
    step._position.push_back(message->start_y);
    step._position.push_back(message->start_z);

    std::cout << "Adding footstep at: " << message->start_x << ", " << message->start_y << ", " << message->start_z << std::endl;
    renderFootstep(step);
  }

  std::cout << std::endl << "-------------------------------------------------" << std::endl;
  std::cout << "Connection to client " << inet_ntoa(si_other.sin_addr) << ":" << ntohs(si_other.sin_port) << " terminated!" << std::endl;
  std::cout << "-------------------------------------------------" << std::endl << std::endl;
  close(socket_remote);
}

int create_socket(unsigned int port)
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

void listen(int sock_footsteps, int sock_obstacles, bool verbose)
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
    FD_SET(sock_footsteps, &readfds);
    FD_SET(sock_obstacles, &readfds);
    if (sock_footsteps > sock_obstacles)
      maxfd = sock_footsteps;
    else
      maxfd = sock_obstacles;

    if (select(maxfd + 1, &readfds, NULL, NULL, NULL) < 0)
      continue;
    fd = -1;
    if (FD_ISSET(sock_footsteps, &readfds))
    {
      fd = sock_footsteps;
    }
    else if (FD_ISSET(sock_obstacles, &readfds))
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
      std::thread servicer(readObstaclesFrom, s_other, si_other, verbose);
      servicer.detach();
    }
    else if (fd == sock_footsteps)
    {
      std::thread servicer(readFootstepsFrom, s_other, si_other, verbose);
      servicer.detach();
    }
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
    printHelp(argv[0]);
    return 0;
  }

  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = onSigInt;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);


  viz.Start();

  int footstep_socket = create_socket(params.footstepPort);
  int obstacle_socket = create_socket(params.obstaclePort);

  listen(footstep_socket, obstacle_socket, params.verbose);

  std::cout << "Closing open sockets..." << std::endl;
  close(footstep_socket);
  close(obstacle_socket);

  std::cout << "Stopping visualizer..." << std::endl;
  viz.Stop();
  return 0;
}
