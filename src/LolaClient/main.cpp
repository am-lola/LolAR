#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <getopt.h>
#include <algorithm>
#include <string>
#include <vector>
#include <iostream>

#include <am2b-arvis/ARVisualizer.hpp>

#include "Obstacle.hpp"

/**
 * A UDP client to collect and visualize data from LOLA
 *
 * Listens on the given port for any incoming data from the vision
 * system, and adds it to a local visulization.
 */

// maximum # of bytes expected in any incoming packet
#define BUFLEN 2048
ar::ARVisualizer viz;

struct ParsedParams
{
  unsigned int port = 0; // port number to listen on
  bool verbose = false;
};

// format used by LolaAggregator to send data
struct Obstacle_net {
  int type;
  int radius;
  int rest[9];
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
        {"port",    required_argument, 0, 'p'},
        {"verbose", no_argument,       0, 'v'},
        {"help",    no_argument,       0, 'h'},
        {0}
      };

      int option_index = 0;
      c = getopt_long (argc, argv, "p:vh",
                       long_options, &option_index);

      /* Detect the end of the options. */
      if (c == -1)
        break;

      switch (c)
      {
        case 'p':
          params->port = std::stoi(optarg);
          break;

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

  if (params->port == 0)
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

Obstacle getRealObstacle(Obstacle_net ob)
{
  Obstacle real;
  real._type = (ObstacleType)ob.type;
  real._radius = (double)ob.radius / 1000.0; // LolaAggregator multiplies all non-int values by 1000 before sending

  for (size_t i = 0; i < 3; i++)
  {
    real._coords.push_back({});
    for (size_t j = 0; j < 3; j++)
    {
      real._coords[i].push_back((double)ob.rest[i*3+j] / 1000.0);
    }
  }

  return real;
}

void failWithError(char *s)
{
  perror(s);
  exit(1);
}

void renderObstacle(Obstacle obstacle)
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
      viz.Add(newSphere);
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
      viz.Add(newCapsule);
      break;
    }
    default:
    {
      std::cout << "Unexpected obstacle type: " << obstacle._type << std::endl;
    }
  }
}

void listen(unsigned int port, bool verbose)
{
  struct sockaddr_in si_me, si_other;
  socklen_t s, slen=sizeof(si_other);

  // create & bind socket
  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
    failWithError("creating socket failed!");

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (sockaddr*)&si_me, sizeof(si_me))==-1)
    failWithError("bind failed!");

  std::cout << "Socket opened, listening for data on port " << port << "..." << std::endl;

  // Each packet should contain all obstacles for one frame
  // After receiving a packet, break it into individual Obstacle_net's, and extract relevant info
  Obstacle_net tmp_obs;
  char buf[BUFLEN];
  while (1)
  {
    std::fill(buf, buf+BUFLEN, 0);
    ssize_t nRecvd = 0;
    if ((nRecvd = recvfrom(s, &buf, BUFLEN, 0, (sockaddr*)&si_other, &slen)) == -1 )
      failWithError("recvfrom() failed!");

    printf("Received %d bytes from %s:%d\n",
      nRecvd, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));

    // there should be on obstacle per sizeof(tmp_obs) bytes in input
    size_t nObs = nRecvd / sizeof(tmp_obs);
    if (verbose)
      std::cout << "Extracting " << nObs << " obstacles from packet..." << std::endl;

    // extract obstacles from packet
    for (size_t i = 0; i < nObs; i++)
    {
      int idx = i * sizeof(tmp_obs);
      tmp_obs = *reinterpret_cast<Obstacle_net*>(buf+idx);

      // obstacle data is all scaled & converted from double to int before transmission
      // convert back before doing anything with the data
      Obstacle real_obstacle = getRealObstacle(tmp_obs);
      if (verbose)
      {
        std::cout << "\tObstacle " << i << "/" << nObs-1 << ":" << std::endl;
        std::cout << "\t" << (std::string)(real_obstacle) << std::endl;
      }

      // display obstacle in visualizer
      renderObstacle(real_obstacle);
    }
  }

  close(s);
}

int main(int argc, char* argv[])
{
  ParsedParams params;

  if (!parse_args(argc, argv, &params))
  {
    printHelp(argv[0]);
    return 0;
  }

  viz.Start();

  listen(params.port, params.verbose);

  viz.Stop();
  return 0;
}
