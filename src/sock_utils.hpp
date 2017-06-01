#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

/////////////////////////////////////////////////
///
/// A small collection of helper functions
/// for dealing with socket connections
///
/////////////////////////////////////////////////

// prints an error and exits
static void failWithError(const std::string s)
{
  perror(s.c_str());
  exit(1);
}

// creates a new TCP socket on the given port
// which can be used to listen for new connections
static int create_server_socket(unsigned int port)
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

// Creates a new TCP socket connected to the given host:port
static int create_client_socket(unsigned int port, std::string host)
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

// Creates a new UDP socket on the given port
static socklen_t create_udp_socket(unsigned int port)
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

