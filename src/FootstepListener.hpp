#ifndef _FOOTSTEP_LISTENER_H
#define _FOOTSTEP_LISTENER_H

///////////////////////////////////////////
// Provides a convenient wrapper around
// communication to receive footstep data
// from Lola
// ////////////////////////////////////////

class FootstepListener
{
public:
  typedef std::function<void(Footstep)>    OnNewStep;
  typedef std::function<void(std::string)> OnError;

  FootstepListener(unsigned int port, std::string host, bool verbose=false) :
    _port(port), _hostname(host), _verbose(verbose)
  {
  }

  void listen()
  {
    if (_listening)
      return;

    _socket = create_client_socket(_port, _hostname);
    _listener = std::thread(&FootstepListener::listen_impl, this);
  }

  bool listening()
  {
    return _listening;
  }

  void stop()
  {
    if (_listening)
    {
      _listening = false;
      if (_listener.joinable())
        _listener.join();
    }
  }

  void onError(OnError cb)     { _onError   = cb; }
  void onNewStep(OnNewStep cb) { _onNewStep = cb; }
private:
  int         _port;
  int         _socket;
  std::string _hostname;
  bool        _verbose = false;
  bool        _listening = false;
  std::thread _listener;
  size_t      _buflen = 2048;

  OnNewStep _onNewStep;
  OnError   _onError;

  // wrapper to ensure safe use of callbacks
  template <class Functor, class... Args>
  void cb(Functor&& f, Args&&... args)
  {
    if (f)
      f(std::forward<Args>(args)...);
  }

  void listen_impl()
  {
    unsigned char buf[_buflen];

    if (_verbose)
      std::cout << "[footsteps] Attempting to subscribe to footstep data from " << _hostname << std::endl;

    am2b_iface::MsgId footstep_sub_id  = __DOM_WPATT; //am2b_iface::STEPSEQ_AR_VIZUALIZATION;
    am2b_iface::MsgHeader footstep_sub = { am2b_iface::ps::SIG_PS_SUBSCRIBE, sizeof(am2b_iface::MsgId)};

    size_t sent = write(_socket, (unsigned char*)&footstep_sub, sizeof(footstep_sub));
    if (sent <= 0)
      std::cout << "[footsteps] Error sending subscribe request!" << std::endl;
    sent = write(_socket, (unsigned char*)&footstep_sub_id, sizeof(footstep_sub_id));
    if (sent <= 0)
      std::cout << "[footsteps] Error sending footstep sub ID!" << std::endl;

    if (_verbose)
      std::cout << "[footsteps] Subscribed!" << std::endl;

    while (_listening)
    {
      std::fill(buf, buf+_buflen, 0);
      ssize_t total_received = 0;
      ssize_t header_size    = sizeof(am2b_iface::MsgHeader);
      ssize_t step_size      = sizeof(am2b_iface::struct_data_stepseq_ssv_log);

      if (_verbose)
      {
        std::cout << "[footsteps] Waiting for am2b_iface::MsgHeader (" << header_size << " bytes)..." << std::endl;
      }

      while (total_received < header_size)
      {
        int recvd = 0;
        recvd = read(_socket, &buf[total_received], _buflen-total_received);

        if (recvd == 0) // connection died
          break;
        if (recvd == -1) // failed to read from socket
          cb(_onError, "read() failed!");

        total_received += recvd;

        if (_verbose)
        {
          std::printf("[footsteps] footstep header) Received %zu total bytes from %s\n",
                     total_received, _hostname.c_str());
        }
      }
      if (total_received < header_size)
        break;

      am2b_iface::MsgHeader* header = (am2b_iface::MsgHeader*)buf;
      while (total_received < header_size + header->len)
      {
        int recvd = 0;
        recvd = read(_socket, &buf[total_received], _buflen-total_received);

        if (recvd == 0) // connection died
          break;
        if (recvd == -1) // failed to read from socket
          cb(_onError, "read() failed!");

        total_received += recvd;

        if (_verbose)
        {
          std::printf("[footsteps] (footstep data) Received %zu total bytes from %s\n",
                      total_received, _hostname.c_str());
        }
      }

      if (header->id != am2b_iface::STEPSEQ_AR_VIZUALIZATION)
      {
      	if (header->id == am2b_iface::COM_EOK)
      	{
      	  std::cout << "[footsteps] Received COM_EOK! msg len: " << header->len << std::endl;
      	}
      	else
      	{
          std::cout << "[footsteps] Skipping message (type: 0x"
                    << std::hex << header->id << std::dec
                    << ", expecting: 0x" << std::hex << am2b_iface::STEPSEQ_AR_VIZUALIZATION << std::dec
                    << ")" << std::endl;;
      	}
        continue;
      }

      am2b_iface::struct_data_stepseq_ssv_log* message = (am2b_iface::struct_data_stepseq_ssv_log*)(buf + header_size);
      Footstep step;
      step._foot = (Foot)(message->stance);
      step._position.push_back(message->start_x);
      step._position.push_back(message->start_y);
      step._position.push_back(message->start_z);

      cb(_onNewStep, step);
    }

    if (_verbose)
    {
      std::cout << std::endl << "-------------------------------------------------" << std::endl;
      std::cout << "[footsteps] Connection to server " << _hostname << " terminated!" << std::endl;
      std::cout << "-------------------------------------------------" << std::endl << std::endl;
    }
    close(_socket);
  }
};

#endif // _FOOTSTEP_LISTENER_H
