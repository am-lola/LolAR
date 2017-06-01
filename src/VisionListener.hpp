#ifndef VISION_LISTENER_H
#define VISION_LISTENER_H

///////////////////////////////////////////
///
/// Provides convenient access to vision
/// messages from lepp3 (or other sources)
///
//////////////////////////////////////////
#include <functional>

#include <iface_msg.hpp>
#include <iface_vision_msg.hpp>
#include "sock_utils.hpp"

class VisionListener
{
public:
    typedef std::function<void(am2b_iface::VisionMessage*)>     OnVisionMessage;
    typedef std::function<void(am2b_iface::ObstacleMessage*)>   OnObstacleMessage;
    typedef std::function<void(am2b_iface::SurfaceMessage*)>    OnSurfaceMessage;
    typedef std::function<void(am2b_iface::RGBMessage*)>        OnRGBMessage;
    typedef std::function<void(am2b_iface::PointCloudMessage*)> OnPointCloudMessage;
    typedef std::function<void(std::string)>                    OnNewConnection;
    typedef std::function<void(std::string)>                    OnDisconnect;
    typedef std::function<void(std::string)>                    OnError;

    VisionListener(bool verbose=false)
    {
        _verbose = verbose;
    }

    VisionListener(int port, bool verbose=false) : VisionListener(verbose)
    {
        _port = port;
    }

    // open for incoming connections
    void listen()
    {
        if (_listening)
            return;

        _listener = std::thread(&VisionListener::listen_impl, this);
    }

    // stop listening & close any active connections
    void stop() 
    {
        _listening = false;
        _listener.join();
    }

    // callback connections
    void onConnect          ( OnNewConnection cb )     { _onConnect           = cb; }
    void onDisconnect       ( OnDisconnect cb )        { _onDisconnect        = cb; }
    void onError            ( OnError cb )             { _onError             = cb; }
    void onVisionMessage    ( OnVisionMessage cb )     { _onVisionMessage     = cb; }
    void onObstacleMessage  ( OnObstacleMessage cb )   { _onObstacleMessage   = cb; }
    void onSurfaceMessage   ( OnSurfaceMessage cb )    { _onSurfaceMessage    = cb; }
    void onRGBMessage       ( OnRGBMessage cb )        { _onRGBMessage        = cb; }
    void onPointCloudMessage( OnPointCloudMessage cb ) { _onPointCloudMessage = cb; }
private:
    bool _verbose = false;
    int  _port;
    int  _socket;
    bool _listening = false;
    std::thread _listener;
    size_t _buflen = 2048;

    OnError             _onError;
    OnNewConnection     _onConnect;
    OnDisconnect        _onDisconnect;
    OnVisionMessage     _onVisionMessage;
    OnSurfaceMessage    _onSurfaceMessage;
    OnObstacleMessage   _onObstacleMessage;
    OnRGBMessage        _onRGBMessage;
    OnPointCloudMessage _onPointCloudMessage;

    // wrapper to ensure safe use of callbacks
    template <class Functor, class... Args>
    void cb(Functor&& f, Args&&... args)
    {
        if (f)
            f(std::forward<Args>(args)...);
    }

    std::string hostString(const sockaddr_in& s)
    {
        return (std::string)(inet_ntoa(s.sin_addr)) + ":" + std::to_string(ntohs(s.sin_port));
    }

    void listen_impl()
    {
        struct sockaddr_in si_other;
        int s_other;
        socklen_t slen=sizeof(si_other);

        _socket = create_server_socket(_port);
        _listening = true;

        while( _listening )
        {
            fd_set readfds; FD_ZERO(&readfds);
            int maxfd, fd;

            maxfd = -1;
            FD_SET(_socket, &readfds);
            maxfd = _socket;

            if (select(maxfd + 1, &readfds, NULL, NULL, NULL) < 0)
                continue;
            fd = -1;

            if (FD_ISSET(_socket, &readfds))
            {
                fd = _socket;
            }

            if (fd == -1)
                cb(_onError, "Invalid socket returned from select!");

            s_other = accept(fd, (struct sockaddr*) &si_other, &slen);
            if (s_other < 0)
                cb(_onError, "accept failed!");

            cb(_onConnect, hostString(si_other));

            if (fd == _socket)
            {
                std::thread servicer(&VisionListener::readVisionMessagesFrom, this, s_other, si_other);
                servicer.detach();
            }
        }
    }

    void readVisionMessagesFrom(int socket_remote, const sockaddr_in& si_other)
    {
        std::vector<unsigned char> buf;
        buf.resize(_buflen);

        while (_listening)
        {
            ssize_t ifaceHeaderSize  = sizeof(am2b_iface::MsgHeader);
            ssize_t visionHeaderSize = sizeof(am2b_iface::VisionMessageHeader);
            ssize_t total_received   = 0;
            ssize_t total_expected   = ifaceHeaderSize;

            if (_verbose)
            {
                std::cout << "Waiting for am2b_iface::MsgHeader (" << ifaceHeaderSize << " bytes)..." << std::endl;
            }

            while (total_received < total_expected)
            {
                int recvd = 0;
                recvd = read(socket_remote, &buf[total_received], total_expected - total_received);

                if (recvd == 0) // connection died
                {
                    break;
                }
                if (recvd == -1) // failed to read from socket
                {
                    cb(_onError, "read() failed!");
                    break;
                }

                total_received += recvd;

                if (_verbose)
                {
                    std::printf("(iface header) Received %zu / %zu total bytes from %s\n",
                                total_received, total_expected, hostString(si_other).c_str());
                }
            }

            if (total_received < ifaceHeaderSize) // hit an error above
                break;

            am2b_iface::MsgHeader* iface_header = (am2b_iface::MsgHeader*)buf.data();
            if (_verbose)
            {
                std::cout << "Received am2b_iface::MsgHeader: id = 0x" << std::hex
                          << iface_header->id << std::dec << ", len = " << iface_header->len << std::endl;
            }

            total_expected += iface_header->len;
            if (buf.size() < total_expected)
            {
                if (_verbose)
                {
                    std::cout << "Resizing receive buffer to " << total_expected << " bytes" << std::endl;
                }
                buf.resize(total_expected);
                iface_header = (am2b_iface::MsgHeader*)buf.data(); // update pointer after re-allocation
            }

            while (total_received < total_expected)
            {
                int recvd = 0;
                recvd = read(socket_remote, &buf[total_received], total_expected-total_received);

                if (recvd == 0) // connection died
                    break;
                if (recvd == -1)
                {
                    cb(_onError, "read() failed!");
                    break;
                }

                total_received += recvd;
                if (_verbose)
                {
                    std::printf("(message) Received %zu / %zu total bytes from %s\n",
                                total_received, total_expected, hostString(si_other).c_str());
                }
            }
            
            if (total_received < total_expected) // hit an error above
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
                std::cout << "Received non-vision message type '0x"
                          << std::hex << iface_header->id << std::dec
                          << "' on vision port. Discarding it..." << std::endl;
                continue;
            }

            am2b_iface::VisionMessageHeader* visionHeader = (am2b_iface::VisionMessageHeader*)(buf.data() + sizeof(am2b_iface::MsgHeader));
            if (_verbose)
            {
                std::cout << "Received VisionMessageHeader: " << *visionHeader << std::endl;
                std::cout << "Should have " << visionHeader->len + sizeof(am2b_iface::VisionMessageHeader)
                          << " bytes (got " << total_received << ")" << std::endl;
            }

            switch (visionHeader->type)
            {
                case am2b_iface::Message_Type::Obstacle:
                {
                    cb(_onObstacleMessage,
                       (am2b_iface::ObstacleMessage*)
                       (buf.data() + sizeof(am2b_iface::VisionMessageHeader) + sizeof(am2b_iface::MsgHeader)));
                    break;
                }
                case am2b_iface::Message_Type::Surface:
                {
                    cb(_onSurfaceMessage,
                       (am2b_iface::SurfaceMessage*)
                       (buf.data() + sizeof(am2b_iface::VisionMessageHeader) + sizeof(am2b_iface::MsgHeader)));
                    break;
                }
                case am2b_iface::Message_Type::RGB_Image:
                {
                    am2b_iface::RGBMessage* message = (am2b_iface::RGBMessage*)
                                                      (buf.data()
                                                       + sizeof(am2b_iface::VisionMessageHeader) + sizeof(am2b_iface::MsgHeader));
                    message->pixels = (unsigned char*)((char*)message + sizeof(am2b_iface::RGBMessage)); // fix pointer
                    cb(_onRGBMessage, message);
                    break;
                }
                case am2b_iface::Message_Type::PointCloud:
                {
                    am2b_iface::PointCloudMessage* message = (am2b_iface::PointCloudMessage*)
                                                             (buf.data()
                                                             + sizeof(am2b_iface::VisionMessageHeader) + sizeof(am2b_iface::MsgHeader));
                    message->data = (unsigned char*)((char*)message + sizeof(am2b_iface::PointCloudMessage));
                    cb(_onPointCloudMessage, message);
                    break;
                }
                default:
                {
                    std::cout << "UNKNOWN message type: " << visionHeader->type << "!!" << std::endl;
                }
            }
        }

        if (_verbose)
        {
            std::cout << std::endl << "-------------------------------------------------" << std::endl;
            std::cout << "Connection to client " << inet_ntoa(si_other.sin_addr) << ":" << ntohs(si_other.sin_port) << " terminated!" << std::endl;
            std::cout << "-------------------------------------------------" << std::endl << std::endl;
        }
        close(socket_remote);
        cb(_onDisconnect, hostString(si_other));
    }
};

#endif // VISION_LISTENER_H
