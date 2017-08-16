#ifndef _POSE_LISTENER_H
#define _POSE_LISTENER_H

///////////////////////////////////////////
// Provides a convenient wrapper around
// communication to receive pose data
// from Lola
// ////////////////////////////////////////

#include <functional>

#include <iface_vis.h>

#include "sock_utils.hpp"

template <typename PoseT>
class PoseListener
{
public:
    typedef std::function<void(PoseT*)> OnNewPose;
    typedef std::function<void(std::string)>  OnError;

    PoseListener(int port, bool verbose) : _port(port), _verbose(verbose)
    {
    }

    void listen()
    {
        if (_listening)
            return;

        _listening = true;

        _socket = create_udp_socket(_port);
        _listener = std::thread(&PoseListener::listen_impl, this);
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
    void onNewPose(OnNewPose cb) { _onNewPose = cb; }
private:
    int         _port;
    int         _socket;
    bool        _verbose   = false;
    bool        _listening = false;
    std::thread _listener;
    size_t      _buflen    = sizeof(PoseT) + 1;

    OnNewPose _onNewPose;
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
        char buf[_buflen];
        sockaddr_in si_other;
        unsigned int slen = sizeof(sockaddr);

        while (_listening)
        {
            // clear buf
            memset(buf, 0, _buflen-1);

            // wait for message
            int nrecvd = recvfrom(_socket, buf, _buflen-1, MSG_DONTWAIT, (sockaddr*)&si_other, &slen);

            if (nrecvd < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    continue;
                else
                    cb(_onError, "Unable to receive data");
                continue;
            }

            if (_verbose)
            {
                std::cout << "[PoseListener] Received " << nrecvd << " bytes from " << hostString(si_other) << std::endl;
            }

            PoseT* new_pose = (PoseT*)buf;
            cb(_onNewPose, new_pose);
        }
    }
};

#endif // _POSE_LISTENER_H
