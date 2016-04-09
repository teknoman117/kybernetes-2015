/*
 *  serversocket.h
 *
 *  Copyright (c) 2013 Nathaniel Lewis, Robotics Society at UC Merced
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <kybernetes/network/serversocket.hpp>
#include <iostream>
#include <fcntl.h>

using namespace kybernetes::network;

ServerSocket::ServerSocket()
    : mDescriptor(-1), mPort(0)
{
    memset((char *)&mAddress, 0, sizeof(mAddress));
}

ServerSocket::~ServerSocket()
{
    StopListening();
}

// Start listening on a port
bool ServerSocket::StartListening(int port, int backlog)
{
    // Check if we are connected
    if(IsListening())
        return false;

    // Create the socket we are going to use to listen
    if((mDescriptor = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        StopListening();
        return false;
    }

    // Bind the socket to this server's settings
    memset((char *)&mAddress, 0, sizeof(mAddress));
    mAddress.sin_family = AF_INET;
    mAddress.sin_addr.s_addr = INADDR_ANY;
    mAddress.sin_port = htons(port);
    mPort = port;
    if(bind(mDescriptor, (struct sockaddr *) &mAddress, sizeof(mAddress)) < 0)
    {
        StopListening();
        return false;
    }

    // Set the receive timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // timeout is set to 1/10 sec (100,000 microseconds)
    setsockopt(mDescriptor, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

    // Start listening
    if(listen(mDescriptor, backlog) < 0)
    {
        StopListening();
        return false;
    }

    // Return success
    return true;
}

// Stop listening
void ServerSocket::StopListening()
{
    close(mDescriptor);

    mDescriptor = -1;
    mPort = 0;
    memset((char *)&mAddress, 0, sizeof(mAddress));
}

// Accept a connection
bool ServerSocket::AcceptConnection(Socket& socket)
{
    if(!IsListening())
        return false;

    // Accept the connection
    socklen_t clilen = sizeof(socket.mAddress);
    if((socket.mDescriptor = accept(mDescriptor, (struct sockaddr *) &socket.mAddress, &clilen)) < 0)
    {
        socket.Disconnect();
        return false;
    }

    // Copy the port handler and start the buffer
    socket.mPort = mPort;

    // Set the timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // timeout is set to 1/10 sec (100,000 microseconds)
    setsockopt(socket.mDescriptor, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

    // Return success
    return true;
}

// Return if we are successfully listening
bool ServerSocket::IsListening() const
{
    return mDescriptor >= 0;
}


int ServerSocket::GetHandle() const
{
    return mDescriptor;
}

bool ServerSocket::SetBlocking(bool blocking)
{
    int flags;
    int ret;

    // Fetch the current device flags
    ret = fcntl (mDescriptor, F_GETFL, 0);
    if (ret == -1)
    {
        return false;
    }

    // Enable or disable blocking
    flags = blocking ? (ret & ~O_NONBLOCK) : (ret | O_NONBLOCK);

    // Set the flags
    ret = fcntl (mDescriptor, F_SETFL, flags);
    if (ret == -1)
    {
        return false;
    }

    return true;
}

