/*
 *  socket.h
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

#include <kybernetes/network/socket.hpp>

#include <iostream>
#include <cstring>

using namespace kybernetes::network;

Socket::Socket()
    : mServer(nullptr), mDescriptor(-1), mPort(0)
{
}

Socket::Socket(Socket&& socket)
    : mAddress(socket.mAddress), mServer(socket.mServer), mDescriptor(socket.mDescriptor), mPort(socket.mPort)
{
    std::cout << "Move Constructed" << std::endl;

    memset((char *) &socket.mAddress, 0, sizeof(socket.mAddress));

    socket.mServer = nullptr;
    socket.mDescriptor = -1;
    socket.mPort = 0;
}

Socket::~Socket()
{
    Disconnect();
}

Socket& Socket::operator =(Socket&& socket)
{
    std::cout << "Move Assigned" << std::endl;

    mAddress    = socket.mAddress;
    mServer     = socket.mServer;
    mDescriptor = socket.mDescriptor;
    mPort       = socket.mPort;

    memset((char *) &socket.mAddress, 0, sizeof(socket.mAddress));
    socket.mServer     = nullptr;
    socket.mDescriptor = -1;
    socket.mPort       = 0;

    return *this;
}

// Connection command
bool Socket::Connect(const std::string& host, int port)
{
    if(IsConnected())
        return false;

    // Create a UNIX socket
    if((mDescriptor = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        return false;
    }
    
    // Lookup server by the IP or hostname
    if(!(mServer = gethostbyname(host.c_str())))
    {
        Disconnect();
        return false;
    }
   
    // Build the address
    memset((char *) &mAddress, 0, sizeof(mAddress));
    mAddress.sin_family = AF_INET;
    memcpy((char *) &mAddress.sin_addr.s_addr, (char *) mServer->h_addr, mServer->h_length);
    mAddress.sin_port = htons(port);
    mPort = port;
    
    // Connect to the server
    if(connect(mDescriptor,(struct sockaddr *) &mAddress, sizeof(mAddress)) < 0) 
    {
        Disconnect();
        return false;
    }

    // Set a timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // timeout is set to 1/10 sec (100,000 microseconds)
    setsockopt(mDescriptor, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

    return true;
}

void Socket::Disconnect()
{
    close(mDescriptor);

    memset((char *) &mAddress, 0, sizeof(mAddress));
    mServer     = nullptr;
    mDescriptor = -1;
    mPort       = 0;
}

// Check if the socket is connected
bool Socket::IsConnected() const
{
    return (mDescriptor >= 0);
}

// Get current string in the buffer
bool Socket::Read(void* message, size_t size, bool blocking)
{
	if(!IsConnected())
        return false;

    // Perform the recv operation, handling any sort of major error conditions
    int ret = 0;
    while((ret = recv(mDescriptor, message, size, MSG_WAITALL)) != 0)
    {
        // If we are non blocking and the connection failed to receive data, break out
        if(ret < 0 && !blocking)
            return false;

        // Received a message
        if(ret == (int) size)
            return true;
    }

    // Other side of the network disappeared
    if(ret == 0)
        Disconnect();

    // Return an error
    return false;
}

// Throw out data
bool Socket::Dump(size_t size)
{
    if(!IsConnected())
        return false;

    std::vector<unsigned char> m_dump(size);
    return Read(m_dump.data(), size);
}

// Write a string into the write buffer
bool Socket::Write(void* message, size_t size)
{
    if(!IsConnected())
        return false;

    return (send(mDescriptor, message, size, 0) >= 0);
}
