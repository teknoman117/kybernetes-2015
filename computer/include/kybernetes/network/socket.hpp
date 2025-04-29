/*
 *  socket.hpp
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
 
#ifndef _NETWORKING_SOCKET_HPP_
#define _NETWORKING_SOCKET_HPP_

// Unix includes
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include <cstring>
#include <string>
#include <vector>

// Kybernetes namespace
namespace kybernetes
{
    // Networking namespace
    namespace network
    {
        // A buffered socket
        class Socket
        {
            friend class ServerSocket;

        private:
            // Unix socket information
            struct sockaddr_in               mAddress;
            struct hostent                  *mServer;
            int                              mDescriptor;
            int                              mPort;

        public:
            Socket();
            Socket(Socket&& socket);
            ~Socket();

            Socket&     operator=(Socket&& rhs);

            bool        Connect(const std::string& host, int port);
            void        Disconnect();

            // Buffer access controls
            bool        Read(void* data, size_t size, bool block = true);
            bool        Dump(size_t size);
            bool        Write(void* data, size_t size);

            bool        IsConnected() const;
        };
    }
}

#endif
