/*
 *  serversocket.hpp
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

#ifndef _NETWORKING_SERVER_SOCKET_HPP_
#define _NETWORKING_SERVER_SOCKET_HPP_

// Unix includes
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

// Internal
#include <kybernetes/network/socket.hpp>

// Kybernetes namespace
namespace kybernetes
{
    // Networking namespace
    namespace network
    {
        // Server socket object
        class ServerSocket
        {
        private:
            // Server socket descriptor
            int                mDescriptor;
            int                mPort;
            struct sockaddr_in mAddress;

        public:
            ServerSocket();
            ~ServerSocket();

            // Listening functions
            bool StartListening(int port, int backlog = 3);   
            void StopListening();                             
            bool IsListening() const;

            // Accept a connection
            bool AcceptConnection(Socket &socket);
            int  GetHandle() const;
            bool SetBlocking(bool blocking = true);
        };
    }
}

#endif
