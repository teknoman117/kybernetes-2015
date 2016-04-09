#include <kybernetes/network/socket.hpp>
#include <iostream>

int main ()
{
	kybernetes::network::Socket socket;
	if(socket.Connect("kybernetes.local", 11311))
	{
		//void *imageData = 
	}

	return 0;
}