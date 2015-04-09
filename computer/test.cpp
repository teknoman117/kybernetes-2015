#include <iostream>
#include <thread>
#include <chrono>

#include "kybernetes.hpp"
#include "serialdispatchdevice.hpp"

using namespace std;
using namespace kybernetes;
using namespace kybernetes::io;
using namespace kybernetes::constants;

class Application;
static Application *instance;
class Application
{
    SerialDispatchDevice *gpsDevice;

public:
    Application()
    {
        gpsDevice = new SerialDispatchDevice(GPSPath, dispatch_get_main_queue(), 9600, [] (int error)
        {
            if(error)
            {
                // Do something about it
                cerr << "Error: " << error << endl;
                exit(1);
            }
        });
        gpsDevice->SetHandler([] (const string& message)
        {
            cout << "Received: " << message;
        });
    }

    static void run()
    {
        dispatch_async(dispatch_get_main_queue(), ^
        {
            instance = new Application();
        });
        dispatch_main();
    }
};


int main (int argc, char** argv)
{
    Application::run();
    return 0;
}
