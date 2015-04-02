#include "garmingps.hpp"

int main ()
{
    // Get the main dispatch queeu
    kybernetes::GarminGPS gps("/dev/ttyUSB0", dispatch_get_main_queue());
    gps.Open(^(bool success, std::string error)
    {
        if(success)
            std::cout << "GPS successfully opened" << std::endl;
        else
            std::cout << "GPS failed to open: " << error << std::endl;
    });

    dispatch_main();
    //while(1);

    return 0;
}
