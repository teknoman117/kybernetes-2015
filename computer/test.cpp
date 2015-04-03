#include <iostream>
#include "motion_controller.hpp"

using namespace std;
using namespace kybernetes;

int main (int argc, char** argv)
{
    // Get the main dispatch queue
    MotionController controller("/dev/kybernetes/motion_controller");

    promise<void> p;
    bool done = false;
    controller.RegisterAlertHandler([&p, &done] (string s)
    {
        if(s == "READY" && !done)
        {
            p.set_value();
            done = true;
        }
    });

    p.get_future().wait();

    std::cout << "Sending arming request" << endl;
    future<string> response = controller.RequestARM();
    response.wait();
    std::cout << "Got: " << response.get() << endl;

    controller.Join();

    return 0;
}
