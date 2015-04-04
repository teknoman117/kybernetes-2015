#include <iostream>
#include <thread>
#include <chrono>
#include "motion_controller.hpp"

using namespace std;
using namespace kybernetes;

MotionController *controller = NULL;

void ArmDevice()
{
    while(1)
    {
        future<string> response = controller->RequestArm();
        response.wait();
        if(response.get() == "OK")
        {
            break;
        }
        else
        {
            cerr << "Warn: Failed to arm device" << endl;

            std::chrono::duration<double, std::milli> delay(250.0);
            std::this_thread::sleep_for(delay);
        }
    }
}

int main (int argc, char** argv)
{
    // Get the main dispatch queue
    controller = new MotionController("/dev/ttyACM0");

    // Defer to secondary thread
    std::thread([] ()
    {
        promise<void> p;
        bool done = false;
        cout << "Waiting for motion controller to be ready" << endl;
        controller->RegisterAlertHandler([&p, &done] (string s)
        {
            if(s == "HEARTBEAT" && !done)
            {
                p.set_value();
                done = true;
            }
        });
        p.get_future().wait();

        // Arm the device
        ArmDevice();
        cout << "Device armed" << endl;



        controller->Join();


    }).join();

    return 0;
}
