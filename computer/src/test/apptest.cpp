#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <memory>

#include <kybernetes/utility/application.hpp>

using namespace std;
using namespace kybernetes::utility;

class TestApplication : public Application::Delegate
{
public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {

    }

    void ApplicationWillTerminate()
    {
        cout << "Termination Requested" << endl;
    }
};

int main (int argc, char** argv)
{
    Application::Run<TestApplication>(argc, argv);
    return 0;
}

