#include <kybernetes/utility/application.hpp>
#include <csignal>
#include <cstdlib>

using namespace std;

namespace kybernetes
{
    namespace utility
    {
        // Dispatch application wrapper
        unique_ptr<Application> Application::instance;
        Application::Delegate::~Delegate() {}

        // Constructor of the application
        Application::Application(int argc, char** argv, unique_ptr<Delegate> delegate_, Application::ctor_cookie)
            : delegate(move(delegate_))
        {
            // Register signal handlers
            signal(SIGINT, &Application::SignalTerminateApplication);

            // Assuming everything went well
            dispatch_async(dispatch_get_main_queue(), ^
            {
                delegate->ApplicationDidLaunch(this, argc, argv);
            });
        }

        Application* Application::Instance()
        {
            return instance.get();
        }

        void Application::SignalTerminateApplication(int)
        {
            Application::Instance()->HandleTerminateApplication();
        }

        void Application::HandleTerminateApplication()
        {
            dispatch_async(dispatch_get_main_queue(), ^
            {
                delegate->ApplicationWillTerminate();
                exit(0);
            });
        }
    }
}
