#include <kybernetes/utility/application.hpp>
#include <csignal>
#include <cstdlib>

namespace kybernetes
{
    namespace utility
    {
        // Dispatch application wrapper
        Application* Application::instance;

        // Constructor of the application
        Application::Application(int argc, char** argv, Delegate *delegate)
            : delegate(delegate)
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
            return instance;
        }

        void Application::SignalTerminateApplication(int)
        {
            Application::Instance()->HandleTerminateApplication();
        }

        void Application::HandleTerminateApplication()
        {
            delegate->ApplicationWillTerminate();
            exit(0);
        }
    }
}
