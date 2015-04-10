#include "application.hpp"

namespace kybernetes
{
    // Dispatch application wrapper
    Application* Application::instance;

    // Constructor of the application
    Application::Application(int argc, char** argv, Delegate *delegate)
        : delegate(delegate)
    {
        // Register signal handlers

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
}
