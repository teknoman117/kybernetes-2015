#include <kybernetes/utility/application.hpp>
#include <csignal>
#include <cstdlib>

using namespace std;

namespace kybernetes
{
    namespace utility
    {
        Application::Delegate::~Delegate() {}
        unique_ptr<Application> Application::instance;

        // Constructor of the application
        Application::Application(int argc, char** argv, unique_ptr<Delegate> delegate_, Application::ctor_cookie)
            : delegate(move(delegate_))
        {
            // Register signal handlers for POSIX signals
            signal(SIGINT, &Application::CatchPOSIXSignal);

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

        void Application::Exit()
        {
            dispatch_async(dispatch_get_main_queue(), ^
            {
                delegate->ApplicationWillTerminate();
                exit(0);
            });
        }

        // POSIX Signal Handler (serves to redirect the signals into the application singleton)
        void Application::CatchPOSIXSignal(int signalNumber)
        {
            Application::Instance()->HandlePOSIXSignal(signalNumber);
        }

        void Application::HandlePOSIXSignal(int signalNumber)
        {
            switch(signalNumber)
            {
                case SIGINT:
                    Exit();
                    break;

                default:
                    break;
            }
        }
    }
}
