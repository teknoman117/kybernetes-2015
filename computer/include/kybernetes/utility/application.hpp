#ifndef __APPLICATION_HPP__
#define __APPLICATION_HPP__

#include <dispatch/dispatch.h>

namespace kybernetes
{
    namespace utility
    {
        // Dispatch application wrapper
        class Application
        {
        public:
            // Delegate class
            class Delegate
            {
            public:
                virtual void ApplicationDidLaunch(Application *application, int argc, char **argv) = 0;
                virtual void ApplicationWillTerminate() = 0;
            };

        private:
            // Kill the application
            static void SignalTerminateApplication(int);

            // Shared instance of the application
            static Application *instance;
            Delegate           *delegate;

            // Constructor of the application
            Application(int argc, char** argv, Delegate *delegate);
            void HandleTerminateApplication();

        public:
            static Application* Instance();

            // Run this application with a delegate class
            template<class T>
            static void Run(int argc, char **argv)
            {
                dispatch_async(dispatch_get_main_queue(), ^
                {
                    instance = new Application(argc, argv, new T());
                });
                dispatch_main();
            }
        };
    }
}

#endif
