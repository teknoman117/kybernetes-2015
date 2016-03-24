#ifndef __APPLICATION_HPP__
#define __APPLICATION_HPP__

#include <dispatch/dispatch.h>
#include <memory>

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
                
                virtual ~Delegate() = 0;
            };

        private:
            struct ctor_cookie {};

            // Kill the application
            static void SignalTerminateApplication(int);

            // Shared instance of the application
            static std::unique_ptr<Application> instance;
            std::unique_ptr<Delegate>           delegate;

            // Constructor of the application
            void HandleTerminateApplication();

        public:
            static Application* Instance();

            // Run this application with a delegate class
            template<class T>
            static void Run(int argc, char **argv)
            {
                dispatch_async(dispatch_get_main_queue(), ^
                {
                    std::unique_ptr<Delegate> delegate(new T());
                    instance = std::make_unique<Application>(argc, argv, std::move(delegate), ctor_cookie());
                });
                dispatch_main();
            }

            explicit Application(int argc, char** argv, std::unique_ptr<Delegate> delegate, ctor_cookie);
        };
    }
}

#endif
