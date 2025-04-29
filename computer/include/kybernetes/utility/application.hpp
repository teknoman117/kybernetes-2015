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
            // Cookie to lock out public construction
            struct ctor_cookie {};

            // Singleton application instance
            static std::unique_ptr<Application> instance;

            // Handle a POSIX signal received by the process
            static void CatchPOSIXSignal (int signalNumber);
            void        HandlePOSIXSignal(int signalNumber);

            // Application Delegate
            std::unique_ptr<Delegate> delegate;

        public:
            // Return a singleton instance to the application
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

            // Request the application to terminate
            void Exit();

            explicit Application(int argc, char** argv, std::unique_ptr<Delegate> delegate, ctor_cookie);
        };
    }
}

#endif
