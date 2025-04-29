#ifndef __POLLHANDLER_HPP__
#define __POLLHANDLER_HPP__

#include <unordered_map>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>

#include <dispatch/dispatch.h>

namespace kybernetes
{
    namespace utility
    {
        class PollHandler
        {
            std::thread                                         mMonitor;
            std::atomic<bool>                                   mMonitorKill;
            std::mutex                                          mContentMutex;
            std::unordered_map<int, std::function<void (void)>> mHandlers;
            int                                                 mEpollFd;
            int                                                 mEpollNotifyFd;
            std::atomic<size_t>                                 mEpollFdCount;

            dispatch_queue_t                                    mQueue;

        public:
            PollHandler(dispatch_queue_t queue);
            ~PollHandler();

            bool AttachHandler(int fd, std::function<void (void)>&& handler);
            bool DetachHandler(int fd);
            bool IsActive() const;
            void Interrupt();

            static PollHandler* Instance();
        };
    }
}

#endif
