#include <kybernetes/utility/pollhandler.hpp>
#include <algorithm>

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <iostream>

using namespace std;

namespace
{
    kybernetes::utility::PollHandler *sharedInstance;
    dispatch_once_t                   predicate;
}

namespace kybernetes
{
    namespace utility
    {
        PollHandler::PollHandler(dispatch_queue_t queue)
            : mQueue(queue), mEpollFd(-1), mEpollFdCount(0)
        {
            // Create the epoll structure to monitor file descriptors
            mEpollFd = epoll_create1 (0);
            if (mEpollFd == -1)
            {
                return;
            }

            // Create an event descriptor to allow us to unblock epoll
            mEpollNotifyFd = eventfd(0, EFD_NONBLOCK);
            if (mEpollNotifyFd == -1)
            {
                return;
            }
            AttachHandler(mEpollNotifyFd, [] () {});

            // Start the event generating thread
            mMonitor = thread([this] ()
            {
                std::vector<struct epoll_event> events;

                // Run until the kill request occurs
                while(!mMonitorKill)
                {
                    // Check for any events which are ready to be handled
                    size_t count = mEpollFdCount.load();
                    events.resize(count);
                    if(epoll_wait (mEpollFd, events.data(), count, -1) > 0)
                    {
                        lock_guard<mutex> lock(mContentMutex);
                        for_each(events.begin(), events.end(), [this] (struct epoll_event& e)
                        {
                            if(e.events & EPOLLIN)
                            {
                                dispatch_async(mQueue, ^{ mHandlers[e.data.fd](); });
                            }
                        });
                    }
                }
            });
            mMonitor.detach();
        }

        PollHandler::~PollHandler()
        {
            mMonitorKill = true;
            Interrupt();
            mMonitor.join();
        }

        bool PollHandler::IsActive() const
        {
            return !(mEpollFd == -1); 
        }

        void PollHandler::Interrupt()
        {
            const int i = 42;
            write(mEpollNotifyFd, &i, sizeof(const int));
        }

        bool PollHandler::AttachHandler(int fd, function<void (void)>&& handler)
        {
            // Add the handler (will fail on exist, this is desired)
            struct epoll_event event = 
            {
                .events = EPOLLIN | EPOLLET,
                .data = { .fd = fd },
            };

            if (epoll_ctl (mEpollFd, EPOLL_CTL_ADD, fd, &event) == -1)
                return false;

            lock_guard<mutex> lock(mContentMutex);
            mHandlers[fd] = move(handler);
            mEpollFdCount++;
            return true;
        }

        bool PollHandler::DetachHandler(int fd)
        {
            // Remove the handlers (will fail on not exist, this is desired)
            if (epoll_ctl (mEpollFd, EPOLL_CTL_DEL, fd, nullptr) == -1)
                return false;

            lock_guard<mutex> lock(mContentMutex);
            mHandlers.erase(mHandlers.find(fd));
            mEpollFdCount--;
            return true;
        }

        PollHandler* PollHandler::Instance()
        {
            dispatch_once(&predicate, ^
            {
                sharedInstance = new PollHandler(dispatch_get_main_queue());
            });

            return sharedInstance;
        }
    }
}
