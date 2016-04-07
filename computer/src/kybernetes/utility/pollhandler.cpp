#include <kybernetes/utility/pollhandler.hpp>
#include <iostream>
#include <algorithm>

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
			mEpollFd = epoll_create1 (0);
			if (mEpollFd == -1)
			{
				cout << "FATAL - PollHandler failed to be created" << endl;
				exit(0);
			}

			mMonitor = thread([this] ()
			{
				std::vector<struct epoll_event> events;

				// Run until the kill request occurs
				while(!mMonitorKill)
				{
					lock_guard<mutex> lock(mContentMutex);
					
					events.resize(mEpollFdCount);

					if(epoll_wait (mEpollFd, events.data(), mEpollFdCount, -1) > 0)
					{
						for_each(events.begin(), events.end(), [this] (struct epoll_event& e)
						{
							if(!(e.events & EPOLLIN))
							{
								cout << "danger wut huh much wat" << endl;
							}
							else
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
			mMonitor.join();
		}

		void PollHandler::AttachHandler(int fd, function<void (void)>&& handler)
		{
			lock_guard<mutex> lock(mContentMutex);

			// No duplicate entries
			if(mHandlers.find(fd) != mHandlers.end())
				return;

			// Add the handler
			mHandlers[fd] = move(handler);
			struct epoll_event event = 
			{
				.events = EPOLLIN | EPOLLET,
				.data = { .fd = fd },
			};

			if (epoll_ctl (mEpollFd, EPOLL_CTL_ADD, fd, &event) == -1)
			{
				cout << "WARN - Failed to add fd to epoll" << endl;
			}
			else
			{
				mEpollFdCount++;
			}
		}

		void PollHandler::DetachHandler(int fd)
		{
			lock_guard<mutex> lock(mContentMutex);

			// can't erase what doesn't exist
			if(mHandlers.find(fd) == mHandlers.end())
				return;

			// Remove the handlers
			mHandlers.erase(mHandlers.find(fd));
			if (epoll_ctl (mEpollFd, EPOLL_CTL_DEL, fd, nullptr) == -1)
			{
				cout << "WARN - Failed to del fd from epoll" << endl;
			}
			else
			{
				mEpollFdCount--;
			}
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
