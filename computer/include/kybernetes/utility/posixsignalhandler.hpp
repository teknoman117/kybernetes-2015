#ifndef __POSIXSIGNALHANDLER_HPP__
#define __POSIXSIGNALHANDLER_HPP__

#include <cerrno>
#include <csignal>
#include <vector>
#include <tuple>
#include <functional>

#include <dispatch/dispatch.h>

namespace kybernetes
{
	namespace utility
	{
		class PosixSignalHandler
		{
			typedef std::tuple<size_t, int, std::function<void (void)>> handler_t;

			std::vector<handler_t>                                      handlers;
			dispatch_queue_t                                            queue;
			size_t                                                      index;

			static void SigactionHandler(int);
			void SigactionDispatch(int);

		public:
			PosixSignalHandler(dispatch_queue_t queue);
			~PosixSignalHandler();

			size_t AttachHandler(int signal, const std::function<void (void)>&);
			void   DetachHandler(size_t handler);

			static PosixSignalHandler* Instance();
		};
	}
}

#endif
