#include <kybernetes/utility/posixsignalhandler.hpp>
#include <iostream>
#include <algorithm>

using namespace std;

namespace
{
	kybernetes::utility::PosixSignalHandler *sharedInstance;
	dispatch_once_t                          predicate;
}

namespace kybernetes
{
	namespace utility
	{
		PosixSignalHandler::PosixSignalHandler(dispatch_queue_t queue)
			: queue(queue)
		{
		}

		PosixSignalHandler::~PosixSignalHandler()
		{
		}

		size_t PosixSignalHandler::AttachHandler(int signo, const function<void (void)>& handler)
		{
			// Register this signal to ourself
			struct sigaction sigaction_info ;
	        sigaction_info.sa_handler = PosixSignalHandler::SigactionHandler;
	        sigemptyset( &sigaction_info.sa_mask ) ;
	        sigaction_info.sa_flags = 0 ;

	        struct sigaction old_action ;
	        if ( sigaction( signo, &sigaction_info, &old_action ) < 0 )
	        {
	            cout << "WHAT THE FUCK???" << endl;
	            return -1;
	        }

			handlers.push_back(PosixSignalHandler::handler_t(index, signo, handler));
			return index++;
		}

		void PosixSignalHandler::DetachHandler(size_t id)
		{
			handlers.erase(remove_if(handlers.begin(), 
				                     handlers.end(), 
				                     [id] (const handler_t& h) -> bool {return get<0>(h) == id;}), 
			               handlers.end());
		}

		void PosixSignalHandler::SigactionHandler(int signo)
		{
			PosixSignalHandler::Instance()->SigactionDispatch(signo);
		}

		void PosixSignalHandler::SigactionDispatch(int signo)
		{
			for_each(handlers.begin(), handlers.end(), [this, signo] (const handler_t& handler)
			{
				if(get<1>(handler) == signo)
				{
					dispatch_async(queue, ^{ get<2>(handler)(); });
				}
			});	
		}

		PosixSignalHandler* PosixSignalHandler::Instance()
		{
			dispatch_once(&predicate, ^
			{
				sharedInstance = new PosixSignalHandler(dispatch_get_main_queue());
			});

			return sharedInstance;
		}
	}
}