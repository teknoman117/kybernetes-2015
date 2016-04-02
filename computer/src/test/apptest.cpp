#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <memory>

#include <kybernetes/utility/application.hpp>
#include <kybernetes/cv/cv.hpp>

using namespace std;
using namespace kybernetes::utility;

class TestApplication : public Application::Delegate
{
	uint8_t testPixels[32];
	uint8_t testPixelResults[16];

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
    	for(int i = 0; i < 32; i++)
    	{
    		if(i%4==0 && i >=16 )
    			testPixels[i] = 20;
    		else
    			testPixels[i] = 128;
    	}

    	kybernetes::cv::yuv422_bithreshold((void *) &testPixels[0], (void *) &testPixelResults[0], 16, 1, 64, 64, 64, 128+64, 128+64, 128+64);

    	cout << "results = ";
    	for(int i = 0; i < 16; i++)
    	{
    		cout << (int) testPixelResults[i] << " ";
    	}
    	cout << endl;
    }

    void ApplicationWillTerminate()
    {
        cout << "Termination Requested" << endl;
    }
};

int main (int argc, char** argv)
{
    Application::Run<TestApplication>(argc, argv);
    return 0;
}

