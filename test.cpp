#include <iostream>
#include "utility.hpp"

#include <algorithm>

using namespace std;

int main (int argc, char** argv)
{
    // Get the main dispatch queue
    std::string inputMessage = argv[1];
    vector<string> buffera;
    tokenize(inputMessage, ";", buffera);
    for_each(buffera.begin(), buffera.end(), [] (string& t)
    {
        std::cout << "Found: " << t << endl;
    });

    return 0;
}
