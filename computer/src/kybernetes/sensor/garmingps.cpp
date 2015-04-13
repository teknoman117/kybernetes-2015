#include <kybernetes/sensor/garmingps.hpp>
#include <kybernetes/constants/constants.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>

using namespace std;
using namespace kybernetes::constants;

namespace kybernetes
{
    namespace sensor
    {
        // Earth properties
        const double R = 6371000;

        // Open the GPS
        GarminGPS::GarminGPS(std::string path, dispatch_queue_t queue, const uint32_t baudrate)
        {
            // Open the GarminGPS device
            device = new io::SerialDispatchDevice(path, queue, baudrate, [] (int error)
            {
                if(error)
                {
                    // do somethign about it
                }
            });

            // Register the handler which will do the processing for the
            device->SetHandler([this] (const string& message)
            {
                // Only continue if the sentence is valid
                if(!IsValidGPSSentence(message))
                    return;

                // Process the data in the message
                GarminGPS::State state;

                // Get the timestamp
                struct tm currentTime;
                char yearString[3] = {message[1], message[2], '\0'};
                currentTime.tm_year = 100 + atoi(yearString);        // assume its currently after 2000
                char monthString[3] = {message[3], message[4], '\0'};
                currentTime.tm_mon = atoi(monthString) - 1;
                char dayString[3] = {message[5], message[6], '\0'};
                currentTime.tm_mday = atoi(dayString);
                char hourString[3] = {message[7], message[8], '\0'};
                currentTime.tm_hour = atoi(hourString);
                char minuteString[3] = {message[9], message[10], '\0'};
                currentTime.tm_min = atoi(minuteString);
                char secondString[3] = {message[11], message[12], '\0'};
                currentTime.tm_sec = atoi(secondString);
                time_t t = timegm(&currentTime);
                state.timestamp = (int32_t) t;
                //cout << asctime(localtime(&t)) << endl;

                // Get the fix status
                state.status = (State::FixStatus) message[30];
                if(state.status != State::Invalid)
                {
                    // Get the latitude
                    double latitudeHem = (message[13] == 'N') ? 1.0 : -1.0;
                    char   latitudeDegString[3] = {message[14], message[15], '\0'};
                    char   latitudeMinString[7] = {message[16], message[17], '.', message[18], message[19], message[20], '\0'};
                    state.latitude = latitudeHem * (atof(latitudeDegString) + (atof(latitudeMinString) / 60.0));

                    // Get the longitude
                    double longitudeHem = (message[21] == 'E') ? 1.0 : -1.0;
                    char   longitudeDegString[4] = {message[22], message[23], message[24], '\0'};
                    char   longitudeMinString[7] = {message[25], message[26], '.', message[27], message[28], message[29], '\0'};
                    state.longitude = longitudeHem * (atof(longitudeDegString) + (atof(longitudeMinString) / 60.0));

                    // Get the error
                    char positionErrorString[4] = {message[31], message[32], message[33], '\0'};
                    state.precision = atof(positionErrorString);

                    // If we have a 3d position, get the altitude
                    if(state.status == State::ThreeDimentional || state.status == State::Differential3D)
                    {
                        double altitudeSign = (message[34] == '+') ? 1.0 : -1.0;
                        char   altitudeString[6] = {message[35], message[36], message[37], message[38], message[39], '\0'};
                        state.altitude = altitudeSign * atof(altitudeString);
                    }

                    // Velocity
                    double velocityXSign = (message[40] == 'E') ? 1.0 : -1.0;
                    char   velocityXString[6] = {message[41], message[42], message[43], '.', message[44], '\0'};
                    state.velocity[0] = velocityXSign * atof(velocityXString);

                    double velocityYSign = (message[45] == 'N') ? 1.0 : -1.0;
                    char   velocityYString[6] = {message[46], message[47], message[48], '.', message[49], '\0'};
                    state.velocity[1] = velocityYSign * atof(velocityYString);

                    double velocityZSign = (message[50] == 'U') ? 1.0 : -1.0;
                    char   velocityZString[6] = {message[51], message[52], '.', message[53], message[54], '\0'};
                    state.velocity[2] = velocityZSign * atof(velocityZString);
                }

                // Push out the event to the registered handler
                if(handler)
                {
                    handler(state);
                }
            });
        }

        GarminGPS::~GarminGPS()
        {
            delete device;
        }

        // Verify that a packet from the GPS is valid
        bool GarminGPS::IsValidGPSSentence(const std::string& sentence)
        {
            if(sentence.size() != 57)
                return false;
            else if(sentence[0] != '@' || sentence[55] != '\r' || sentence[56] != '\n')
                return false;

            // Its probably valid
            return true;
        }

        void GarminGPS::SetHandler(std::function<void (GarminGPS::State& state)> handler)
        {
            this->handler = handler;
        }

        // Initialzation
        GarminGPS::State::State()
        {
            latitude = 0.0;
            longitude = 0.0;
            altitude = 0.0;
            precision = 0.0;
            status = State::Invalid;
            timestamp = 0;
            velocity[0] = 0.0;
            velocity[1] = 0.0;
            velocity[2] = 0.0;
        }

        // Some utility functions
        double GarminGPS::State::DistanceTo(struct State& state)
        {
            // Compute our input variables
            double phi1 = latitude * DegToRad;
            double phi2 = state.latitude * DegToRad;
            double deltaPhi = (state.latitude - latitude) * DegToRad;
            double deltaLambda = (state.longitude - longitude) * DegToRad;

            double a = (sin(deltaPhi/2.0) * sin(deltaPhi/2.0)) +
                       (cos(phi1) * cos(phi2) *
                       (sin(deltaLambda/2.0) * sin(deltaLambda/2.0)));
            double c = 2.0 * atan2(sqrt(a), sqrt(1-a));
            return R * c;
        }

        double GarminGPS::State::HeadingTo(struct State& state)
        {
            // Get everything in radians
            double phi1 = latitude * DegToRad;
            double phi2 = state.latitude * DegToRad;
            double lambda1 = longitude * DegToRad;
            double lambda2 = state.longitude * DegToRad;

            // Temporaries
            double y = sin(lambda2-lambda1) * cos(phi2);
            double x = cos(phi1)*sin(phi2) -
                       sin(phi1)*cos(phi2)*cos(lambda2-lambda1);

            return atan2(y,x) * RadToDeg;
        }
    }
}
