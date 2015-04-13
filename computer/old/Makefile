CC=clang
CXX=clang++
#CFLAGS=-std=c11 -O3 -g -march=armv7-a
#CXXFLAGS=-std=c++11 -O3 -g -march=armv7-a
#LDFLAGS=-O3 -lserial -lpthread -lopencv_core -lopencv_imgproc -march=armv7-a
CFLAGS=-std=c11 -O3 -fblocks -g
CXXFLAGS=-std=c++11 -O3 -fblocks -g
LDFLAGS=-O3 -fblocks -ldispatch -lserial -lpthread -lopencv_core -lopencv_imgproc

SOURCES=application.cpp serialport.cpp serialdispatchdevice.cpp utility.cpp serialdevice.cpp garmingps.cpp sensor_controller.cpp motion_controller.cpp uvccamera.cpp
OBJECTS=$(SOURCES:.cpp=.o)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.s
	$(CC) -c -o $@ $<

all: robomagellan blob_track_demo test

robomagellan: $(OBJECTS) robomagellan.o
	$(CXX) $(LDFLAGS) -o robomagellan $(OBJECTS) robomagellan.o

blob_track_demo: $(OBJECTS) blob_track_demo.o
	$(CXX) $(LDFLAGS) -o blob_track_demo $(OBJECTS) blob_track_demo.o /usr/local/lib/libopencvblobslib.a

test: $(OBJECTS) test.o
	$(CXX) $(LDFLAGS) -o test $(OBJECTS) test.o

clean:
	rm -rf robomagellan blob_track_demo test *.o
