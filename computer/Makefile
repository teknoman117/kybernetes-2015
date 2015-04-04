CC=clang
CXX=clang++
CFLAGS=-std=c11 -O3 -g
CXXFLAGS=-std=c++11 -O3 -g
LDFLAGS=-O3 -lserial -lpthread

SOURCES=utility.cpp serialdevice.cpp garmingps.cpp sensor_controller.cpp motion_controller.cpp
OBJECTS=$(SOURCES:.cpp=.o)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

all: robomagellan test

robomagellan: $(OBJECTS) robomagellan.o
	$(CXX) $(LDFLAGS) -o robomagellan $(OBJECTS) robomagellan.o

test: $(OBJECTS) test.o
	$(CXX) $(LDFLAGS) -o test $(OBJECTS) test.o

clean:
	rm -rf robomagellan test *.o
