CC=clang
CXX=clang++
CFLAGS=-std=c11 -O3
CXXFLAGS=-std=c++11 -O3
LDFLAGS=-O3 -lserial -lpthread

SOURCES=serialdevice.cpp garmingps.cpp
OBJECTS=$(SOURCES:.cpp=.o)

%.o: %.cpp $(SOURCES)
	$(CXX) -c -o $@ $< $(CXXFLAGS)

%.o: %.c $(SOURCES)
	$(CC) -c -o $@ $< $(CFLAGS)

all: robomagellan test

robomagellan: $(OBJECTS) robomagellan.o
	$(CXX) $(LDFLAGS) -o robomagellan $(OBJECTS) robomagellan.o

test: $(OBJECTS) test.o
	$(CXX) $(LDFLAGS) -o test $(OBJECTS) test.o

clean:
	rm -rf robomagellan test *.o
