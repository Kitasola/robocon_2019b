CXX = g++
CXXFLAGS = -Wall -std=c++11
GPIOLDFLAGS = -lpigpiod_if2 -pthread -lrt
ROSINCLUDE = -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib -Wl,-rpath,/opt/ros/indigo/lib -lroscpp -lrosconsole -lrostime \
						 -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib

test: test.o GY521.o i2c.o pigpiod.o
		$(CXX) -o $@ $^ $(CXXFLAGS) $(GPIOLDFLAGS) $(ROSINCLUDE)
pigpiod.o: ../../src/pigpiod.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS)
i2c.o: ../../src/i2c.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS)
GY521.o: ../../src/GY521.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS) $(ROSINCLUDE)
test.o: test.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(ROSINCLUDE)

clean:
		rm -f *.o test
		
