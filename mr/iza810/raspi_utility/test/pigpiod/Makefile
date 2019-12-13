CXX = g++
CXXFLAGS = -Wall -std=c++11
GPIOLDFLAGS = -lpigpiod_if2 -pthread -lrt
ROSINCLUDE = -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib -Wl,-rpath,/opt/ros/indigo/lib -lroscpp -lrosconsole -lrostime \
						 -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib

test: test.o pigpiod.o
		$(CXX) -o $@ $^ $(CXXFLAGS) $(GPIOLDFLAGS)
pigpiod.o: ../../src/pigpiod.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS)
test.o: test.cpp
		$(CXX) -c $^ $(CXXFLAGS)

clean:
		rm -f *.o test
		
