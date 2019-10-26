CXX = g++
CXXFLAGS = -Wall -std=c++11
GPIOLDFLAGS = -lpigpiod_if2 -pthread -lrt
ROSINCLUDE = -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib -Wl,-rpath,/opt/ros/indigo/lib -lroscpp -lrosconsole -lrostime \
						 -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib

test: test.o pigpiod.o rotary_inc.o
		$(CXX) -o $@ $^ $(CXXFLAGS) $(GPIOLDFLAGS) $(ROSINCLUDE)
pigpiod.o: ../../src/pigpiod.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS)
rotary_inc.o: ../../src/rotary_inc.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS) $(ROSINCLUDE)
test.o: test.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(ROSINCLUDE)

clean:
		rm -f *.o test
		
