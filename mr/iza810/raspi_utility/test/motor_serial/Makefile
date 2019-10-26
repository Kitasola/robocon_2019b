CXX = g++
CXXFLAGS = -Wall -std=c++11
GPIOLDFLAGS = -lpigpiod_if2 -pthread -lrt
ROSINCLUDE = -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib -Wl,-rpath,/opt/ros/indigo/lib -lroscpp -lrosconsole -lrostime \
						 -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib

test: test.o motor_serial.o serial.o pigpiod.o
		$(CXX) -o $@ $^ $(CXXFLAGS) $(GPIOLDFLAGS) $(ROSINCLUDE)
pigpiod.o: ../../src/pigpiod.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS)
serial.o: ../../src/serial.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS)
motor_serial.o: ../../src/motor_serial.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS) $(ROSINCLUDE)
test.o: test.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(ROSINCLUDE)

clean:
		rm -f *.o test
