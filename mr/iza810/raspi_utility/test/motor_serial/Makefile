CXX = g++
CXXFLAGS = -Wall -std=c++11
GPIOLDFLAGS = -lpigpiod_if2 -pthread -lrt

test: test.o motor_serial.o serial.o pigpiod.o
		$(CXX) -o $@ $^ $(CXXFLAGS) $(GPIOLDFLAGS)
pigpiod.o: ../../src/pigpiod.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS)
serial.o: ../../src/serial.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS)
motor_serial.o: ../../src/motor_serial.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS)
test.o: test.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(ROSINCLUDE)

clean:
		rm -f *.o test
