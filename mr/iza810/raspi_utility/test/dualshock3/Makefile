CXX = g++
CXXFLAGS = -Wall -std=c++17
GPIOLDFLAGS = -lpigpiod_if2 -pthread -lrt
INCLUE = -I../../include

test: test.o dualshock3.o 
	$(CXX) -o $@ $^ $(CXXFLAGS) $(GPIOLDFLAGS) $(INCLUE)
		
dualshock3.o: ../../src/dualshock3.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(GPIOLDFLAGS) $(INCLUE)

test.o: test.cpp
		$(CXX) -c $^ $(CXXFLAGS) $(ROSINCLUDE) $(INCLUE)


clean:
		rm -f *.o test
