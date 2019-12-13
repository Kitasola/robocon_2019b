Do: main.o pigpiod.o i2c.o serial.o gy521.o motor_serial.o dualshock3.o
	g++ -Wall -o Do main.o pigpiod.o i2c.o serial.o gy521.o motor_serial.o dualshock3.o -std=c++17 -pthread -lpigpiod_if2 -lrt
pigpiod.o: raspi_utility/src/pigpiod.cpp
	g++ -Wall -c raspi_utility/src/pigpiod.cpp -std=c++17 -pthread -lpigpiod_if2 -lrt
i2c.o: raspi_utility/src/i2c.cpp
	g++ -Wall -c raspi_utility/src/i2c.cpp -std=c++17 -pthread -lpigpiod_if2 -lrt
serial.o: raspi_utility/src/serial.cpp
	g++ -Wall -c raspi_utility/src/serial.cpp -std=c++17 -pthread -lpigpiod_if2 -lrt
gy521.o: raspi_utility/src/gy521.cpp 
	g++ -Wall -c raspi_utility/src/gy521.cpp -std=c++17 -pthread -lrt
motor_serial.o: raspi_utility/src/motor_serial.cpp
	g++ -Wall -c raspi_utility/src/motor_serial.cpp -std=c++17 -pthread -lrt
dualshock3.o: raspi_utility/src/dualshock3.cpp
	g++ -Wall -c raspi_utility/src/dualshock3.cpp -std=c++17 -pthread
main.o: main.cpp
	g++ -Wall -c main.cpp -std=c++17 -pthread -lrt
clean:
	rm -f *.o Do
