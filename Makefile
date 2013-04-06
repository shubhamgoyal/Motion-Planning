all: master.o

master.o: pedestrian.o car.o
	g++ -c master.cpp

pedestrian.o:
	g++ -c pedestrian.cpp

car.o:
	g++ -c car.cpp