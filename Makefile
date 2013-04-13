.PHONY: all master.o pedestrian.o car.o pedestrian_behavior.o
CFLAGS=-std=c++11 -g 

all: master

master: master.o pedestrian.o pedestrian_behavior.o
	g++ master.o pedestrian.o pedestrian_behavior.o -o master $(CFLAGS)

master.o: pedestrian.o car.o pedestrian_behavior.o
	g++ -c master.cpp $(CFLAGS)

pedestrian.o:
	g++ -c pedestrian.cpp $(CFLAGS)

car.o:
	g++ -c car.cpp $(CFLAGS)

pedestrian_behavior.o:
	g++ -c pedestrian_behavior.cpp $(CFLAGS)
