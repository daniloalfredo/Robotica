#!/bin/bash

clear

LIBRARY="object_detector"
PROGRAM="main"

g++ -c $LIBRARY.cpp -o $LIBRARY.o -fopenmp `pkg-config --cflags --libs opencv`
g++ -c $PROGRAM.cpp -o $PROGRAM.o -fopenmp `pkg-config --cflags --libs opencv`
g++ $PROGRAM.o $LIBRARY.o -o $PROGRAM.bin -fopenmp `pkg-config --cflags --libs opencv`

if [ -f $PROGRAM.bin ]
then
	./$PROGRAM.bin
	rm $PROGRAM.bin
fi
