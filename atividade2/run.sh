#!/bin/bash

clear

LIBRARY="object_detector"
PROGRAM="main"

#Compile
g++ -c $LIBRARY.cpp -o $LIBRARY.o -fopenmp `pkg-config --cflags --libs opencv`

if [ -f $LIBRARY.o ]
then
	g++ -c $PROGRAM.cpp -o $PROGRAM.o -fopenmp `pkg-config --cflags --libs opencv`
	
	if [ -f $PROGRAM.o ]
	then
		g++ $PROGRAM.o $LIBRARY.o -o $PROGRAM.bin -fopenmp `pkg-config --cflags --libs opencv`
	fi
fi

#Run
if [ -f $PROGRAM.bin ]
then
	./$PROGRAM.bin
	rm $PROGRAM.o
	rm $LIBRARY.o
	rm $PROGRAM.bin
fi
