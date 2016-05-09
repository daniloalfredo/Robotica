#!/bin/bash

clear

PROGRAM="main"
LIBRARY1="object_detector"
LIBRARY2="timer"

#Compile
g++ -c $LIBRARY1.cpp -o $LIBRARY1.o -fopenmp `pkg-config --cflags --libs opencv`

if [ -f $LIBRARY1.o ]
then
	g++ -c $LIBRARY2.cpp -o $LIBRARY2.o -fopenmp `pkg-config --cflags --libs opencv`
	
	if [ -f $LIBRARY2.o ]
	then
		g++ -c $PROGRAM.cpp -o $PROGRAM.o -fopenmp `pkg-config --cflags --libs opencv`
	
		if [ -f $PROGRAM.o ]
		then
			g++ $PROGRAM.o $LIBRARY1.o $LIBRARY2.o -o $PROGRAM.bin -fopenmp `pkg-config --cflags --libs opencv`
		fi
	fi
fi

#Run
if [ -f $PROGRAM.bin ]
then
	./$PROGRAM.bin
	rm $PROGRAM.o
	rm $LIBRARY1.o
	rm $LIBRARY2.o
	rm $PROGRAM.bin
fi
