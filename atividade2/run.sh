#!/bin/bash

clear

MAIN="src/main"
LIBRARY1="src/object_detector"
LIBRARY2="src/timer"
BINARY="detector"

#Compile
g++ -c $LIBRARY1.cpp -o $LIBRARY1.o -fopenmp `pkg-config --cflags --libs opencv`

if [ -f $LIBRARY1.o ]
then
	g++ -c $LIBRARY2.cpp -o $LIBRARY2.o -fopenmp `pkg-config --cflags --libs opencv`
fi
	
if [ -f $LIBRARY2.o ]
then
	g++ -c $MAIN.cpp -o $MAIN.o -fopenmp `pkg-config --cflags --libs opencv`
fi

if [ -f $MAIN.o ]
then
	g++ $MAIN.o $LIBRARY1.o $LIBRARY2.o -o $BINARY.bin -fopenmp `pkg-config --cflags --libs opencv`
fi

#train flag triggers new training
if [ "${1}" == "-train" ]
then
	rm ini/dictionary.yml
fi

#Run
if [ -f $BINARY.bin ]
then
	rm $MAIN.o
	rm $LIBRARY1.o
	rm $LIBRARY2.o
	./$BINARY.bin
	rm $BINARY.bin
fi
