#!/bin/bash
clang++ -std=gnu++14 -g -I./include main.cpp ./libraries/libraylib.5.5.0.dylib -framework IOKit -framework Cocoa -framework OpenGL -o main
./main