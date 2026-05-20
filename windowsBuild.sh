#!/bin/bash
clang++ -std=c++14 -g main.cpp -I/ucrt64/include/eigen3 -lraylib -lopengl32 -lgdi32 -lwinmm -o main.exe
./main.exe