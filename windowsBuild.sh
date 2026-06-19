#!/bin/bash
clang++ -std=gnu++14 -g \
  -I./include \
  main.cpp src/BoidBehavior.cpp \
  -L./lib -lraylib -lopengl32 -lgdi32 -lwinmm \
  -o main.exe && ./main.exe