#!/bin/bash
# Compiles and runs the full bacteria simulation.
# (Replaces the old testing.cpp build.)
/usr/bin/clang++ -std=gnu++14 -g \
  -stdlib=libc++ \
  -isysroot /Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk \
  -I/Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk/usr/include/c++/v1 \
  -I./include \
  bacteria.cpp ./libraries/libraylib.5.5.0.dylib \
  -framework IOKit -framework Cocoa -framework OpenGL \
  -o bacteria && ./bacteria
