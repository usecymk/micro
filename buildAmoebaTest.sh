#!/bin/bash
/usr/bin/clang++ -std=gnu++14 -g \
  -stdlib=libc++ \
  -isysroot /Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk \
  -I/Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk/usr/include/c++/v1 \
  -I./include \
  -I/opt/homebrew/include/eigen3 \
  amoebatest.cpp ./libraries/libraylib.5.5.0.dylib \
  -framework IOKit -framework Cocoa -framework OpenGL \
  -o amoebatest && ./amoebatest
