#!/bin/bash

#g++ -std=c++17 -O2 -o decoder main.cpp
#g++ -std=c++17 -O2 -o decoder.exe main.cpp


mkdir build
cd build
cmake ..
cmake --build .

./decoder ../hga22_8khz_u8bit_agc0.raw
./decoder ../hga22_8khz_u8bit_agc1.raw
./decoder ../hga22_8khz_u8bit_agc2.raw
./decoder ../hga22_8khz_u8bit_agc3.raw
./decoder ../hga22_8khz_u8bit_agc4.raw
