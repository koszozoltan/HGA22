mkdir build
cd build
cmake ..
cmake --build .

decoder.exe ../hga22_8khz_u8bit_agc0.raw
decoder.exe ../hga22_8khz_u8bit_agc1.raw
decoder.exe ../hga22_8khz_u8bit_agc2.raw
decoder.exe ../hga22_8khz_u8bit_agc3.raw
decoder.exe ../hga22_8khz_u8bit_agc4.raw
