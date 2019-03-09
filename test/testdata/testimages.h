#pragma once

#include <vector>
#include <string>
#include <array>

typedef unsigned char BYTE;

const int NUM_TEST_IMAGES=3;
extern const std::array<std::string,NUM_TEST_IMAGES> TestImages;

const size_t TestImageWidth = 640;
const size_t TestImageHeight = 480;

std::vector<BYTE> TestImage( int i );
