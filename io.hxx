#pragma once
#include <string>
#include <vector>

using bytearray = std::vector<unsigned char>;

bytearray read_file(std::string const &filename);
