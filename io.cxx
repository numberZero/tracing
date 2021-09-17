#include "io.hxx"
#include <cstdio>
#include <exception>
#include <system_error>
#include <vector>

bytearray read_file(std::string const &filename)
{
	long length;
	FILE *f = fopen(filename.c_str(), "rb"); // nothrow
	if (!f)
		throw std::system_error(errno, std::generic_category(), "Can't open file " + filename);
	fseek(f, 0, SEEK_END);
	length = ftell(f);
	fseek(f, 0, SEEK_SET);
	try {
		bytearray buf(length); // can throw! (bad_alloc)
		if (fread(buf.data(), length, 1, f) != 1)
			throw std::system_error(errno, std::generic_category(), "Can't read from file " + filename);
		fclose(f);
		return buf;
	} catch(...) {
		fclose(f);
		throw;
	}
}
