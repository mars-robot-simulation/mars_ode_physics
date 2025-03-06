#include "logging.hpp"
#include <cstdio>
#include <cstdarg>

void myprint(const std::string format, ...)
{
    va_list args;
    va_start (args, format);
    vfprintf(stderr, format.c_str(), args);
    va_end (args);
    fprintf(stderr, "\n");
}

void printError(const std::string format, ...)
{
    std::string errf = "\033[1;31merror: "+format+"\033[0m";
    va_list args;
    va_start (args, format);
    vfprintf(stderr, errf.c_str(), args);
    va_end (args);
    fprintf(stderr, "\n");
}
