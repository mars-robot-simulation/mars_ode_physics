#pragma once

#ifdef TEST

#include <string>

#if defined(LOG_FATAL)
    #undef LOG_FATAL
    #undef LOG_ERROR
    #undef LOG_WARN
    #undef LOG_INFO
    #undef LOG_DEBUG
#endif

#define LOG_FATAL(...) myprint(__VA_ARGS__);
#define LOG_ERROR(...) printError(__VA_ARGS__);
#define LOG_WARN(...) myprint(__VA_ARGS__);
#define LOG_DEBUG(...) myprint(__VA_ARGS__);
#define LOG_INFO(...) myprint(__VA_ARGS__);

void myprint(const std::string format, ...);
void printError(const std::string format, ...);


#endif
