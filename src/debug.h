#pragma once
#include <string>
#include <iostream>
const std::string red("\033[0;31m");
const std::string reset("\033[0m");

#define DEBUG_MSG_RELEASE(str)                                                         \
    do {                                                                       \
        std::cerr << str << std::endl;                                         \
    } while (false)

#define DEBUG_MSG_RELEASE_NO_LINE_BREAK(str)                                           \
    do {                                                                       \
        std::cerr << str;                                                      \
    } while (false)

#ifdef DEBUG
#define DEBUG_MSG(str)                                                         \
    do {                                                                       \
        std::cerr << str << std::endl;                                         \
    } while (false)
#define DEBUG_MSG_NO_LINE_BREAK(str)                                           \
    do {                                                                       \
        std::cerr << str;                                                      \
    } while (false)
#define DEBUG_MSG_RED(str)                                                     \
    do {                                                                       \
        std::cerr << red << str << reset << std::endl;                         \
    } while (false)
#define DEBUG_MSG_NO_LINE_BREAK_RED(str)                                       \
    do {                                                                       \
        std::cerr << red << str << reset;                                      \
    } while (false)
#else
#define DEBUG_MSG(str)                                                         \
    do {                                                                       \
    } while (false)
#define DEBUG_MSG_NO_LINE_BREAK(str)                                           \
    do {                                                                       \
    } while (false)
#define DEBUG_MSG_RED(str)                                                     \
    do {                                                                       \
    } while (false)
#define DEBUG_MSG_NO_LINE_BREAK_RED(str)                                       \
    do {                                                                       \
    } while (false)
#endif
