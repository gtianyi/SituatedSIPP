#ifdef DEBUG
#define DEBUG_MSG(str)                                                         \
    do {                                                                       \
        std::cerr << str << std::endl;                                         \
    } while (false)                                                            
#define DEBUG_MSG_NO_LINE_BREAK(str)                                            \
    do {                                                                       \
        std::cerr << str;                                                      \
    } while (false)
#else
#define DEBUG_MSG(str)                                                         \
    do {                                                                       \
    } while (false)
#define DEBUG_MSG_NO_LINE_BREAK(str)                                            \
    do {                                                                       \
    } while (false)
#endif
