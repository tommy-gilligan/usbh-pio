#ifndef RECORD_COVERAGE
#define XSTR(x) STR(x)
#define STR(x) #x
#define RECORD_COVERAGE extern int *COVERAGE_BUFFER_ITEM; \
((*COVERAGE_BUFFER_ITEM++) = __LINE__); \
_Pragma("message \"coverage instrumentation at line \" XSTR(__LINE__)");
#endif
