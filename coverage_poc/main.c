#include <stdio.h>
#include "pico/stdlib.h"

#define XSTR(x) STR(x)
#define STR(x) #x

static char ALWAYS_TRUE = 1;

static int __not_in_flash("coverage_buffer") COVERAGE_BUFFER[8192] = { 0 };
static int *COVERAGE_BUFFER_ITEM = COVERAGE_BUFFER;

#define PUSH ((*COVERAGE_BUFFER_ITEM++) = __LINE__); \
_Pragma("message \"coverage instrumentation at line \" XSTR(__LINE__)");


void called() {
	PUSH;
}

void uncalled() {
	PUSH;
}

int main() {
    stdio_init_all();
    called();
    while (ALWAYS_TRUE) {
        printf("Hello, world!\n");
        sleep_ms(1000);
	int *i = COVERAGE_BUFFER;
	do {
		printf("%d\n", *i);
	} while (i++ != COVERAGE_BUFFER_ITEM);
    }
    uncalled();
}
