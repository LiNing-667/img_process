#include <stdio.h>
#include "dev_screen.h"

int screen_init(void) {
    printf("[Screen] Initialized 128x64 Display.\n");
    return 0;
}

void screen_show_string(int row, const char *str) {
    printf("[Screen Row %d]: %s\n", row, str);
}

void screen_clear(void) {}