#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "time.h"

#include "SDL.h"

int main()
{
    uint8_t *memory;
    memory = malloc(2 << 16);
    if (memory == NULL)
        return;
}