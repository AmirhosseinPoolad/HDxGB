#include <cstdio>
#include <string>
#include "disassembly.h"
#include "processor.h"
#include "log.h"

void logGB(const char* str)
{
    printf("%s", str);
    printf("\n");
}

void logGB(DisassemblyObject disObj)
{
    disObj.print(stdout);
}