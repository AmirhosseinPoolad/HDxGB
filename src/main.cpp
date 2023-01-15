#include "SDL.h"
#include "memory.h"
#include "processor.h"

int main()
{
    Memory mem = Memory();
    Processor proc = Processor(&mem);
    char path[128];
    scanf("%s", path);
    mem.loadROM(path);
    while(true)
    {
        proc.Tick();
        //getchar();
    }
}