#include "memory.h"
#include <cstdint>
#include <cstdlib>
#include <cstdio>

Memory::Memory() { memory = (uint8_t *)malloc(sizeof(uint8_t) * (2 << 16)); }

Memory::~Memory()
{
    free(memory);
}

uint8_t Memory::getByte(uint16_t address)
{
    //printf("get byte\n");
    return memory[address];
}

void Memory::setByte(uint16_t address, uint8_t value)
{
    //printf("set byte\n");
    memory[address] = value;
    // serial print
    if(address == 0xFF02 && value == 0x81)
    {
        printf("%c", memory[0xFF01]);
    }
}

void Memory::loadROM(char *path)
{
    printf("opening file...\n");
    FILE* fp = fopen(path, "rb");
    printf("opened file...");

    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    fread(this->memory, sizeof(uint8_t), fsize, fp);
    printf("done\n");
    fclose(fp);
}