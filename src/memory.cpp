#include "memory.h"
#include <cstdint>
#include <cstdlib>

Memory::Memory() { memory = (uint8_t *)malloc(sizeof(uint8_t) * (2 << 16)); }

Memory::~Memory(){
    free(memory);
}

uint8_t Memory::getByte(int16_t address) { return memory[address]; }

void Memory::setByte(int16_t address, uint8_t value) {
  memory[address] = value;
}