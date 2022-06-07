#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "memory.h"
#include <cstdint>

class Processor
{

private:
    Memory *memory;
    // Program Counter
    uint16_t PC;
    // Stack Pointer
    uint16_t SP;
    // Registers: B, C, D, E, H, L, F, A
    // B to L are general porpuse, F is flag register and A is accumulator
    uint8_t reg[8];

public:
    Processor(Memory *mem);
    void instructionDecode();
    bool checkCondition(uint8_t cc);
};

#endif