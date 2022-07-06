#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "memory.h"
#include <cstdint>

enum Flag {CARRY = 4, HALF_CARRY, SUBTRACT, ZERO};
enum Operation {ADD, SUB, AND, XOR, OR, CMP};

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

    void setFlag(enum Flag flag);
    void resetFlag(enum Flag flag);
    uint8_t getFlag(enum Flag flag);
    void ALUOpUpdateFlag(uint8_t acc_pre,uint8_t val, int op);

public:
    Processor(Memory *mem);
    void instructionDecode();
    bool checkCondition(uint8_t cc);
};

#endif