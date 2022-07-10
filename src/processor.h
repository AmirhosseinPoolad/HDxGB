#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "memory.h"
#include <cstdint>

namespace Flag {enum Flags {CARRY = 4, HALF_CARRY, SUBTRACT, ZERO};};
namespace ALUOp {enum Operation {ADD, SUB, AND, XOR, OR, CMP, INC, DEC};};
namespace Reg {enum Registers {B, C, D , E, H, L, F, A};};
namespace RegPair{enum RegisterPairs {BC, DE, HL, AF};};

class Processor
{

private:
    Memory *memory;
    // Program Counter
    uint16_t PC;
    // Stack Pointer. Descending full.
    uint16_t SP;
    // Registers: B, C, D, E, H, L, F, A
    // B to L are general porpuse, F is flag register and A is accumulator
    uint8_t reg[8];
    // Interrupt Master Enable
    bool ime;

    void setFlag(enum Flag::Flags flag);
    void resetFlag(enum Flag::Flags flag);
    uint8_t getFlag(enum Flag::Flags flag);
    void ALUOpUpdateFlag(uint8_t acc_pre,uint8_t val, enum ALUOp::Operation op);
    uint16_t getRegisterPair(enum RegPair::RegisterPairs rp);
    void setRegisterPair(enum RegPair::RegisterPairs rp, uint16_t val);
    void stackPush(uint16_t val);
    uint16_t stackPop();
    void CBExecute();
public:
    Processor(Memory *mem);
    void instructionDecode();
    bool checkCondition(uint8_t cc);
};

#endif