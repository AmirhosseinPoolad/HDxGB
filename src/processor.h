#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "memory.h"
#include <cstdint>
class Processor {

private:
  Memory *memory;
  // Program Counter
  int16_t PC;
  // Stack Pointer
  int16_t SP;
  // Accumulator (high byte) and flags(low byte)
  int16_t AF;
  // GP Registers
  int16_t BC;
  int16_t DE;
  int16_t HL;

  public:
  Processor(Memory* mem);
  void instructionDecode();
};

#endif