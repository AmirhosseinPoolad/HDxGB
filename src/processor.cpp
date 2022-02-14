#include "processor.h"
#include "memory.h"
#include <cstdint>

Processor::Processor(Memory *mem) { memory = mem; }

void Processor::instructionDecode() {
  // Check out this link for GBC's instruction table
  uint8_t opcode = memory->getByte(PC);
  PC++;
  // opcode is broken into:
  //  76543210
  //  xxyyyzzz
  //  --ppq---
  uint8_t x = (opcode & 0b11000000) >> 6;
  uint8_t y = (opcode & 0b00111000) >> 3;
  uint8_t z = (opcode & 0b00000111) >> 0;
  uint8_t p = (opcode & 0b00110000) >> 4;
  uint8_t q = (opcode & 0b00001000) >> 3;

  switch (x) {
  case 0:
    // TODO
    break;
  case 1:
    if (y == 6) {
      // TODO: HALT
    } else {
      // LD r[y], r[z]
      reg[y] = reg[z];
    }
    break;
  case 2:
    // TODO
    break;
  case 3:
    // TODO
    break;
  }
}