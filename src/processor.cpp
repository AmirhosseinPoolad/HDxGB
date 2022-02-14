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

  switch (opcode) {
  // LD r[y], r[z]
  // clang-format off
  case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x47: //LD B, r[z]
  case 0x48: case 0x49: case 0x4A: case 0x4B: case 0x4C: case 0x4D: case 0x4F: //LD C, r[z]
  case 0x50: case 0x51: case 0x52: case 0x53: case 0x54: case 0x55: case 0x57: //LD D, r[z]
  case 0x58: case 0x59: case 0x5A: case 0x5B: case 0x5C: case 0x5D: case 0x5F: //LD E, r[z]
  case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x65: case 0x67: //LD H, r[z]
  case 0x68: case 0x69: case 0x6A: case 0x6B: case 0x6C: case 0x6D: case 0x6F: //LD L, r[z]
  case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x77: //LD A, r[z]
    // clang-format on
    reg[y] = reg[z];
    break;
  }
}