#include "processor.h"
#include "memory.h"
#include <cstdint>

Processor::Processor(Memory *mem) { memory = mem; }

void Processor::instructionDecode()
{
  // Check out this link for GBC's instruction table and decoding informations:
  // https://meganesulli.com/generate-gb-opcodes/
  // https://gb-archive.github.io/salvage/decoding_gbz80_opcodes/Decoding%20Gamboy%20Z80%20Opcodes.html
  uint8_t opcode = memory->getByte(PC++);
  // opcode is broken into:
  //  76543210
  //  xxyyyzzz
  //  --ppq---
  // uint8_t x = (opcode & 0b11000000) >> 6;
  uint8_t y = (opcode & 0b00111000) >> 3;
  uint8_t z = (opcode & 0b00000111) >> 0;
  uint8_t p = (opcode & 0b00110000) >> 4;
  // uint8_t q = (opcode & 0b00001000) >> 3;

  switch (opcode)
  {
  // nop
  case 0x00:
    break;
  // TODO: STOP 0x1000

  // 16 bit immediate load
  case 0x01: // LD BC,d16
  case 0x11: // LD DE,d16
  case 0x21: // LD HL,d16
  {
    uint8_t d2 = memory->getByte(PC++); // low byte of d16
    uint8_t d1 = memory->getByte(PC++); // high byte of d16
    reg[2 * p] = d1;
    reg[2 * p + 1] = d2;
  }
  break;
  case 0x31: // LD SP,d16
  {
    // reversing the bytes because gameboy is little endian
    uint16_t d16 = (memory->getByte(PC + 1) << 8) | memory->getByte(PC);
    PC += 2;
    SP = d16;
  }
  break;
  case 0x02: // LD (BC), A
    memory->setByte((reg[0] << 8) | reg[1], reg[7]);
    break;
  case 0x12: // LD (DE), A
    memory->setByte((reg[2] << 8 | reg[3]), reg[7]);
    break;
  case 0x22: // LD (HL+), A
  {
    memory->setByte((reg[4] << 8 | reg[5]), reg[7]);
    // convert two 8 bit registers into a single 16 bit value
    uint16_t HL = (reg[4] << 8) | reg[5];
    // add one to it
    HL++;
    // put it back into the registers
    reg[4] = (HL & 0xFF00) >> 8;
    reg[5] = HL & 0x00FF;
  }
  break;
  case 0x32: // LD (HL-), A
  {
    memory->setByte(((reg[4] << 8) | reg[5]), reg[7]);
    // convert two 8 bit registers into a single 16 bit value
    uint16_t HL = (reg[4] << 8) | reg[5];
    // subtract one from it
    HL--;
    // put it back into the registers
    reg[4] = (HL & 0xFF00) >> 8;
    reg[5] = HL & 0x00FF;
  }
  break;
  // 8 bit immediate load
  case 0x06: // LD B, d8
  case 0x16: // LD D, d8
  case 0x26: // LD H, d8
  case 0x0E: // LD C, d8
  case 0x1E: // LD E, d8
  case 0x2E: // LD L, d8
  case 0x3E: // LD A, d8
    reg[y] = memory->getByte(PC++);
    break;
  // 8 bit immediate indirect load
  case 0x36: // LD (HL), d8
    memory->setByte((reg[4] << 8) | reg[5], memory->getByte(PC++));
    break;
    // LD r[y], r[z]
    // clang-format off
  case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x47: //LD B, r[z]
  case 0x48: case 0x49: case 0x4A: case 0x4B: case 0x4C: case 0x4D: case 0x4F: //LD C, r[z]
  case 0x50: case 0x51: case 0x52: case 0x53: case 0x54: case 0x55: case 0x57: //LD D, r[z]
  case 0x58: case 0x59: case 0x5A: case 0x5B: case 0x5C: case 0x5D: case 0x5F: //LD E, r[z]
  case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x65: case 0x67: //LD H, r[z]
  case 0x68: case 0x69: case 0x6A: case 0x6B: case 0x6C: case 0x6D: case 0x6F: //LD L, r[z]
  case 0x78: case 0x79: case 0x7A: case 0x7B: case 0x7C: case 0x7D: case 0x7F: //LD A, r[z]
    // clang-format on
    reg[y] = reg[z];
    break;

  // 8 bit indirect load
  case 0x70: // LD (HL), B
  case 0x71: // LD (HL), C
  case 0x72: // LD (HL), D
  case 0x73: // LD (HL), E
  case 0x74: // LD (HL), H
  case 0x75: // LD (HL), L
  case 0x77: // LD (HL), A
    memory->setByte((reg[4] << 8) | reg[5], reg[z]);
    break;

  // TODO: HALT instruction
  case 0x76: // HALT
    break;

  // 8 bit indirect load
  case 0x46: // LD B, (HL)
  case 0x56: // LD D, (HL)
  case 0x66: // LD H, (HL)
  case 0x4E: // LD C, (HL)
  case 0x5E: // LD E, (HL)
  case 0x6E: // LD L, (HL)
  case 0x7E: // LD A, (HL)
    reg[y] = memory->getByte((reg[4] << 8) | reg[5]);
    break;

  case 0x0A: // LD A, (BC)
    reg[7] = memory->getByte((reg[0] << 8) + reg[1]);
    break;
  case 0x1A: // LD A, (DE)
    reg[7] = memory->getByte((reg[2] << 8) + reg[3]);
    break;
  case 0x2A: // LD A, (HL+)
  {
    reg[7] = memory->getByte((reg[4] << 8) + reg[5]);
    // convert two 8 bit registers into a single 16 bit value
    uint16_t HL = (reg[4] << 8) | reg[5];
    // add one to it
    HL++;
    // put it back into the registers
    reg[4] = (HL & 0xFF00) >> 8;
    reg[5] = HL & 0x00FF;
    break;
  }
  case 0x3A: // LD A, (HL-)
  {
    reg[7] = memory->getByte((reg[4] << 8) + reg[5]);
    // convert two 8 bit registers into a single 16 bit value
    uint16_t HL = (reg[4] << 8) | reg[5];
    // subtract one from it
    HL--;
    // put it back into the registers
    reg[4] = (HL & 0xFF00) >> 8;
    reg[5] = HL & 0x00FF;
    break;
  }
  // relative jump
  case 0x18: // JR s8
  {
    int8_t offset = memory->getByte(PC++);
    PC += offset;
    break;
  }
  // conditional relative jump
  case 0x20: // JRNZ s8
  case 0x30: // JRNC s8
  case 0x28: // JRZ s8
  case 0x38: // JRC s8
  {
    int8_t offset = memory->getByte(PC++);
    bool condition = checkCondition(4 - y);
    if (condition)
    {
      PC += offset;
    }
    break;
  }
  }
}

// checks flag register for condition code
// cc: 0 NZ, 1 Z, 2 NC, 3 C
bool Processor::checkCondition(uint8_t cc)
{
  // zero flag -> 7th bit of F (zero indexed)
  // carry flag -> 4th bit of F
  if (cc == 0) // not zero
  {
    return !((reg[7] && 0x80) >> 7);
  }
  if (cc == 1) // zero
  {
    return ((reg[7] && 0x80) >> 7);
  }
  if (cc == 2) // not carry
  {
    return !((reg[7] && 0x10) >> 4);
  }
  if (cc == 3) // carry
  {
    return ((reg[7] && 0x10) >> 4);
  }
}