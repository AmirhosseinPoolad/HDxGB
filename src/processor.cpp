#include "processor.h"
#include <cstdio>
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
    case 0x00: // NOP
        break;
    case 0x10: // STOP
        // TODO: STOP 0x1000
        PC += 2;
        break;
    case 0xF3: // DI
        ime = false;
    case 0xFB:
        ime = true;
    case 0xCB:
        CBExecute();
    // 16 bit immediate load
    case 0x01: // LD BC,d16
    case 0x11: // LD DE,d16
    case 0x21: // LD HL,d16
    {
        uint8_t d2 = memory->getByte(PC++); // low byte of d16
        uint8_t d1 = memory->getByte(PC++); // high byte of d16
        uint16_t val = (d1 << 8) | d1;
        // Note to c++: just use the int coward
        enum RegPair::RegisterPairs rp = static_cast<RegPair::RegisterPairs>(p);
        setRegisterPair(rp, val);
        reg[2 * p] = d1;
        reg[2 * p + 1] = d2;
        break;
    }
    case 0x31: // LD SP,d16
    {
        // reversing the bytes because gameboy is little endian
        uint16_t d16 = (memory->getByte(PC + 1) << 8) | memory->getByte(PC);
        PC += 2;
        SP = d16;
        break;
    }
    case 0x02: // LD (BC), A
        memory->setByte(getRegisterPair(RegPair::BC), reg[7]);
        break;
    case 0x12: // LD (DE), A
        memory->setByte(getRegisterPair(RegPair::DE), reg[7]);
        break;
    case 0x22: // LD (HL+), A
    {
        uint16_t HL = getRegisterPair(RegPair::HL);
        memory->setByte(HL, reg[7]);
        HL++;
        setRegisterPair(RegPair::HL, HL);
        break;
    }
    case 0x32: // LD (HL-), A
    {
        uint16_t HL = getRegisterPair(RegPair::HL);
        memory->setByte(HL, reg[7]);
        HL++;
        setRegisterPair(RegPair::HL, HL);
        break;
    }
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
        memory->setByte(getRegisterPair(RegPair::HL), memory->getByte(PC++));
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
        memory->setByte(getRegisterPair(RegPair::HL), reg[z]);
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
        reg[y] = memory->getByte(getRegisterPair(RegPair::HL));
        break;

    case 0x0A: // LD A, (BC)
        reg[7] = memory->getByte(getRegisterPair(RegPair::BC));
        break;
    case 0x1A: // LD A, (DE)
        reg[7] = memory->getByte(getRegisterPair(RegPair::DE));
        break;
    case 0x2A: // LD A, (HL+)
    {
        uint16_t HL = getRegisterPair(RegPair::HL);
        reg[7] = memory->getByte(HL);
        HL++;
        setRegisterPair(RegPair::HL, HL);
        break;
    }
    case 0x3A: // LD A, (HL-)
    {
        uint16_t HL = getRegisterPair(RegPair::HL);
        reg[7] = memory->getByte(HL);
        HL--;
        setRegisterPair(RegPair::HL, HL);
        break;
    }
    case 0xE0: // LD (a8), A
    {
        uint16_t address = 0xFF00 + memory->getByte(PC++);
        memory->setByte(address, reg[Reg::A]);
        break;
    }
    case 0xF0: // LD A, (a8)
    {
        uint16_t address = 0xFF00 + memory->getByte(PC++);
        reg[Reg::A] = memory->getByte(address);
        break;
    }
    case 0xE2: // LD (C), A
    {
        uint16_t address = 0xFF00 + reg[Reg::C];
        memory->setByte(address, reg[Reg::A]);
        break;
    }
    case 0xF2: // LD A, (C)
    {
        uint16_t address = 0xFF00 + reg[Reg::C];
        reg[Reg::A] = memory->getByte(address);
        break;
    }
    case 0xEA: // LD (a16), A
    {
        uint16_t address = memory->getByte(PC + 1) + memory->getByte(PC);
        PC += 2;
        memory->setByte(address, reg[Reg::A]);
        break;
    }
    case 0xFA: // LD A, (a16)
    {
        uint16_t address = memory->getByte(PC + 1) + memory->getByte(PC);
        PC += 2;
        reg[Reg::A] = memory->getByte(address);
        break;
    }
    // stack operations:
    case 0x08: // LD (a16), SP
    {
        uint8_t d2 = memory->getByte(PC++); // low byte of d16
        uint8_t d1 = memory->getByte(PC++); // high byte of d16
        uint16_t address = (d1 << 8) | d1;
        uint8_t lower, higher;
        lower = SP & 0xFF;
        higher = (SP & 0xFF00) >> 8;
        memory->setByte(address, lower);
        memory->setByte(address + 1, higher);
        break;
    }
    case 0xC1: // POP BC
    case 0xD1: // POP DE
    case 0xE1: // POP HL
    case 0xF1: // POP AF
    {
        enum RegPair::RegisterPairs rp = static_cast<RegPair::RegisterPairs>(p);
        uint16_t popped = stackPop();
        setRegisterPair(rp, popped);
    }
    case 0xC5: // PUSH BC
    case 0xD5: // PUSH DE
    case 0xE5: // PUSH HL
    case 0xF5: // PUSH AF
    {
        enum RegPair::RegisterPairs rp = static_cast<RegPair::RegisterPairs>(p);
        uint16_t regpPair = getRegisterPair(rp);
        stackPush(regpPair);
        break;
    }
    case 0xE8: // ADD SP, s8
    {
        int16_t offset = (int8_t)memory->getByte(PC++);

        if ((SP & 0xF) + (offset & 0xF) > 0xF)
        {
            setFlag(Flag::HALF_CARRY);
        }

        if ((SP & 0xFF)  + (offset & 0xFF) > 0xFF)
        {
            setFlag(Flag::CARRY);
        }

        SP = SP + offset;
        break;
    }
    case 0xF8: // LD HL, SP + s8
    {
        int16_t offset = (int8_t)memory->getByte(PC++);
        // TODO: test this and other s8 instructions to see if casting works as intended
        setRegisterPair(RegPair::HL, SP + offset);

        if ((SP & 0xF) + (offset & 0xF) > 0xF)
        {
            setFlag(Flag::HALF_CARRY);
        }

        if ((SP & 0xFF)  + (offset & 0xFF) > 0xFF)
        {
            setFlag(Flag::CARRY);
        }
        break;
    }
    case 0xF9: // LD SP, HL
    {
        SP = getRegisterPair(RegPair::HL);
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
    // jump
    case 0xC3: // JP d16
    {
        uint16_t d16 = (memory->getByte(PC + 1) << 8) | memory->getByte(PC);
        PC = d16;
        break;
    }
    // conditional jump
    case 0xC2: // JNZ d16
    case 0xD2: // JNC d16
    case 0xCA: // JZ d16
    case 0xDA: // JC d16
    {
        uint16_t d16 = (memory->getByte(PC + 1) << 8) | memory->getByte(PC);
        bool condition = checkCondition(y);
        if (condition)
        {
            PC = d16;
        }
        break;
    }
    case 0xE9: // JP HL
    {
        uint16_t HL = getRegisterPair(RegPair::HL);
        PC = HL;
        break;
    }
    // call and ret
    case 0xCD: // CALL a16
    case 0xC4: // CALL NZ a16
    case 0xD4: // CALL NC a16
    case 0xCC: // CALL Z a16
    case 0xDC: // CALL C a16
    {
        uint16_t a16 = (memory->getByte(PC + 1) << 8) | memory->getByte(PC);
        PC += 2;
        uint8_t cc = y;
        if(opcode == 0xCD) cc = -1;
        if (checkCondition(cc))
        {
            stackPush(PC);
            PC = a16;
        }
        break;
    }
    case 0xC9: // RET
    case 0xC0: // RET NZ
    case 0xD0: // RET NC
    case 0xC8: // RET Z
    case 0xD8: // RET C
    {
        uint8_t cc = y;
        if(opcode == 0xC9) cc = -1;
        if (checkCondition(cc))
        {
            PC = stackPop();
        }
        break;
    }
    case 0xD9: // RETI
    {
        ime = true;
        PC = stackPop();
        break;
    }
    // 8 bit increment
    case 0x04: // INC B
    case 0x0C: // INC C
    case 0x14: // INC D
    case 0x1C: // INC E
    case 0x24: // INC H
    case 0x2C: // INC L
    case 0x3C: // INC A
    {
        ALUOpUpdateFlag(reg[y], 1, ALUOp::INC);
        reg[y] += 1;
        break;
    }
    // indirect increment
    case 0x34: // INC (HL)
    {
        uint16_t HL = getRegisterPair(RegPair::HL);
        uint8_t val = memory->getByte(HL);
        ALUOpUpdateFlag(val, 1, ALUOp::INC);
        val++;
        memory->setByte(HL, val);
        break;
    }
    // 8 bit decrement
    case 0x05: // DEC B
    case 0x0D: // DEC C
    case 0x15: // DEC D
    case 0x1D: // DEC E
    case 0x25: // DEC H
    case 0x2D: // DEC L
    case 0x3D: // DEC A
    {
        ALUOpUpdateFlag(reg[y], 1, ALUOp::DEC);
        reg[y] -= 1;
        break;
    }
    // indirect decrement
    case 0x35: // DEC (HL)
    {
        uint16_t HL = getRegisterPair(RegPair::HL);
        uint8_t val = memory->getByte(HL);
        ALUOpUpdateFlag(val, 1, ALUOp::DEC);
        val--;
        memory->setByte(HL, val);
        break;
    }
    // 16 bit increment
    case 0x03: // INC BC
    case 0x13: // INC DE
    case 0x23: // INC HL
    {
        enum RegPair::RegisterPairs rp = static_cast<RegPair::RegisterPairs>(p);
        uint16_t registerPair = getRegisterPair(rp);
        registerPair++;
        setRegisterPair(rp, registerPair);
        break;
    }
    case 0x33: // INC SP
    {
        SP++;
        break;
    }
    // 16 bit decrement
    case 0x0B: // DEC BC
    case 0x1B: // DEC DE
    case 0x2B: // DEC HL
    {
        enum RegPair::RegisterPairs rp = static_cast<RegPair::RegisterPairs>(p);
        uint16_t registerPair = getRegisterPair(rp);
        registerPair--;
        setRegisterPair(rp, registerPair);

        break;
    }
    case 0x3B: // DEC SP
    {
        SP--;
        break;
    }
    case 0x80: // ADD A, B
    case 0x81: // ADD A, C
    case 0x82: // ADD A, D
    case 0x83: // ADD A, E
    case 0x84: // ADD A, H
    case 0x85: // ADD A, L
    case 0x87: // ADD A, A
    {
        uint8_t acc_pre = reg[7]; //accumulator before alu operation
        uint8_t val = reg[z]; // value to be added
        reg[7] += val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::ADD);
        break;
    }
    case 0x86: // ADD A, (HL)
    {
        uint8_t acc_pre = reg[7]; //accumulator before alu operation
        uint16_t HL = (reg[4] << 8) | reg[5];
        uint8_t val = memory->getByte(HL);
        reg[7] += val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::ADD);
        break;
    }

    case 0x88: // ADC A, B
    case 0x89: // ADC A, C
    case 0x8A: // ADC A, D
    case 0x8B: // ADC A, E
    case 0x8C: // ADC A, H
    case 0x8D: // ADC A, L
    case 0x8F: // ADC A, A
    {
        uint8_t acc_pre = reg[7]; //accumulator before alu operation
        uint8_t val = reg[z] + getFlag(Flag::CARRY); // value to be added
        ALUOpUpdateFlag(acc_pre, val, ALUOp::ADD);
        break;
    }
    case 0x8E: // ADC A, (HL)
    {
        uint8_t acc_pre = reg[7]; //accumulator before alu operation
        uint16_t HL = (reg[4] << 8) | reg[5];
        uint8_t val = memory->getByte(HL) + getFlag(Flag::CARRY);
        reg[7] += val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::ADD);
        break;
    }
    case 0x90: // SUB A, B
    case 0x91: // SUB A, C
    case 0x92: // SUB A, D
    case 0x93: // SUB A, E
    case 0x94: // SUB A, H
    case 0x95: // SUB A, L
    case 0x97: // SUB A, A
    {
        uint8_t acc_pre = reg[7]; //accumulator before alu operation
        uint8_t val = reg[z]; // value to be subtracted
        reg[7] -= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::SUB);
        break;
    }
    case 0x96: // SUB A, (HL)
    {
        uint8_t acc_pre = reg[7]; //accumulator before alu operation
        uint16_t HL = (reg[4] << 8) | reg[5];
        uint8_t val = memory->getByte(HL);
        reg[7] -= val;
        setFlag(Flag::SUBTRACT);
        ALUOpUpdateFlag(acc_pre, val, ALUOp::SUB);
        break;
    }

    case 0x98: // SBC A, B
    case 0x99: // SBC A, C
    case 0x9A: // SBC A, D
    case 0x9B: // SBC A, E
    case 0x9C: // SBC A, H
    case 0x9D: // SBC A, L
    case 0x9F: // SBC A, A
    {
        uint8_t acc_pre = reg[7]; //accumulator before alu operation
        uint8_t val = reg[z] + getFlag(Flag::CARRY); // value to be subtracted
        reg[7] -= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::SUB);
        break;
    }
    case 0x9E: // SBC A, (HL)
    {
        uint8_t acc_pre = reg[7]; //accumulator before alu operation
        uint16_t HL = (reg[4] << 8) | reg[5];
        uint8_t val = memory->getByte(HL) + getFlag(Flag::ZERO);
        reg[7] -= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::SUB);
        break;
    }

    case 0xA0: // AND B
    case 0xA1: // AND C
    case 0xA2: // AND D
    case 0xA3: // AND E
    case 0xA4: // AND H
    case 0xA5: // AND L
    case 0xA7: // AND A
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = reg[z];
        reg[7] &= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::AND);
        break;
    }
    case 0xA6: // AND (HL)
    {
        uint8_t acc_pre = reg[7];
        uint16_t HL = (reg[4] << 8) | reg[5];
        uint8_t val = memory->getByte(HL);
        reg[7] &= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::AND);
        break;
    }
    case 0xA8: // XOR B
    case 0xA9: // XOR C
    case 0xAA: // XOR D
    case 0xAB: // XOR E
    case 0xAC: // XOR H
    case 0xAD: // XOR L
    case 0xAF: // XOR A
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = reg[z];
        reg[7] ^= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::XOR);
        break;
    }
    case 0xAE: // XOR (HL)
    {
        uint8_t acc_pre = reg[7];
        uint16_t HL = (reg[4] << 8) | reg[5];
        uint8_t val = memory->getByte(HL);
        reg[7] ^= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::XOR);
        break;
    }
    case 0xB0: // OR B
    case 0xB1: // OR C
    case 0xB2: // OR D
    case 0xB3: // OR E
    case 0xB4: // OR H
    case 0xB5: // OR L
    case 0xB7: // OR A
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = reg[z];
        reg[7] |= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::OR);
        break;
    }
    case 0xB6: // OR (HL)
    {
        uint8_t acc_pre = reg[7];
        uint16_t HL = (reg[4] << 8) | reg[5];
        uint8_t val = memory->getByte(HL);
        reg[7] |= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::OR);
        break;
    }
    case 0xB8: // CMP B
    case 0xB9: // CMP C
    case 0xBA: // CMP D
    case 0xBB: // CMP E
    case 0xBC: // CMP H
    case 0xBD: // CMP L
    case 0xBF: // CMP A
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = reg[z];
        ALUOpUpdateFlag(acc_pre, val, ALUOp::CMP);
        break;
    }
    case 0xBE: // CMP (HL)
    {
        uint8_t acc_pre = reg[7];
        uint16_t HL = (reg[4] << 8) | reg[5];
        uint8_t val = memory->getByte(HL);
        ALUOpUpdateFlag(acc_pre, val, ALUOp::CMP);
        break;
    }
    // 8 bit immediate ALU
    case 0xC6: // ADD d8
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = memory->getByte(PC++);
        reg[7] += val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::ADD);
        break;
    }
    case 0xCE: // ADC d8
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = memory->getByte(PC++) + getFlag(Flag::CARRY);
        reg[7] += val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::ADD);
        break;
    }
    case 0xD6: // SUB d8
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = memory->getByte(PC++);
        reg[7] -= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::SUB);
        break;
    }
    case 0xDE: // SBC d8
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = memory->getByte(PC++) + getFlag(Flag::CARRY);
        reg[7] -= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::SUB);
        break;
    }
    case 0xE6: // AND d8
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = memory->getByte(PC++);
        reg[7] &= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::AND);
        break;
    }
    case 0xEE: // XOR d8
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = memory->getByte(PC++);
        reg[7] ^= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::XOR);
        break;
    }
    case 0xF6: // OR d8
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = memory->getByte(PC++);
        reg[7] |= val;
        ALUOpUpdateFlag(acc_pre, val, ALUOp::OR);
        break;
    }
    case 0xFE: // CMP d8
    {
        uint8_t acc_pre = reg[7];
        uint8_t val = memory->getByte(PC++);
        ALUOpUpdateFlag(acc_pre, val, ALUOp::CMP);
        break;
    }
    // register pair add
    case 0x09: // ADD HL, BC
    case 0x19: // ADD HL, DE
    case 0x29: // ADD HL, HL
    {
        enum RegPair::RegisterPairs rp = static_cast<RegPair::RegisterPairs>(p);
        uint16_t regPair = getRegisterPair(rp);
        uint16_t HL = getRegisterPair(RegPair::HL);
        resetFlag(Flag::SUBTRACT);
        resetFlag(Flag::ZERO);
        resetFlag(Flag::HALF_CARRY);
        if ((HL + regPair) == 0)
        {
            setFlag(Flag::ZERO);
        }
        if ((HL & 0xFFF) + (regPair & 0xFFF) > 0xFFF)
        {
            setFlag(Flag::HALF_CARRY);
        }
        HL += regPair;
        setRegisterPair(RegPair::HL, HL);
        break;
    }
    case 0x39: // ADD HL, SP
    {
        uint16_t HL = getRegisterPair(RegPair::HL);
        HL += SP;
        setRegisterPair(RegPair::HL, HL);
        break;
    }

    case 0x07: // RLCA
    {
        bool carry = reg[7] & 0x80; // last bit of reg a
        reg[7] = reg[7] << 1;
        if (carry)
        {
            reg[7] |= 0x1; // first bit should be set
            setFlag(Flag::CARRY); // carry flag set
        }
        else
        {
            resetFlag(Flag::CARRY);
        }
        resetFlag(Flag::ZERO);
        resetFlag(Flag::HALF_CARRY);
        resetFlag(Flag::SUBTRACT);
        break;
    }
    case 0x17: // RLA
    {
        bool lbit = reg[7] & 0x80; // last bit of reg a
        uint8_t cflag = getFlag(Flag::CARRY);
        reg[7] = reg[7] << 1;
        if (lbit)
            setFlag(Flag::CARRY);
        else
            setFlag(Flag::CARRY);

        if (cflag)
            reg[7] |= 0x1;
        
        resetFlag(Flag::ZERO);
        resetFlag(Flag::HALF_CARRY);
        resetFlag(Flag::SUBTRACT);
        break;
    }
    case 0x0F: // RRCA
    {
        bool carry = reg[7] & 0x01; // first bit of reg a
        reg[7] = reg[7] >> 1;
        if (carry)
        {
            reg[7] |= 0x80; // last bit should be set
            setFlag(Flag::CARRY); // carry flag set
        }
        else
        {
            resetFlag(Flag::CARRY);
        }
        resetFlag(Flag::ZERO);
        resetFlag(Flag::HALF_CARRY);
        resetFlag(Flag::SUBTRACT);
        break;
    }
    case 0x1F: // RRA
    {
        bool rbit = reg[7] & 0x01; // first bit of reg a
        uint8_t cflag = getFlag(Flag::CARRY);
        reg[7] = reg[7] >> 1;
        if (rbit)
            setFlag(Flag::CARRY);
        else
            setFlag(Flag::CARRY);

        if (cflag)
            reg[7] |= 0x80;
        
        resetFlag(Flag::ZERO);
        resetFlag(Flag::HALF_CARRY);
        resetFlag(Flag::SUBTRACT);
        break;
    }
    case 0x27: // DAA
    {
        // Adjust the accumulator to a BCD number after BCD addition and subtraction operations.
        // algorithm: if subtraction flag is set, subtract 6 from low/high nibble in case of half carry/carry
        // else add 6 to low/high nibble in case of half carry/carry
        // using 16 bits for easier carry detection
        uint16_t result = reg[Reg::A];
        if (getFlag(Flag::SUBTRACT))
        {
            if (getFlag(Flag::HALF_CARRY))
            {
            result = (result - 0x06) & 0xFF;
            }

            if (getFlag(Flag::CARRY))
            {
                result -= 0x60;
            }
        }
        else
        {
            if (getFlag(Flag::HALF_CARRY) || (result & 0x0F) > 0x09)
            {
                result += 0x06;
            }

            if (getFlag(Flag::CARRY) || result > 0x9F)
            {
                result += 0x60;
            }
        }
        if ((result & 0xFF) == 0)
        {
            setFlag(Flag::ZERO);
        }

        if ((result & 0x100) == 0x100)
        {
            setFlag(Flag::CARRY);
        }

        resetFlag(Flag::HALF_CARRY);
        reg[Reg::A] = result & 0xFF;
    }
    case 0x37: // SCF Set Carry Flag
    {
        resetFlag(Flag::SUBTRACT);
        resetFlag(Flag::HALF_CARRY);
        setFlag(Flag::CARRY);
    }
    case 0x2F: // CPL Complement
    {
        reg[7] = ~reg[7];
        setFlag(Flag::SUBTRACT);
        setFlag(Flag::HALF_CARRY);
    }
    case 0x3F: // CCF Flip Carry Flag (Why is it not *Clear* Carry Flag?!)
    {
        resetFlag(Flag::SUBTRACT);
        resetFlag(Flag::HALF_CARRY);
        getFlag(Flag::CARRY) ? resetFlag(Flag::CARRY) : setFlag(Flag::CARRY);
    }
    // reset
    case 0xC7: // RST 0
    case 0xCF: // RST 1
    case 0xD7: // RST 2
    case 0xDF: // RST 3
    case 0xE7: // RST 4
    case 0xEF: // RST 5
    case 0xF7: // RST 6
    case 0xFF: // RST 7
    {
        stackPush(PC);
        PC = y * 8;
    }

    default:
        fprintf(stderr, "Unknown Instruction. Opcode: %x\n", opcode);
        break;
    }
}

// checks flag register for condition code
// cc: -1 none,  0 NZ, 1 Z, 2 NC, 3 C
bool Processor::checkCondition(uint8_t cc)
{
    // zero flag -> 7th bit of F (zero indexed)
    // carry flag -> 4th bit of F
    if (cc == -1)
    {
        return true;
    }
    if (cc == 0) // not zero
    {
        return !getFlag(Flag::ZERO);
    }
    if (cc == 1) // zero
    {
        return getFlag(Flag::ZERO);
    }
    if (cc == 2) // not carry
    {
        return !getFlag(Flag::CARRY);
    }
    if (cc == 3) // carry
    {
        return getFlag(Flag::CARRY);
    }

    return false;
}

void Processor::setFlag(enum Flag::Flags flag)
{
    reg[6] |= (1U << flag);
}
void Processor::resetFlag(enum Flag::Flags flag)
{
    reg[6] &= ~(1U << flag);
}

uint8_t Processor::getFlag(enum Flag::Flags flag)
{
    return (reg[6] >> flag) & 1U;
}

void Processor::ALUOpUpdateFlag(uint8_t acc_pre,uint8_t val, enum ALUOp::Operation op)
{
    switch (op)
    {
    case ALUOp::INC:
        resetFlag(Flag::SUBTRACT);

        if ((uint8_t)(acc_pre + val) == 0)
            setFlag(Flag::ZERO);
        else
            resetFlag(Flag::ZERO);

        if ((acc_pre & 0xF) + (val & 0xF) > 0x0F)
            setFlag(Flag::HALF_CARRY);
        else
            resetFlag(Flag::HALF_CARRY);
        break;

    case ALUOp::ADD:
        resetFlag(Flag::SUBTRACT);

        if ((uint8_t)(acc_pre + val) == 0)
            setFlag(Flag::ZERO);
        else
            resetFlag(Flag::ZERO);

        if ((acc_pre & 0xF) + (val & 0xF) > 0x0F)
            setFlag(Flag::HALF_CARRY);
        else
            resetFlag(Flag::HALF_CARRY);
        // cast to unsigned because we need more than 8 bits to see if we're past 8 bits
        if ((unsigned int)(acc_pre) + (unsigned int)(val) > 0xFF)
            setFlag(Flag::CARRY);
        else
            resetFlag(Flag::CARRY);
        break;

    case ALUOp::DEC:
        setFlag(Flag::SUBTRACT);

        if ((acc_pre - val) == 0)
            setFlag(Flag::ZERO);
        else
            resetFlag(Flag::ZERO);

        if ((acc_pre & 0xF) < (val & 0xF))
            setFlag(Flag::HALF_CARRY);
        else
            resetFlag(Flag::HALF_CARRY);
        break;

    case ALUOp::SUB:
    case ALUOp::CMP:
        setFlag(Flag::SUBTRACT);

        if ((acc_pre - val) == 0)
            setFlag(Flag::ZERO);
        else
            resetFlag(Flag::ZERO);

        if ((acc_pre & 0xF) < (val & 0xF))
            setFlag(Flag::HALF_CARRY);
        else
            resetFlag(Flag::HALF_CARRY);

        if (acc_pre < val)
            setFlag(Flag::CARRY);
        else
            resetFlag(Flag::CARRY);
        break;

    case ALUOp::AND:
        if ((uint8_t)(acc_pre & val) == 0)
            setFlag(Flag::ZERO);
        resetFlag(Flag::SUBTRACT);
        setFlag(Flag::HALF_CARRY);
        resetFlag(Flag::CARRY);
        break;
        
    case ALUOp::OR:
        if ((uint8_t)(acc_pre | val) == 0)
            setFlag(Flag::ZERO);
        resetFlag(Flag::SUBTRACT);
        resetFlag(Flag::HALF_CARRY);
        resetFlag(Flag::CARRY);
        break;

    case ALUOp::XOR:
        if ((uint8_t)(acc_pre ^ val) == 0)
            setFlag(Flag::ZERO);
        resetFlag(Flag::SUBTRACT);
        resetFlag(Flag::HALF_CARRY);
        resetFlag(Flag::CARRY);
        break;
    }
    
}

uint16_t Processor::getRegisterPair(enum RegPair::RegisterPairs rp)
{
    uint16_t pair = (reg[2 * rp] << 8) | reg[2 * (rp + 1)];
    return pair;
}
void Processor::setRegisterPair(enum RegPair::RegisterPairs rp, uint16_t val)
{
    uint8_t high = (val & 0xFF00) >> 8;
    uint8_t low = val & 0x00FF;
    reg[2 * rp] = high;
    reg[2 * (rp + 1)] = low;
    if(rp == RegPair::AF)
    {
        reg[2 * (rp + 1)] &= 0xF0; // the low 4 bits of F register should always be zero
    }
}

void Processor::stackPush(uint16_t val)
{
    uint8_t lower, higher;
    lower = val & 0xFF;
    higher = (val & 0xFF00) >> 8;
    SP--;
    memory->setByte(SP, higher);
    SP--;
    memory->setByte(SP, lower);
}

uint16_t Processor::stackPop()
{
    uint8_t lower, higher;
    lower = memory->getByte(SP);
    SP++;
    higher = memory->getByte(SP);
    SP++;
    uint16_t popped = (higher << 8) | lower;
    return popped;
}

void Processor::CBExecute()
{
    uint8_t opcode = memory->getByte(PC++);
    uint8_t y = (opcode & 0b00111000) >> 3;
    uint8_t z = (opcode & 0b00000111) >> 0;
    uint8_t operand;
    uint8_t result;
    if ((opcode & 0xF == 0xE) || (opcode & 0xF == 0x6)) // value comes from memory
    {
        operand = memory->getByte(getRegisterPair(RegPair::HL));
    }
    else
    {
        operand = reg[z];
    }

    switch(opcode)
    {
    case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06: case 0x07: // RLC reg[z]
    {
        bool carry = operand & 0x80; // last bit of operand
        result = operand << 1;
        if (carry)
        {
            result |= 0x1; // first bit should be set
            setFlag(Flag::CARRY); // carry flag set
        }
        else
        {
            resetFlag(Flag::CARRY);
        }

        if(result == 0)
            setFlag(Flag::ZERO);
        else
            resetFlag(Flag::ZERO);
        resetFlag(Flag::HALF_CARRY);
        resetFlag(Flag::SUBTRACT);
        break;
    }
    case 0x08: case 0x09: case 0x0A: case 0x0B: case 0x0C: case 0x0D: case 0x0E: case 0x0F: // RRC reg[z]
    {
        bool carry = operand & 0x01; // first bit of operand
        result = operand >> 1;
        if (carry)
        {
            result |= 0x80; // last bit should be set
            setFlag(Flag::CARRY); // carry flag set
        }
        else
        {
            resetFlag(Flag::CARRY);
        }

        if(result == 0)
            setFlag(Flag::ZERO);
        else
            resetFlag(Flag::ZERO);
        resetFlag(Flag::HALF_CARRY);
        resetFlag(Flag::SUBTRACT);
        break;
    }
    case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17: // RL reg[z]
   {
        bool lbit = operand & 0x80; // last bit of operand
        uint8_t cflag = getFlag(Flag::CARRY);
        result = operand << 1;
        if (lbit)
            setFlag(Flag::CARRY);
        else
            setFlag(Flag::CARRY);

        if (cflag)
            result |= 0x1;
        
        if(result == 0)
            setFlag(Flag::ZERO);
        else
            resetFlag(Flag::ZERO);

        resetFlag(Flag::HALF_CARRY);
        resetFlag(Flag::SUBTRACT);
        break;
   }
    case 0x18: case 0x19: case 0x1A: case 0x1B: case 0x1C: case 0x1D: case 0x1E: case 0x1F: // RR reg[z]
    {
        bool rbit = operand & 0x80; // first bit of operand
        uint8_t cflag = getFlag(Flag::CARRY);
        result = operand >> 1;
        if (rbit)
            setFlag(Flag::CARRY);
        else
            setFlag(Flag::CARRY);

        if (cflag)
            result |= 0x80; // last bit should be set if cflag was set
        
        if(result == 0)
            setFlag(Flag::ZERO);
        else
            resetFlag(Flag::ZERO);

        resetFlag(Flag::HALF_CARRY);
        resetFlag(Flag::SUBTRACT);
        break;
    }
    case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27: // SLA reg[z]

    case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2C: case 0x2D: case 0x2E: case 0x2F: // SRA reg[z]

    case 0x30: case 0x31: case 0x32: case 0x33: case 0x34: case 0x35: case 0x36: case 0x37: // SWAP reg[z]

    case 0x38: case 0x39: case 0x3A: case 0x3B: case 0x3C: case 0x3D: case 0x3E: case 0x3F: // SRL reg[z]

    // BIT y reg[z]
    case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x46: case 0x47:
    case 0x48: case 0x49: case 0x4A: case 0x4B: case 0x4C: case 0x4D: case 0x4E: case 0x4F:
    case 0x50: case 0x51: case 0x52: case 0x53: case 0x54: case 0x55: case 0x56: case 0x57:
    case 0x58: case 0x59: case 0x5A: case 0x5B: case 0x5C: case 0x5D: case 0x5E: case 0x5F:
    case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x65: case 0x66: case 0x67:
    case 0x68: case 0x69: case 0x6A: case 0x6B: case 0x6C: case 0x6D: case 0x6E: case 0x6F:
    case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76: case 0x77:
    case 0x78: case 0x79: case 0x7A: case 0x7B: case 0x7C: case 0x7D: case 0x7E: case 0x7F:
        result = operand;
        setFlag(Flag::HALF_CARRY);
        resetFlag(Flag::SUBTRACT);
        if((operand && (0x1 << y)) >> y) // z bit set
        {
            resetFlag(Flag::ZERO);
        }
        else
        {
            setFlag(Flag::ZERO);
        }
        break;
    // RES y reg[z]
    case 0x80: case 0x81: case 0x82: case 0x83: case 0x84: case 0x85: case 0x86: case 0x87:
    case 0x88: case 0x89: case 0x8A: case 0x8B: case 0x8C: case 0x8D: case 0x8E: case 0x8F:
    case 0x90: case 0x91: case 0x92: case 0x93: case 0x94: case 0x95: case 0x96: case 0x97:
    case 0x98: case 0x99: case 0x9A: case 0x9B: case 0x9C: case 0x9D: case 0x9E: case 0x9F:
    case 0xA0: case 0xA1: case 0xA2: case 0xA3: case 0xA4: case 0xA5: case 0xA6: case 0xA7:
    case 0xA8: case 0xA9: case 0xAA: case 0xAB: case 0xAC: case 0xAD: case 0xAE: case 0xAF:
    case 0xB0: case 0xB1: case 0xB2: case 0xB3: case 0xB4: case 0xB5: case 0xB6: case 0xB7:
    case 0xB8: case 0xB9: case 0xBA: case 0xBB: case 0xBC: case 0xBD: case 0xBE: case 0xBF:
        result = operand;
        result |= 0x1 << y;
        break;
    // SET y reg[z]
    case 0xC0: case 0xC1: case 0xC2: case 0xC3: case 0xC4: case 0xC5: case 0xC6: case 0xC7:
    case 0xC8: case 0xC9: case 0xCA: case 0xCB: case 0xCC: case 0xCD: case 0xCE: case 0xCF:
    case 0xD0: case 0xD1: case 0xD2: case 0xD3: case 0xD4: case 0xD5: case 0xD6: case 0xD7:
    case 0xD8: case 0xD9: case 0xDA: case 0xDB: case 0xDC: case 0xDD: case 0xDE: case 0xDF:
    case 0xE0: case 0xE1: case 0xE2: case 0xE3: case 0xE4: case 0xE5: case 0xE6: case 0xE7:
    case 0xE8: case 0xE9: case 0xEA: case 0xEB: case 0xEC: case 0xED: case 0xEE: case 0xEF:
    case 0xF0: case 0xF1: case 0xF2: case 0xF3: case 0xF4: case 0xF5: case 0xF6: case 0xF7:
    case 0xF8: case 0xF9: case 0xFA: case 0xFB: case 0xFC: case 0xFD: case 0xFE: case 0xFF:
        result = operand;
        result &= ~(0x1 << y);
        break;
    }

    if ((opcode & 0xF == 0xE) || (opcode & 0xF == 0x6)) // result goes to memory
    {
        memory->setByte(getRegisterPair(RegPair::HL), result);
    }
    else
    {
        reg[z] = result;
    }
}