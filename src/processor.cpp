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

    default:
        fprintf(stderr, "Unknown Instruction. Opcode: %x\n", opcode);
        break;
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
}