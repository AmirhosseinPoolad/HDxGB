#include "disassembly.h"
#include <string>
#include <cstdio>

const static char regString[8][2] = {"B", "C", "D" , "E", "H", "L", "F", "A"};
const static char regPairString[8][3] = {"BC", "DE", "HL" , "AF"};
const static char opcodeString[256][10] =
{
//  0    1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
    "NOP", "LD", "LD", "INC", "INC", "DEC", "LD", "RLCA", "LD", "ADD", "LD", "DEC", "INC", "DEC", "LD", "RRCA",
    "STOP", "LD", "LD", "INC", "INC", "DEC", "LD", "RLA", "JR", "ADD", "LD", "DEC", "INC", "DEC", "LD", "RRA",
    "JR", "LD", "LD", "INC", "INC", "DEC", "LD", "DAA", "JR", "ADD", "LD", "DEC", "INC", "DEC", "LD", "CPL",
    "JR", "LD", "LD", "INC", "INC", "DEC", "LD", "SCF", "JR", "ADD", "LD", "DEC", "INC", "DEC", "LD", "CCF",
    "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
    "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
    "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
    "LD", "LD", "LD", "LD", "LD", "LD", "HALT", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
    "ADD", "ADD", "ADD", "ADD", "ADD", "ADD", "ADD", "ADD", "ADC", "ADC", "ADC", "ADC", "ADC", "ADC", "ADC", "ADC",
    "SUB", "SUB", "SUB", "SUB", "SUB", "SUB", "SUB", "SUB", "SBC", "SBC", "SBC", "SBC", "SBC", "SBC", "SBC", "SBC",
    "AND", "AND", "AND", "AND", "AND", "AND", "AND", "AND", "XOR", "XOR", "XOR", "XOR", "XOR", "XOR", "XOR", "XOR",
    "OR", "OR", "OR", "OR", "OR", "OR", "OR", "OR", "CP", "CP", "CP", "CP", "CP", "CP", "CP", "CP",
    "RET", "POP", "JP", "JP", "CALL", "PUSH", "ADD", "RST", "RET", "RET", "JP", "CB", "CALL", "CALL", "ADC", "RST",
    "RET", "POP", "JP", "ERR", "CALL", "PUSH", "SUB", "RST", "RET", "RETI", "JP", "ERR", "CALL", "ERR", "SBC", "RST",
    "LD", "POP", "LD", "ERR", "ERR", "PUSH", "AND", "RST", "ADD", "JP", "LD", "ERR", "ERR", "ERR", "XOR", "RST",
    "LD", "POP", "LD", "DI", "ERR", "PUSH", "OR", "RST", "LD", "LD", "LD", "EI", "ERR", "ERR", "CP", "RST"
};

DisassemblyObject::DisassemblyObject(const char *instrName)
{
    this->str = std::string(instrName);
}

DisassemblyObject::DisassemblyObject(uint8_t opcode)
{
    this->str = std::string(opcodeString[opcode]);
    //std::cout << this->str << std::endl;
}

// TODO: maybe change postpre into an enum
void DisassemblyObject::addArg(const char *name, bool isMem, int postpre)
{
    std::string argStr(name);
    //std::cout << argStr << std::endl;
    switch (postpre)
    {
    case 1:
        argStr = argStr + "+";
        break;
    case -1:
        argStr = argStr + "-";
        break;
    default:
        break;
    }
    //std::cout << argStr << std::endl;
    if(isMem)
    {
        argStr = "(" + argStr + ")";
    }
    //std::cout << argStr << std::endl;
    this->str = this->str + argStr;
}

void DisassemblyObject::addArg(RegPair::RegisterPairs rp, bool isMem, int postpre)
{
    this->addArg(regPairString[rp], isMem, postpre);
}

void DisassemblyObject::addArg(Reg::Registers reg, bool isMem, int postpre)
{
    this->addArg(regString[reg], isMem, postpre);
}

void DisassemblyObject::print(FILE *fp)
{
    fprintf(fp, "%s\n", this->str.c_str());
}
