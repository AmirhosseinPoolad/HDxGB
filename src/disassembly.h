#ifndef DISASSEMBLY_H
#define DISASSEMBLY_H

#include <string>
#include <cstdio>
#include "processor.h"

class DisassemblyObject
{
private:
    std::string str;
    
public:
    DisassemblyObject(const char *instrName);
    DisassemblyObject(uint8_t opcode);

    void addArg(const char *name, bool isMem = false, int postpre = 0);
    void addArg(RegPair::RegisterPairs rp, bool isMem = false, int postpre = 0);
    void addArg(Reg::Registers reg, bool isMem = false, int postpre = 0);
    void addArg(uint8_t u8, bool isMem = false, int postpre = 0);
    void addArg(uint16_t u16, bool isMem = false, int postpre = 0);
    void addArg(int8_t s8, bool isMem = false, int postpre = 0);
    void addArg(int16_t s16, bool isMem = false, int postpre = 0);

    void print(FILE *fp);
};

#endif