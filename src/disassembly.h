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

    void print(FILE *fp);
};

#endif