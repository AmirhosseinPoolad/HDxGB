#ifndef MEMORY_H
#define MEMORY_H

#include <cstdint>

// check out the pandocs for more info:
// https://gbdev.io/pandocs/Memory_Map.html
class Memory {
private:
  /*
   Memory and mappings:

  // 0x0000 to 0x3FFF --- 16 KiB, mapped to cartridge ROM
  uint8_t *cartridgeRomBank_00;
  // 0x4000 to 0x7FFF --- 16 KiB, mapped to cartridge ROM. The bank number can
  be changed by the Memory Bank Controller uint8_t *cartridgeRomBank_nn;
  // 0x8000 to 0x9FFF --- 8 KiB, video RAM
  uint8_t *VRam;
  // 0xA000 to 0xBFFF --- 8 KiB, cartridge RAM
  uint8_t *ExternalRam;
  // 0xC000 to 0xCFFF --- 4 KiB, work RAM bank 0
  uint8_t *WorkRamBank_0;
  // 0xD000 to 0xDFFF --- 4 Kib, work RAM banks 1 to 7 (2 to 7 only in GBC)
  uint8_t *WorkRamBank_n;
  // 0xE000 to 0xFDFF --- ECHO RAM, mirror of C000 to DDFF
  uint8_t *EchoRam;
  // 0xFE00 to 0xFE9F --- Sprite attribute table
  uint8_t *SpriteAttributeTable;
  // 0xFEA0 to 0xFEFF --- Unusable
  uint8_t *unusableMemory;
  // 0xFF00 to 0xFF7F --- I/O Registers
  uint8_t *IORegisters;
  // 0xFF90 to 0xFFFE --- High RAM
  uint8_t *HighRam;
  // 0xFFFF to 0xFFFF --- Interrupt Enable Register
  uint8_t *InterruptEnableRegister;
  */
  uint8_t *memory;

public:
  Memory();
  ~Memory();
  uint8_t getByte(uint16_t address);
  void setByte(uint16_t address, uint8_t value);
};
#endif