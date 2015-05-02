#ifndef z80_h
#define z80_h

#include <cstdint>
#include "Registers.h"

class MemoryInterface;

class Z80
{
private:
  MainRegisters mainRegisters;
  MainRegisters alternateRegisters;
  IndexRegisters indexRegisters;
  OtherRegisters otherRegisters;
  uint16_t PC;
  MemoryInterface *memory;
  MainRegisters *currentRegisters;

public:
  Z80(MemoryInterface *memoryInterface);
  virtual ~Z80(void);
  void Execute(void);
};



#endif //z80_h