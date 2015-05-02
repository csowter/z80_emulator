#include "Z80.h"
#include "MemoryInterface.h"

Z80::Z80(MemoryInterface *memoryInterface)
  : mainRegisters(), alternateRegisters(), indexRegisters(), otherRegisters(), PC(0), memory(memoryInterface), currentRegisters(&mainRegisters)
{
}

Z80::~Z80(void)
{
}

void Z80::Execute(void)
{
  uint8_t instruction = memory->ReadByte(PC++);
}