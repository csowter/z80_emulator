#include "Memory.h"
#include <cstring>
#include <cassert>

Memory::Memory(uint32_t size)
  : memory(new uint8_t[size]), size(size)
{
  memset(memory, 0x00, size);
  for(uint32_t i = 0; i < size; i++)
	  memory[i] = i & 0xFF;

  memory[0] = 0x3e;
  memory[1] = 0x88;
  memory[2] = 0x07;
}

Memory::~Memory(void)
{
  delete[] memory;
}

uint8_t Memory::ReadByte(uint16_t address) const
{
  assert(address < size);
  return memory[address];
}

void Memory::WriteByte(uint16_t address, uint8_t data)
{
  assert(address < size);
  memory[address] = data;
}
