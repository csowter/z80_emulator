#include "Memory.h"
#include <cstring>
#include <cassert>

Memory::Memory(uint16_t size)
  : memory(new uint8_t[size]), size(size)
{
  memset(memory, 0x00, size);
  for(int i = 0; i < size; i++)
	memory[i] = i & 0xFF;
}

Memory::~Memory(void)
{
  delete[] memory;
}

uint8_t Memory::ReadByte(uint16_t address)
{
  assert(address < size);
  return memory[address];
}

void Memory::WriteByte(uint16_t address, uint8_t data)
{
  assert(address < size);
  memory[address] = data;
}
