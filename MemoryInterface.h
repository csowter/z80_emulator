#ifndef memoryinterface_h
#define memoryinterface_h

#include <cstdint>

class MemoryInterface
{
public:
  virtual ~MemoryInterface(void){}
  virtual uint8_t ReadByte(uint16_t address) const = 0;
  virtual void WriteByte(uint16_t address, uint8_t data) = 0;
};

#endif //memoryinterface_h