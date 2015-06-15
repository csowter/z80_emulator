#ifndef memory_h
#define memory_h

#include <cstdint>
#include "MemoryInterface.h"

class Memory : public MemoryInterface
{
private:
  uint8_t *memory;
  uint16_t size;
public:
  Memory(uint16_t size);
  virtual ~Memory(void);
  virtual uint8_t ReadByte(uint16_t address) const;
  virtual void WriteByte(uint16_t address, uint8_t data);
};

#endif //memory_h