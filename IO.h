#ifndef io_h
#define io_h

#include "IOInterface.h"
#include <cstdint>

class IO : public IOInterface
{
public:
  virtual uint8_t ReadByte(uint16_t address);
  virtual void WriteByte(uint16_t address, uint8_t data);
};

#endif //io_h