#ifndef iointerface_h
#define iointerface_h

#include <cstdint>

class IOInterface
{
public:
  ~IOInterface(void){};
  virtual uint8_t ReadByte(uint16_t address) = 0;
  virtual void WriteByte(uint16_t address, uint8_t data) = 0;
};

#endif //iointerface