#ifndef registers_h
#define registers_h

#include <cstdint>

#define CARRY_BIT 0x01
#define SUBTRACT_BIT 0x02
#define PARITY_OVERFLOW_BIT 0x04
#define HALF_CARRY_BIT 0x10
#define ZERO_BIT 0x40
#define SIGN_BIT 0x80

struct AF
{
  union
  {
    uint16_t af;
    struct
    {
#ifdef LITTLE_ENDIAN
      uint8_t a;
      uint8_t f;
#else
      uint8_t f;
      uint8_t a;
#endif
    };
  };
};

struct BC
{
  union
  {
    uint16_t bc;
    struct
    {
#ifdef LITTLE_ENDIAN
      uint8_t b;
      uint8_t c;
#else
      uint8_t c;
      uint8_t b;
#endif
    };
  };
};

struct DE
{
  union
  {
    uint16_t de;
    struct
    {
#ifdef LITTLE_ENDIAN
      uint8_t d;
      uint8_t e;
#else
      uint8_t e;
      uint8_t d;
#endif
    };
  };  
};

struct HL
{
  union
  {
    struct
    {
#ifdef LITTLE_ENDIAN
      uint8_t h;
      uint8_t l;
#else
      uint8_t l;
      uint8_t h;
#endif
    };
  };
};

struct MainRegisters
{
  AF af;
  BC bc;
  DE de;
  HL hl;  
};

struct IndexRegisters
{
  uint16_t ix;
  uint16_t iy;
  uint16_t sp;
};

struct OtherRegisters
{
  uint8_t i;
  uint8_t r;
};

#endif //registers_h