#include "Memory.h"
#include "Z80.h"
#include "IO.h"

#include <iostream>

int main(int argc, char* argv[])
{
  Memory memory(0x10000);
  IO io;

  Z80 z80(&memory, &io);

	for(int i = 0; i <= 0xFFFF; i++)
		z80.Execute();

	return 0; 
}
 
