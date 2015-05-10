#include "Memory.h"
#include "Z80.h"

#include <iostream>

int main(int argc, char* argv[])
{
  Memory memory(0xFF);
  Z80 z80(&memory);

for(int i = 0; i < 500; i++)
	z80.Execute();
	return 0; 
}
 
