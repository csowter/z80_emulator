#include "Z80.h"
#include "MemoryInterface.h"
#include "OpCodes.h"
#include <iostream>

Z80::Z80(MemoryInterface *memoryInterface)
  : mainRegisters(), alternateRegisters(), indexRegisters(), otherRegisters(), PC(0), memory(memoryInterface), currentRegisters(&mainRegisters), Op(new fptr[OpCodes::NUMBER_OF_OPCODES])
{
  Op[OpCodes::NOP] = &Z80::NOP;               
  Op[OpCodes::LD_BC_word] = &Z80::LD_BC_word;        
  Op[OpCodes::LD_iBC_A] = &Z80::LD_iBC_A;          
  Op[OpCodes::INC_BC] = &Z80::INC_BC;            
  Op[OpCodes::INC_B] = &Z80::INC_B;
  Op[OpCodes::DEC_B] = &Z80::DEC_B;             
  Op[OpCodes::LD_B_byte] = &Z80::LD_B_byte;    
  Op[OpCodes::RLCA] = &Z80::RLCA;
  Op[OpCodes::EX_AF_AF] = &Z80::EX_AF_AF;
  Op[OpCodes::ADD_HL_BC] = &Z80::ADD_HL_BC;
  Op[OpCodes::LD_A_iBC] = &Z80::LD_A_iBC;
  Op[OpCodes::DEC_BC] = &Z80::DEC_BC;
  Op[OpCodes::INC_C] = &Z80::INC_C;
  Op[OpCodes::DEC_C] = &Z80::DEC_C;
  Op[OpCodes::LD_C_byte] = &Z80::LD_C_byte;
  Op[OpCodes::RRCA] = &Z80::RRCA;
  Op[OpCodes::DJNZ] = &Z80::DJNZ;
  Op[OpCodes::LD_DE_word] = &Z80::LD_DE_word;
  Op[OpCodes::LD_iDE_A] = &Z80::LD_iDE_A;
  Op[OpCodes::INC_DE] = &Z80::INC_DE;
  Op[OpCodes::INC_D] = &Z80::INC_D;
  Op[OpCodes::DEC_D] = &Z80::DEC_D;
  Op[OpCodes::LD_D_byte] = &Z80::LD_D_byte;
  Op[OpCodes::RLA] = &Z80::RLA;
  Op[OpCodes::JR] = &Z80::JR;
  Op[OpCodes::ADD_HL_DE] = &Z80::ADD_HL_DE;
  Op[OpCodes::LD_A_iDE] = &Z80::LD_A_iDE;
  Op[OpCodes::DEC_DE] = &Z80::DEC_DE;
  Op[OpCodes::INC_E] = &Z80::INC_E;
  Op[OpCodes::DEC_E] = &Z80::DEC_E;
  Op[OpCodes::LD_E_byte] = &Z80::LD_E_byte;
  Op[OpCodes::RRA] = &Z80::RRA;
  Op[OpCodes::JR_NZ] = &Z80::JR_NZ;
  Op[OpCodes::LD_HL_word] = &Z80::LD_HL_word;
  Op[OpCodes::LD_iNN_HL] = &Z80::LD_iNN_HL;
  Op[OpCodes::INC_HL] = &Z80::INC_HL;
  Op[OpCodes::INC_H] = &Z80::INC_H;
  Op[OpCodes::DEC_H] = &Z80::DEC_H;
  Op[OpCodes::LD_H_byte] = &Z80::LD_H_byte;
  Op[OpCodes::DAA] = &Z80::DAA;
  Op[OpCodes::JR_Z] = &Z80::JR_Z;
  Op[OpCodes::ADD_HL_HL] = &Z80::ADD_HL_HL;
  Op[OpCodes::LD_HL_iNN] = &Z80::LD_HL_iNN;
  Op[OpCodes::DEC_HL] = &Z80::DEC_HL;
  Op[OpCodes::INC_L] = &Z80::INC_L;
  Op[OpCodes::DEC_L] = &Z80::DEC_L;
  Op[OpCodes::LD_L_byte] = &Z80::LD_L_byte;
  Op[OpCodes::CPL] = &Z80::CPL;
  Op[OpCodes::JR_NC] = &Z80::JR_NC;
  Op[OpCodes::LD_SP_word] = &Z80::LD_SP_word;
  Op[OpCodes::LD_iNN_A] = &Z80::LD_iNN_A;
  Op[OpCodes::INC_SP] = &Z80::INC_SP;
  Op[OpCodes::INC_iHL] = &Z80::INC_iHL;
  Op[OpCodes::DEC_iHL] = &Z80::DEC_iHL;
  Op[OpCodes::LD_iHL_byte] = &Z80::LD_iHL_byte;
  Op[OpCodes::SCF] = &Z80::SCF;
  Op[OpCodes::JR_C] = &Z80::JR_C;
  Op[OpCodes::ADD_HL_SP] = &Z80::ADD_HL_SP;
  Op[OpCodes::LD_A_iNN] = &Z80::LD_A_iNN;
  Op[OpCodes::DEC_SP] = &Z80::DEC_SP;
  Op[OpCodes::INC_A] = &Z80::INC_A;
  Op[OpCodes::DEC_A] = &Z80::DEC_A;
  Op[OpCodes::LD_A_byte] = &Z80::LD_A_byte;
  Op[OpCodes::CCF] = &Z80::CCF;
  Op[OpCodes::LD_B_B] = &Z80::LD_B_B;
  Op[OpCodes::LD_B_C] = &Z80::LD_B_C;
  Op[OpCodes::LD_B_D] = &Z80::LD_B_D;
  Op[OpCodes::LD_B_E] = &Z80::LD_B_E;
  Op[OpCodes::LD_B_H] = &Z80::LD_B_H;
  Op[OpCodes::LD_B_L] = &Z80::LD_B_L;
  Op[OpCodes::LD_B_iHL] = &Z80::LD_B_iHL;
  Op[OpCodes::LD_B_A] = &Z80::LD_B_A;
  Op[OpCodes::LD_C_B] = &Z80::LD_C_B;
  Op[OpCodes::LD_C_C] = &Z80::LD_C_C;
  Op[OpCodes::LD_C_D] = &Z80::LD_C_D;
  Op[OpCodes::LD_C_E] = &Z80::LD_C_E;
  Op[OpCodes::LD_C_H] = &Z80::LD_C_H;
  Op[OpCodes::LD_C_L] = &Z80::LD_C_L;
  Op[OpCodes::LD_C_iHL] = &Z80::LD_C_iHL;
  Op[OpCodes::LD_C_A] = &Z80::LD_C_A;
  Op[OpCodes::LD_D_B] = &Z80::LD_D_B;
  Op[OpCodes::LD_D_C] = &Z80::LD_D_C;
  Op[OpCodes::LD_D_D] = &Z80::LD_D_D;
  Op[OpCodes::LD_D_E] = &Z80::LD_D_E;
  Op[OpCodes::LD_D_H] = &Z80::LD_D_H;
  Op[OpCodes::LD_D_L] = &Z80::LD_D_L;
  Op[OpCodes::LD_D_iHL] = &Z80::LD_D_iHL;
  Op[OpCodes::LD_D_A] = &Z80::LD_D_A;
  Op[OpCodes::LD_E_B] = &Z80::LD_E_B;
  Op[OpCodes::LD_E_C] = &Z80::LD_E_C;
  Op[OpCodes::LD_E_D] = &Z80::LD_E_D;
  Op[OpCodes::LD_E_E] = &Z80::LD_E_E;
  Op[OpCodes::LD_E_H] = &Z80::LD_E_H;
  Op[OpCodes::LD_E_L] = &Z80::LD_E_L;
  Op[OpCodes::LD_E_iHL] = &Z80::LD_E_iHL;
  Op[OpCodes::LD_E_A] = &Z80::LD_E_A;
  Op[OpCodes::LD_H_B] = &Z80::LD_H_B;
  Op[OpCodes::LD_H_C] = &Z80::LD_H_C;
  Op[OpCodes::LD_H_D] = &Z80::LD_H_D;
  Op[OpCodes::LD_H_E] = &Z80::LD_H_E;
  Op[OpCodes::LD_H_H] = &Z80::LD_H_H;
  Op[OpCodes::LD_H_L] = &Z80::LD_H_L;
  Op[OpCodes::LD_H_iHL] = &Z80::LD_H_iHL;
  Op[OpCodes::LD_H_A] = &Z80::LD_H_A;
  Op[OpCodes::LD_L_B] = &Z80::LD_L_B;
  Op[OpCodes::LD_L_C] = &Z80::LD_L_C;
  Op[OpCodes::LD_L_D] = &Z80::LD_L_D;
  Op[OpCodes::LD_L_E] = &Z80::LD_L_E;
  Op[OpCodes::LD_L_H] = &Z80::LD_L_H;
  Op[OpCodes::LD_L_L] = &Z80::LD_L_L;
  Op[OpCodes::LD_L_iHL] = &Z80::LD_L_iHL;
  Op[OpCodes::LD_L_A] = &Z80::LD_L_A;
  Op[OpCodes::LD_iHL_B] = &Z80::LD_iHL_B;
  Op[OpCodes::LD_iHL_C] = &Z80::LD_iHL_C;
  Op[OpCodes::LD_iHL_D] = &Z80::LD_iHL_D;
  Op[OpCodes::LD_iHL_E] = &Z80::LD_iHL_E;
  Op[OpCodes::LD_iHL_H] = &Z80::LD_iHL_H;
  Op[OpCodes::LD_iHL_L] = &Z80::LD_iHL_L;
  Op[OpCodes::HALT] = &Z80::HALT;
  Op[OpCodes::LD_iHL_A] = &Z80::LD_iHL_A;
  Op[OpCodes::LD_A_B] = &Z80::LD_A_B;
  Op[OpCodes::LD_A_C] = &Z80::LD_A_C;
  Op[OpCodes::LD_A_D] = &Z80::LD_A_D;
  Op[OpCodes::LD_A_E] = &Z80::LD_A_E;
  Op[OpCodes::LD_A_H] = &Z80::LD_A_H;
  Op[OpCodes::LD_A_L] = &Z80::LD_A_L;
  Op[OpCodes::LD_A_iHL] = &Z80::LD_A_iHL;
  Op[OpCodes::LD_A_A] = &Z80::LD_A_A;
  Op[OpCodes::ADD_A_B] = &Z80::ADD_A_B;
  Op[OpCodes::ADD_A_C] = &Z80::ADD_A_C;
  Op[OpCodes::ADD_A_D] = &Z80::ADD_A_D;
  Op[OpCodes::ADD_A_E] = &Z80::ADD_A_E;
  Op[OpCodes::ADD_A_H] = &Z80::ADD_A_H;
  Op[OpCodes::ADD_A_L] = &Z80::ADD_A_L;
  Op[OpCodes::ADD_A_iHL] = &Z80::ADD_A_iHL;
  Op[OpCodes::ADD_A_A] = &Z80::ADD_A_A;
  Op[OpCodes::ADC_A_B] = &Z80::ADC_A_B;
  Op[OpCodes::ADC_A_C] = &Z80::ADC_A_C;
  Op[OpCodes::ADC_A_D] = &Z80::ADC_A_D;
  Op[OpCodes::ADC_A_E] = &Z80::ADC_A_E;
  Op[OpCodes::ADC_A_H] = &Z80::ADC_A_H;
  Op[OpCodes::ADC_A_L] = &Z80::ADC_A_L;
  Op[OpCodes::ADC_A_iHL] = &Z80::ADC_A_iHL;
  Op[OpCodes::ADC_A_A] = &Z80::ADC_A_A;
  Op[OpCodes::SUB_A_B] = &Z80::SUB_A_B;
  Op[OpCodes::SUB_A_C] = &Z80::SUB_A_C;
  Op[OpCodes::SUB_A_D] = &Z80::SUB_A_D;
  Op[OpCodes::SUB_A_E] = &Z80::SUB_A_E;
  Op[OpCodes::SUB_A_H] = &Z80::SUB_A_H;
  Op[OpCodes::SUB_A_L] = &Z80::SUB_A_L;
  Op[OpCodes::SUB_A_iHL] = &Z80::SUB_A_iHL;
  Op[OpCodes::SUB_A_A] = &Z80::SUB_A_A;
  Op[OpCodes::SBC_A_B] = &Z80::SBC_A_B;
  Op[OpCodes::SBC_A_C] = &Z80::SBC_A_C;
  Op[OpCodes::SBC_A_D] = &Z80::SBC_A_D;
  Op[OpCodes::SBC_A_E] = &Z80::SBC_A_E;
  Op[OpCodes::SBC_A_H] = &Z80::SBC_A_H;
  Op[OpCodes::SBC_A_L] = &Z80::SBC_A_L;
  Op[OpCodes::SBC_A_iHL] = &Z80::SBC_A_iHL;
  Op[OpCodes::SBC_A_A] = &Z80::SBC_A_A;
  Op[OpCodes::AND_B] = &Z80::AND_B;
  Op[OpCodes::AND_C] = &Z80::AND_C;
  Op[OpCodes::AND_D] = &Z80::AND_D;
  Op[OpCodes::AND_E] = &Z80::AND_E;
  Op[OpCodes::AND_H] = &Z80::AND_H;
  Op[OpCodes::AND_L] = &Z80::AND_L;
  Op[OpCodes::AND_iHL] = &Z80::AND_iHL;
  Op[OpCodes::AND_A] = &Z80::AND_A;
  Op[OpCodes::XOR_B] = &Z80::XOR_B;
  Op[OpCodes::XOR_C] = &Z80::XOR_C;
  Op[OpCodes::XOR_D] = &Z80::XOR_D;
  Op[OpCodes::XOR_E] = &Z80::XOR_E;
  Op[OpCodes::XOR_H] = &Z80::XOR_H;
  Op[OpCodes::XOR_L] = &Z80::XOR_L;
  Op[OpCodes::XOR_iHL] = &Z80::XOR_iHL;
  Op[OpCodes::XOR_A] = &Z80::XOR_A;
  Op[OpCodes::OR_B] = &Z80::OR_B;
  Op[OpCodes::OR_C] = &Z80::OR_C;
  Op[OpCodes::OR_D] = &Z80::OR_D;
  Op[OpCodes::OR_E] = &Z80::OR_E;
  Op[OpCodes::OR_H] = &Z80::OR_H;
  Op[OpCodes::OR_L] = &Z80::OR_L;
  Op[OpCodes::OR_iHL] = &Z80::OR_iHL;
  Op[OpCodes::OR_A] = &Z80::OR_A;
  Op[OpCodes::CP_B] = &Z80::CP_B;
  Op[OpCodes::CP_C] = &Z80::CP_C;
  Op[OpCodes::CP_D] = &Z80::CP_D;
  Op[OpCodes::CP_E] = &Z80::CP_E;
  Op[OpCodes::CP_H] = &Z80::CP_H;
  Op[OpCodes::CP_L] = &Z80::CP_L;
  Op[OpCodes::CP_iHL] = &Z80::CP_iHL;
  Op[OpCodes::CP_A] = &Z80::CP_A;
  Op[OpCodes::RET_NZ] = &Z80::RET_NZ;
  Op[OpCodes::POP_BC] = &Z80::POP_BC;
  Op[OpCodes::JP_NZ_word] = &Z80::JP_NZ_word;
  Op[OpCodes::JP_word] = &Z80::JP_word;
  Op[OpCodes::CALL_NZ_word] = &Z80::CALL_NZ_word;
  Op[OpCodes::PUSH_BC] = &Z80::PUSH_BC;
  Op[OpCodes::ADD_A_byte] = &Z80::ADD_A_byte;
  Op[OpCodes::RST_00] = &Z80::RST_00;
  Op[OpCodes::RET_Z] = &Z80::RET_Z;
  Op[OpCodes::RET] = &Z80::RET;
  Op[OpCodes::JP_Z_word] = &Z80::JP_Z_word;
  Op[OpCodes::CB] = &Z80::CB;
  Op[OpCodes::CALL_Z_word] = &Z80::CALL_Z_word;
  Op[OpCodes::CALL_word] = &Z80::CALL_word;
  Op[OpCodes::ADC_A_byte] = &Z80::ADC_A_byte;
  Op[OpCodes::RST_08] = &Z80::RST_08;
  Op[OpCodes::RET_NC] = &Z80::RET_NC;
  Op[OpCodes::POP_DE] = &Z80::POP_DE;
  Op[OpCodes::JP_NC_word] = &Z80::JP_NC_word;
  Op[OpCodes::OUT_iNN_A] = &Z80::OUT_iNN_A;
  Op[OpCodes::CALL_NC_word] = &Z80::CALL_NC_word;
  Op[OpCodes::PUSH_DE] = &Z80::PUSH_DE;
  Op[OpCodes::SUB_A_byte] = &Z80::SUB_A_byte;
  Op[OpCodes::RST_10] = &Z80::RST_10;
  Op[OpCodes::RET_C] = &Z80::RET_C;
  Op[OpCodes::EXX] = &Z80::EXX;
  Op[OpCodes::JP_C_word] = &Z80::JP_C_word;
  Op[OpCodes::IN_A_iNN] = &Z80::IN_A_iNN;
  Op[OpCodes::CALL_C_word] = &Z80::CALL_C_word;
  Op[OpCodes::DD] = &Z80::DD;
  Op[OpCodes::SBC_A_byte] = &Z80::SBC_A_byte;
  Op[OpCodes::RST_18] = &Z80::RST_18;
  Op[OpCodes::RET_PO] = &Z80::RET_PO;
  Op[OpCodes::POP_HL] = &Z80::POP_HL;
  Op[OpCodes::JP_PO_word] = &Z80::JP_PO_word;
  Op[OpCodes::EX_iSP_HL] = &Z80::EX_iSP_HL;
  Op[OpCodes::CALL_PO_word] = &Z80::CALL_PO_word;
  Op[OpCodes::PUSH_HL] = &Z80::PUSH_HL;
  Op[OpCodes::AND_byte] = &Z80::AND_byte;
  Op[OpCodes::RST_20] = &Z80::RST_20;
  Op[OpCodes::RET_PE] = &Z80::RET_PE;
  Op[OpCodes::JP_iHL] = &Z80::JP_iHL;
  Op[OpCodes::JP_PE_word] = &Z80::JP_PE_word;
  Op[OpCodes::EX_DE_HL] = &Z80::EX_DE_HL;
  Op[OpCodes::CALL_PE_word] = &Z80::CALL_PE_word;
  Op[OpCodes::ED] = &Z80::ED;
  Op[OpCodes::XOR_byte] = &Z80::XOR_byte;
  Op[OpCodes::RST_28] = &Z80::RST_28;
  Op[OpCodes::RET_P] = &Z80::RET_P;
  Op[OpCodes::POP_AF] = &Z80::POP_AF;
  Op[OpCodes::JP_P_word] = &Z80::JP_P_word;
  Op[OpCodes::DI] = &Z80::DI;
  Op[OpCodes::CALL_P_word] = &Z80::CALL_P_word;
  Op[OpCodes::PUSH_AF] = &Z80::PUSH_AF;
  Op[OpCodes::OR_byte] = &Z80::OR_byte;
  Op[OpCodes::RST_30] = &Z80::RST_30;
  Op[OpCodes::RET_M] = &Z80::RET_M;
  Op[OpCodes::LD_SP_HL] = &Z80::LD_SP_HL;
  Op[OpCodes::JP_M_word] = &Z80::JP_M_word;
  Op[OpCodes::EI] = &Z80::EI;
  Op[OpCodes::CALL_M_word] = &Z80::CALL_M_word;
  Op[OpCodes::FD] = &Z80::FD;
  Op[OpCodes::CP_byte] = &Z80::CP_byte;
  Op[OpCodes::RST_38] = &Z80::RST_38; 
}

Z80::~Z80(void)
{
  delete[] Op;
}

void Z80::Execute(void)
{
  uint8_t instruction = memory->ReadByte(PC++);
  (this->*Op[instruction])();
}

void Z80::NOP(void){}               
void Z80::LD_BC_word(void){}        
void Z80::LD_iBC_A(void){}          
void Z80::INC_BC(void){}            
void Z80::INC_B(void){}
void Z80::DEC_B(void){}             
void Z80::LD_B_byte(void){}    
void Z80::RLCA(void){}
void Z80::EX_AF_AF(void){uint16_t af = currentRegisters->af.af; currentRegisters->af.af = alternateRegisters.af.af; alternateRegisters.af.af = af;}
void Z80::ADD_HL_BC(void){}
void Z80::LD_A_iBC(void){}
void Z80::DEC_BC(void){}
void Z80::INC_C(void){}
void Z80::DEC_C(void){}
void Z80::LD_C_byte(void){}
void Z80::RRCA(void){}
void Z80::DJNZ(void){}
void Z80::LD_DE_word(void){}
void Z80::LD_iDE_A(void){}
void Z80::INC_DE(void){}
void Z80::INC_D(void){}
void Z80::DEC_D(void){}
void Z80::LD_D_byte(void){}
void Z80::RLA(void){}
void Z80::JR(void){}
void Z80::ADD_HL_DE(void){}
void Z80::LD_A_iDE(void){}
void Z80::DEC_DE(void){}
void Z80::INC_E(void){}
void Z80::DEC_E(void){}
void Z80::LD_E_byte(void){}
void Z80::RRA(void){}
void Z80::JR_NZ(void){}
void Z80::LD_HL_word(void){}
void Z80::LD_iNN_HL(void){}
void Z80::INC_HL(void){}
void Z80::INC_H(void){}
void Z80::DEC_H(void){}
void Z80::LD_H_byte(void){}
void Z80::DAA(void){}
void Z80::JR_Z(void){}
void Z80::ADD_HL_HL(void){}
void Z80::LD_HL_iNN(void){}
void Z80::DEC_HL(void){}
void Z80::INC_L(void){}
void Z80::DEC_L(void){}
void Z80::LD_L_byte(void){}
void Z80::CPL(void){}
void Z80::JR_NC(void){}
void Z80::LD_SP_word(void){}
void Z80::LD_iNN_A(void){}
void Z80::INC_SP(void){}
void Z80::INC_iHL(void){}
void Z80::DEC_iHL(void){}
void Z80::LD_iHL_byte(void){}
void Z80::SCF(void){}
void Z80::JR_C(void){}
void Z80::ADD_HL_SP(void){}
void Z80::LD_A_iNN(void){}
void Z80::DEC_SP(void){}
void Z80::INC_A(void){}
void Z80::DEC_A(void){}
void Z80::LD_A_byte(void){}
void Z80::CCF(void){}
void Z80::LD_B_B(void){currentRegisters->bc.b = currentRegisters->bc.b;}
void Z80::LD_B_C(void){currentRegisters->bc.b = currentRegisters->bc.c;}
void Z80::LD_B_D(void){currentRegisters->bc.b = currentRegisters->de.d;}
void Z80::LD_B_E(void){currentRegisters->bc.b = currentRegisters->de.e;}
void Z80::LD_B_H(void){currentRegisters->bc.b = currentRegisters->hl.h;}
void Z80::LD_B_L(void){currentRegisters->bc.b = currentRegisters->hl.l;}
void Z80::LD_B_iHL(void){currentRegisters->bc.b = memory->ReadByte(currentRegisters->hl.hl);}
void Z80::LD_B_A(void){currentRegisters->bc.b = currentRegisters->af.a;}
void Z80::LD_C_B(void){currentRegisters->bc.c = currentRegisters->bc.b;}
void Z80::LD_C_C(void){currentRegisters->bc.c = currentRegisters->bc.c;}
void Z80::LD_C_D(void){currentRegisters->bc.c = currentRegisters->de.d;}
void Z80::LD_C_E(void){currentRegisters->bc.c = currentRegisters->de.e;}
void Z80::LD_C_H(void){currentRegisters->bc.c = currentRegisters->hl.h;}
void Z80::LD_C_L(void){currentRegisters->bc.c = currentRegisters->hl.l;}
void Z80::LD_C_iHL(void){currentRegisters->bc.c = memory->ReadByte(currentRegisters->hl.hl);}
void Z80::LD_C_A(void){currentRegisters->bc.c = currentRegisters->af.a;}
void Z80::LD_D_B(void){currentRegisters->de.d = currentRegisters->bc.b;}
void Z80::LD_D_C(void){currentRegisters->de.d = currentRegisters->bc.c;}
void Z80::LD_D_D(void){currentRegisters->de.d = currentRegisters->de.d;}
void Z80::LD_D_E(void){currentRegisters->de.d = currentRegisters->de.e;}
void Z80::LD_D_H(void){currentRegisters->de.d = currentRegisters->hl.h;}
void Z80::LD_D_L(void){currentRegisters->de.d = currentRegisters->hl.l;}
void Z80::LD_D_iHL(void){currentRegisters->de.d = memory->ReadByte(currentRegisters->hl.hl);}
void Z80::LD_D_A(void){currentRegisters->de.d = currentRegisters->af.a;}
void Z80::LD_E_B(void){currentRegisters->de.e = currentRegisters->bc.b;}
void Z80::LD_E_C(void){currentRegisters->de.e = currentRegisters->bc.c;}
void Z80::LD_E_D(void){currentRegisters->de.e = currentRegisters->de.d;}
void Z80::LD_E_E(void){currentRegisters->de.e = currentRegisters->de.e;}
void Z80::LD_E_H(void){currentRegisters->de.e = currentRegisters->hl.h;}
void Z80::LD_E_L(void){currentRegisters->de.e = currentRegisters->hl.l;}
void Z80::LD_E_iHL(void){currentRegisters->de.e = memory->ReadByte(currentRegisters->hl.hl);}
void Z80::LD_E_A(void){currentRegisters->de.e = currentRegisters->af.a;}
void Z80::LD_H_B(void){currentRegisters->hl.h = currentRegisters->bc.b;}
void Z80::LD_H_C(void){currentRegisters->hl.h = currentRegisters->bc.c;}
void Z80::LD_H_D(void){currentRegisters->hl.h = currentRegisters->de.d;}
void Z80::LD_H_E(void){currentRegisters->hl.h = currentRegisters->de.e;}
void Z80::LD_H_H(void){currentRegisters->hl.h = currentRegisters->hl.h;}
void Z80::LD_H_L(void){currentRegisters->hl.h = currentRegisters->hl.l;}
void Z80::LD_H_iHL(void){currentRegisters->hl.h = memory->ReadByte(currentRegisters->hl.hl);}
void Z80::LD_H_A(void){currentRegisters->hl.h = currentRegisters->af.a;}
void Z80::LD_L_B(void){currentRegisters->hl.l = currentRegisters->bc.b;}
void Z80::LD_L_C(void){currentRegisters->hl.l = currentRegisters->bc.c;}
void Z80::LD_L_D(void){currentRegisters->hl.l = currentRegisters->de.d;}
void Z80::LD_L_E(void){currentRegisters->hl.l = currentRegisters->de.e;}
void Z80::LD_L_H(void){currentRegisters->hl.l = currentRegisters->hl.h;}
void Z80::LD_L_L(void){currentRegisters->hl.l = currentRegisters->hl.l;}
void Z80::LD_L_iHL(void){currentRegisters->hl.l = memory->ReadByte(currentRegisters->hl.hl);}
void Z80::LD_L_A(void){currentRegisters->hl.l = currentRegisters->af.a;}
void Z80::LD_iHL_B(void){memory->WriteByte(currentRegisters->hl.hl, currentRegisters->bc.b);}
void Z80::LD_iHL_C(void){memory->WriteByte(currentRegisters->hl.hl, currentRegisters->bc.c);}
void Z80::LD_iHL_D(void){memory->WriteByte(currentRegisters->hl.hl, currentRegisters->de.d);}
void Z80::LD_iHL_E(void){memory->WriteByte(currentRegisters->hl.hl, currentRegisters->de.e);}
void Z80::LD_iHL_H(void){memory->WriteByte(currentRegisters->hl.hl, currentRegisters->hl.h);}
void Z80::LD_iHL_L(void){memory->WriteByte(currentRegisters->hl.hl, currentRegisters->hl.l);}
void Z80::HALT(void){}
void Z80::LD_iHL_A(void){memory->WriteByte(currentRegisters->hl.hl, currentRegisters->af.a);}
void Z80::LD_A_B(void){currentRegisters->af.a = currentRegisters->bc.b;}
void Z80::LD_A_C(void){currentRegisters->af.a = currentRegisters->bc.c;}
void Z80::LD_A_D(void){currentRegisters->af.a = currentRegisters->de.d;}
void Z80::LD_A_E(void){currentRegisters->af.a = currentRegisters->de.e;}
void Z80::LD_A_H(void){currentRegisters->af.a = currentRegisters->hl.h;}
void Z80::LD_A_L(void){currentRegisters->af.a = currentRegisters->hl.l;}
void Z80::LD_A_iHL(void){currentRegisters->af.a = memory->ReadByte(currentRegisters->hl.hl);}
void Z80::LD_A_A(void){currentRegisters->af.a = currentRegisters->af.a;}
void Z80::ADD_A_B(void){}
void Z80::ADD_A_C(void){}
void Z80::ADD_A_D(void){}
void Z80::ADD_A_E(void){}
void Z80::ADD_A_H(void){}
void Z80::ADD_A_L(void){}
void Z80::ADD_A_iHL(void){}
void Z80::ADD_A_A(void){}
void Z80::ADC_A_B(void){}
void Z80::ADC_A_C(void){}
void Z80::ADC_A_D(void){}
void Z80::ADC_A_E(void){}
void Z80::ADC_A_H(void){}
void Z80::ADC_A_L(void){}
void Z80::ADC_A_iHL(void){}
void Z80::ADC_A_A(void){}
void Z80::SUB_A_B(void){}
void Z80::SUB_A_C(void){}
void Z80::SUB_A_D(void){}
void Z80::SUB_A_E(void){}
void Z80::SUB_A_H(void){}
void Z80::SUB_A_L(void){}
void Z80::SUB_A_iHL(void){}
void Z80::SUB_A_A(void){}
void Z80::SBC_A_B(void){}
void Z80::SBC_A_C(void){}
void Z80::SBC_A_D(void){}
void Z80::SBC_A_E(void){}
void Z80::SBC_A_H(void){}
void Z80::SBC_A_L(void){}
void Z80::SBC_A_iHL(void){}
void Z80::SBC_A_A(void){}
void Z80::AND_B(void){}
void Z80::AND_C(void){}
void Z80::AND_D(void){}
void Z80::AND_E(void){}
void Z80::AND_H(void){}
void Z80::AND_L(void){}
void Z80::AND_iHL(void){}
void Z80::AND_A(void){}
void Z80::XOR_B(void){}
void Z80::XOR_C(void){}
void Z80::XOR_D(void){}
void Z80::XOR_E(void){}
void Z80::XOR_H(void){}
void Z80::XOR_L(void){}
void Z80::XOR_iHL(void){}
void Z80::XOR_A(void){}
void Z80::OR_B(void){}
void Z80::OR_C(void){}
void Z80::OR_D(void){}
void Z80::OR_E(void){}
void Z80::OR_H(void){}
void Z80::OR_L(void){}
void Z80::OR_iHL(void){}
void Z80::OR_A(void){}
void Z80::CP_B(void){}
void Z80::CP_C(void){}
void Z80::CP_D(void){}
void Z80::CP_E(void){}
void Z80::CP_H(void){}
void Z80::CP_L(void){}
void Z80::CP_iHL(void){}
void Z80::CP_A(void){}
void Z80::RET_NZ(void){}
void Z80::POP_BC(void){}
void Z80::JP_NZ_word(void){}
void Z80::JP_word(void){}
void Z80::CALL_NZ_word(void){}
void Z80::PUSH_BC(void){}
void Z80::ADD_A_byte(void){}
void Z80::RST_00(void){}
void Z80::RET_Z(void){}
void Z80::RET(void){}
void Z80::JP_Z_word(void){}
void Z80::CB(void){}
void Z80::CALL_Z_word(void){}
void Z80::CALL_word(void){}
void Z80::ADC_A_byte(void){}
void Z80::RST_08(void){}
void Z80::RET_NC(void){}
void Z80::POP_DE(void){}
void Z80::JP_NC_word(void){}
void Z80::OUT_iNN_A(void){}
void Z80::CALL_NC_word(void){}
void Z80::PUSH_DE(void){}
void Z80::SUB_A_byte(void){}
void Z80::RST_10(void){}
void Z80::RET_C(void){}
void Z80::EXX(void){}
void Z80::JP_C_word(void){}
void Z80::IN_A_iNN(void){}
void Z80::CALL_C_word(void){}
void Z80::DD(void){}
void Z80::SBC_A_byte(void){}
void Z80::RST_18(void){}
void Z80::RET_PO(void){}
void Z80::POP_HL(void){}
void Z80::JP_PO_word(void){}
void Z80::EX_iSP_HL(void){}
void Z80::CALL_PO_word(void){}
void Z80::PUSH_HL(void){}
void Z80::AND_byte(void){}
void Z80::RST_20(void){}
void Z80::RET_PE(void){}
void Z80::JP_iHL(void){}
void Z80::JP_PE_word(void){}
void Z80::EX_DE_HL(void){uint16_t de = currentRegisters->de.de; currentRegisters->de.de = currentRegisters->hl.hl; currentRegisters->hl.hl = de;}
void Z80::CALL_PE_word(void){}
void Z80::ED(void){}
void Z80::XOR_byte(void){}
void Z80::RST_28(void){}
void Z80::RET_P(void){}
void Z80::POP_AF(void){}
void Z80::JP_P_word(void){}
void Z80::DI(void){}
void Z80::CALL_P_word(void){}
void Z80::PUSH_AF(void){}
void Z80::OR_byte(void){}
void Z80::RST_30(void){}
void Z80::RET_M(void){}
void Z80::LD_SP_HL(void){}
void Z80::JP_M_word(void){}
void Z80::EI(void){}
void Z80::CALL_M_word(void){}
void Z80::FD(void){}
void Z80::CP_byte(void){}
void Z80::RST_38(void){}
