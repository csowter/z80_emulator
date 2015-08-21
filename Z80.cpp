#include "Z80.h"
#include "MemoryInterface.h"
#include "IOInterface.h"
#include "OpCodes.h"
#include <iostream>

#define INSTRUCTION_TRACE

#ifdef INSTRUCTION_TRACE
  #define TRACE(x) std::cout << x << std::endl
#else
  #define TRACE(x)
#endif

Z80::Z80(MemoryInterface *memoryInterface, IOInterface *ioInterface)
  : mainRegisters(), alternateRegisters(), indexRegisters(), otherRegisters(), PC(0), memory(memoryInterface), io(ioInterface), Op(new fptr[OpCodes::NUMBER_OF_OPCODES]), CBOp(new fptr[CB_OpCodes::NUMBER_OF_OPCODES]), flagTable(new uint8_t[256])
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

  //CB Opcodes
  CBOp[CB_OpCodes::RLC_B] = &Z80::RLC_B;
  CBOp[CB_OpCodes::RLC_C] = &Z80::RLC_C;
  CBOp[CB_OpCodes::RLC_D] = &Z80::RLC_D;
  CBOp[CB_OpCodes::RLC_E] = &Z80::RLC_E;
  CBOp[CB_OpCodes::RLC_H] = &Z80::RLC_H;
  CBOp[CB_OpCodes::RLC_L] = &Z80::RLC_L;
  CBOp[CB_OpCodes::RLC_iHL] = &Z80::RLC_iHL;
  CBOp[CB_OpCodes::RLC_A] = &Z80::RLC_A;
  CBOp[CB_OpCodes::RRC_B] = &Z80::RRC_B;
  CBOp[CB_OpCodes::RRC_C] = &Z80::RRC_C;
  CBOp[CB_OpCodes::RRC_D] = &Z80::RRC_D;
  CBOp[CB_OpCodes::RRC_E] = &Z80::RRC_E;
  CBOp[CB_OpCodes::RRC_H] = &Z80::RRC_H;
  CBOp[CB_OpCodes::RRC_L] = &Z80::RRC_L;
  CBOp[CB_OpCodes::RRC_iHL] = &Z80::RRC_iHL;
  CBOp[CB_OpCodes::RRC_A] = &Z80::RRC_A;
  CBOp[CB_OpCodes::RL_B] = &Z80::RL_B;
  CBOp[CB_OpCodes::RL_C] = &Z80::RL_C;
  CBOp[CB_OpCodes::RL_D] = &Z80::RL_D;
  CBOp[CB_OpCodes::RL_E] = &Z80::RL_E;
  CBOp[CB_OpCodes::RL_H] = &Z80::RL_H;
  CBOp[CB_OpCodes::RL_L] = &Z80::RL_L;
  CBOp[CB_OpCodes::RL_iHL] = &Z80::RL_iHL;
  CBOp[CB_OpCodes::RL_A] = &Z80::RL_A;
  CBOp[CB_OpCodes::RR_B] = &Z80::RR_B;
  CBOp[CB_OpCodes::RR_C] = &Z80::RR_C;
  CBOp[CB_OpCodes::RR_D] = &Z80::RR_D;
  CBOp[CB_OpCodes::RR_E] = &Z80::RR_E;
  CBOp[CB_OpCodes::RR_H] = &Z80::RR_H;
  CBOp[CB_OpCodes::RR_L] = &Z80::RR_L;
  CBOp[CB_OpCodes::RR_iHL] = &Z80::RR_iHL;
  CBOp[CB_OpCodes::RR_A] = &Z80::RR_A;
  CBOp[CB_OpCodes::SLA_B] = &Z80::SLA_B;
  CBOp[CB_OpCodes::SLA_C] = &Z80::SLA_C;
  CBOp[CB_OpCodes::SLA_D] = &Z80::SLA_D;
  CBOp[CB_OpCodes::SLA_E] = &Z80::SLA_E;
  CBOp[CB_OpCodes::SLA_H] = &Z80::SLA_H;
  CBOp[CB_OpCodes::SLA_L] = &Z80::SLA_L;
  CBOp[CB_OpCodes::SLA_iHL] = &Z80::SLA_iHL;
  CBOp[CB_OpCodes::SLA_A] = &Z80::SLA_A;
  CBOp[CB_OpCodes::SRA_B] = &Z80::SRA_B;
  CBOp[CB_OpCodes::SRA_C] = &Z80::SRA_C;
  CBOp[CB_OpCodes::SRA_D] = &Z80::SRA_D;
  CBOp[CB_OpCodes::SRA_E] = &Z80::SRA_E;
  CBOp[CB_OpCodes::SRA_H] = &Z80::SRA_H;
  CBOp[CB_OpCodes::SRA_L] = &Z80::SRA_L;
  CBOp[CB_OpCodes::SRA_iHL] = &Z80::SRA_iHL;
  CBOp[CB_OpCodes::SRA_A] = &Z80::SRA_A;
  CBOp[CB_OpCodes::SLS_B] = &Z80::SLS_B;
  CBOp[CB_OpCodes::SLS_C] = &Z80::SLS_C;
  CBOp[CB_OpCodes::SLS_D] = &Z80::SLS_D;
  CBOp[CB_OpCodes::SLS_E] = &Z80::SLS_E;
  CBOp[CB_OpCodes::SLS_H] = &Z80::SLS_H;
  CBOp[CB_OpCodes::SLS_L] = &Z80::SLS_L;
  CBOp[CB_OpCodes::SLS_iHL] = &Z80::SLS_iHL;
  CBOp[CB_OpCodes::SLS_A] = &Z80::SLS_A;
  CBOp[CB_OpCodes::SRL_B] = &Z80::SRL_B;
  CBOp[CB_OpCodes::SRL_C] = &Z80::SRL_C;
  CBOp[CB_OpCodes::SRL_D] = &Z80::SRL_D;
  CBOp[CB_OpCodes::SRL_E] = &Z80::SRL_E;
  CBOp[CB_OpCodes::SRL_H] = &Z80::SRL_H;
  CBOp[CB_OpCodes::SRL_L] = &Z80::SRL_L;
  CBOp[CB_OpCodes::SRL_iHL] = &Z80::SRL_iHL;
  CBOp[CB_OpCodes::SRL_A] = &Z80::SRL_A;
  CBOp[CB_OpCodes::BIT_0_B] = &Z80::BIT_0_B;
  CBOp[CB_OpCodes::BIT_0_C] = &Z80::BIT_0_C;
  CBOp[CB_OpCodes::BIT_0_D] = &Z80::BIT_0_D;
  CBOp[CB_OpCodes::BIT_0_E] = &Z80::BIT_0_E;
  CBOp[CB_OpCodes::BIT_0_H] = &Z80::BIT_0_H;
  CBOp[CB_OpCodes::BIT_0_L] = &Z80::BIT_0_L;
  CBOp[CB_OpCodes::BIT_0_iHL] = &Z80::BIT_0_iHL;
  CBOp[CB_OpCodes::BIT_0_A] = &Z80::BIT_0_A;
  CBOp[CB_OpCodes::BIT_1_B] = &Z80::BIT_1_B;
  CBOp[CB_OpCodes::BIT_1_C] = &Z80::BIT_1_C;
  CBOp[CB_OpCodes::BIT_1_D] = &Z80::BIT_1_D;
  CBOp[CB_OpCodes::BIT_1_E] = &Z80::BIT_1_E;
  CBOp[CB_OpCodes::BIT_1_H] = &Z80::BIT_1_H;
  CBOp[CB_OpCodes::BIT_1_L] = &Z80::BIT_1_L;
  CBOp[CB_OpCodes::BIT_1_iHL] = &Z80::BIT_1_iHL;
  CBOp[CB_OpCodes::BIT_1_A] = &Z80::BIT_1_A;
  CBOp[CB_OpCodes::BIT_2_B] = &Z80::BIT_2_B;
  CBOp[CB_OpCodes::BIT_2_C] = &Z80::BIT_2_C;
  CBOp[CB_OpCodes::BIT_2_D] = &Z80::BIT_2_D;
  CBOp[CB_OpCodes::BIT_2_E] = &Z80::BIT_2_E;
  CBOp[CB_OpCodes::BIT_2_H] = &Z80::BIT_2_H;
  CBOp[CB_OpCodes::BIT_2_L] = &Z80::BIT_2_L;
  CBOp[CB_OpCodes::BIT_2_iHL] = &Z80::BIT_2_iHL;
  CBOp[CB_OpCodes::BIT_2_A] = &Z80::BIT_2_A;
  CBOp[CB_OpCodes::BIT_3_B] = &Z80::BIT_3_B;
  CBOp[CB_OpCodes::BIT_3_C] = &Z80::BIT_3_C;
  CBOp[CB_OpCodes::BIT_3_D] = &Z80::BIT_3_D;
  CBOp[CB_OpCodes::BIT_3_E] = &Z80::BIT_3_E;
  CBOp[CB_OpCodes::BIT_3_H] = &Z80::BIT_3_H;
  CBOp[CB_OpCodes::BIT_3_L] = &Z80::BIT_3_L;
  CBOp[CB_OpCodes::BIT_3_iHL] = &Z80::BIT_3_iHL;
  CBOp[CB_OpCodes::BIT_3_A] = &Z80::BIT_3_A;
  CBOp[CB_OpCodes::BIT_4_B] = &Z80::BIT_4_B;
  CBOp[CB_OpCodes::BIT_4_C] = &Z80::BIT_4_C;
  CBOp[CB_OpCodes::BIT_4_D] = &Z80::BIT_4_D;
  CBOp[CB_OpCodes::BIT_4_E] = &Z80::BIT_4_E;
  CBOp[CB_OpCodes::BIT_4_H] = &Z80::BIT_4_H;
  CBOp[CB_OpCodes::BIT_4_L] = &Z80::BIT_4_L;
  CBOp[CB_OpCodes::BIT_4_iHL] = &Z80::BIT_4_iHL;
  CBOp[CB_OpCodes::BIT_4_A] = &Z80::BIT_4_A;
  CBOp[CB_OpCodes::BIT_5_B] = &Z80::BIT_5_B;
  CBOp[CB_OpCodes::BIT_5_C] = &Z80::BIT_5_C;
  CBOp[CB_OpCodes::BIT_5_D] = &Z80::BIT_5_D;
  CBOp[CB_OpCodes::BIT_5_E] = &Z80::BIT_5_E;
  CBOp[CB_OpCodes::BIT_5_H] = &Z80::BIT_5_H;
  CBOp[CB_OpCodes::BIT_5_L] = &Z80::BIT_5_L;
  CBOp[CB_OpCodes::BIT_5_iHL] = &Z80::BIT_5_iHL;
  CBOp[CB_OpCodes::BIT_5_A] = &Z80::BIT_5_A;
  CBOp[CB_OpCodes::BIT_6_B] = &Z80::BIT_6_B;
  CBOp[CB_OpCodes::BIT_6_C] = &Z80::BIT_6_C;
  CBOp[CB_OpCodes::BIT_6_D] = &Z80::BIT_6_D;
  CBOp[CB_OpCodes::BIT_6_E] = &Z80::BIT_6_E;
  CBOp[CB_OpCodes::BIT_6_H] = &Z80::BIT_6_H;
  CBOp[CB_OpCodes::BIT_6_L] = &Z80::BIT_6_L;
  CBOp[CB_OpCodes::BIT_6_iHL] = &Z80::BIT_6_iHL;
  CBOp[CB_OpCodes::BIT_6_A] = &Z80::BIT_6_A;
  CBOp[CB_OpCodes::BIT_7_B] = &Z80::BIT_7_B;
  CBOp[CB_OpCodes::BIT_7_C] = &Z80::BIT_7_C;
  CBOp[CB_OpCodes::BIT_7_D] = &Z80::BIT_7_D;
  CBOp[CB_OpCodes::BIT_7_E] = &Z80::BIT_7_E;
  CBOp[CB_OpCodes::BIT_7_H] = &Z80::BIT_7_H;
  CBOp[CB_OpCodes::BIT_7_L] = &Z80::BIT_7_L;
  CBOp[CB_OpCodes::BIT_7_iHL] = &Z80::BIT_7_iHL;
  CBOp[CB_OpCodes::BIT_7_A] = &Z80::BIT_7_A;
  CBOp[CB_OpCodes::RES_0_B] = &Z80::RES_0_B;
  CBOp[CB_OpCodes::RES_0_C] = &Z80::RES_0_C;
  CBOp[CB_OpCodes::RES_0_D] = &Z80::RES_0_D;
  CBOp[CB_OpCodes::RES_0_E] = &Z80::RES_0_E;
  CBOp[CB_OpCodes::RES_0_H] = &Z80::RES_0_H;
  CBOp[CB_OpCodes::RES_0_L] = &Z80::RES_0_L;
  CBOp[CB_OpCodes::RES_0_iHL] = &Z80::RES_0_iHL;
  CBOp[CB_OpCodes::RES_0_A] = &Z80::RES_0_A;
  CBOp[CB_OpCodes::RES_1_B] = &Z80::RES_1_B;
  CBOp[CB_OpCodes::RES_1_C] = &Z80::RES_1_C;
  CBOp[CB_OpCodes::RES_1_D] = &Z80::RES_1_D;
  CBOp[CB_OpCodes::RES_1_E] = &Z80::RES_1_E;
  CBOp[CB_OpCodes::RES_1_H] = &Z80::RES_1_H;
  CBOp[CB_OpCodes::RES_1_L] = &Z80::RES_1_L;
  CBOp[CB_OpCodes::RES_1_iHL] = &Z80::RES_1_iHL;
  CBOp[CB_OpCodes::RES_1_A] = &Z80::RES_1_A;
  CBOp[CB_OpCodes::RES_2_B] = &Z80::RES_2_B;
  CBOp[CB_OpCodes::RES_2_C] = &Z80::RES_2_C;
  CBOp[CB_OpCodes::RES_2_D] = &Z80::RES_2_D;
  CBOp[CB_OpCodes::RES_2_E] = &Z80::RES_2_E;
  CBOp[CB_OpCodes::RES_2_H] = &Z80::RES_2_H;
  CBOp[CB_OpCodes::RES_2_L] = &Z80::RES_2_L;
  CBOp[CB_OpCodes::RES_2_iHL] = &Z80::RES_2_iHL;
  CBOp[CB_OpCodes::RES_2_A] = &Z80::RES_2_A;
  CBOp[CB_OpCodes::RES_3_B] = &Z80::RES_3_B;
  CBOp[CB_OpCodes::RES_3_C] = &Z80::RES_3_C;
  CBOp[CB_OpCodes::RES_3_D] = &Z80::RES_3_D;
  CBOp[CB_OpCodes::RES_3_E] = &Z80::RES_3_E;
  CBOp[CB_OpCodes::RES_3_H] = &Z80::RES_3_H;
  CBOp[CB_OpCodes::RES_3_L] = &Z80::RES_3_L;
  CBOp[CB_OpCodes::RES_3_iHL] = &Z80::RES_3_iHL;
  CBOp[CB_OpCodes::RES_3_A] = &Z80::RES_3_A;
  CBOp[CB_OpCodes::RES_4_B] = &Z80::RES_4_B;
  CBOp[CB_OpCodes::RES_4_C] = &Z80::RES_4_C;
  CBOp[CB_OpCodes::RES_4_D] = &Z80::RES_4_D;
  CBOp[CB_OpCodes::RES_4_E] = &Z80::RES_4_E;
  CBOp[CB_OpCodes::RES_4_H] = &Z80::RES_4_H;
  CBOp[CB_OpCodes::RES_4_L] = &Z80::RES_4_L;
  CBOp[CB_OpCodes::RES_4_iHL] = &Z80::RES_4_iHL;
  CBOp[CB_OpCodes::RES_4_A] = &Z80::RES_4_A;
  CBOp[CB_OpCodes::RES_5_B] = &Z80::RES_5_B;
  CBOp[CB_OpCodes::RES_5_C] = &Z80::RES_5_C;
  CBOp[CB_OpCodes::RES_5_D] = &Z80::RES_5_D;
  CBOp[CB_OpCodes::RES_5_E] = &Z80::RES_5_E;
  CBOp[CB_OpCodes::RES_5_H] = &Z80::RES_5_H;
  CBOp[CB_OpCodes::RES_5_L] = &Z80::RES_5_L;
  CBOp[CB_OpCodes::RES_5_iHL] = &Z80::RES_5_iHL;
  CBOp[CB_OpCodes::RES_5_A] = &Z80::RES_5_A;
  CBOp[CB_OpCodes::RES_6_B] = &Z80::RES_6_B;
  CBOp[CB_OpCodes::RES_6_C] = &Z80::RES_6_C;
  CBOp[CB_OpCodes::RES_6_D] = &Z80::RES_6_D;
  CBOp[CB_OpCodes::RES_6_E] = &Z80::RES_6_E;
  CBOp[CB_OpCodes::RES_6_H] = &Z80::RES_6_H;
  CBOp[CB_OpCodes::RES_6_L] = &Z80::RES_6_L;
  CBOp[CB_OpCodes::RES_6_iHL] = &Z80::RES_6_iHL;
  CBOp[CB_OpCodes::RES_6_A] = &Z80::RES_6_A;
  CBOp[CB_OpCodes::RES_7_B] = &Z80::RES_7_B;
  CBOp[CB_OpCodes::RES_7_C] = &Z80::RES_7_C;
  CBOp[CB_OpCodes::RES_7_D] = &Z80::RES_7_D;
  CBOp[CB_OpCodes::RES_7_E] = &Z80::RES_7_E;
  CBOp[CB_OpCodes::RES_7_H] = &Z80::RES_7_H;
  CBOp[CB_OpCodes::RES_7_L] = &Z80::RES_7_L;
  CBOp[CB_OpCodes::RES_7_iHL] = &Z80::RES_7_iHL;
  CBOp[CB_OpCodes::RES_7_A] = &Z80::RES_7_A;
  CBOp[CB_OpCodes::SET_0_B] = &Z80::SET_0_B;
  CBOp[CB_OpCodes::SET_0_C] = &Z80::SET_0_C;
  CBOp[CB_OpCodes::SET_0_D] = &Z80::SET_0_D;
  CBOp[CB_OpCodes::SET_0_E] = &Z80::SET_0_E;
  CBOp[CB_OpCodes::SET_0_H] = &Z80::SET_0_H;
  CBOp[CB_OpCodes::SET_0_L] = &Z80::SET_0_L;
  CBOp[CB_OpCodes::SET_0_iHL] = &Z80::SET_0_iHL;
  CBOp[CB_OpCodes::SET_0_A] = &Z80::SET_0_A;
  CBOp[CB_OpCodes::SET_1_B] = &Z80::SET_1_B;
  CBOp[CB_OpCodes::SET_1_C] = &Z80::SET_1_C;
  CBOp[CB_OpCodes::SET_1_D] = &Z80::SET_1_D;
  CBOp[CB_OpCodes::SET_1_E] = &Z80::SET_1_E;
  CBOp[CB_OpCodes::SET_1_H] = &Z80::SET_1_H;
  CBOp[CB_OpCodes::SET_1_L] = &Z80::SET_1_L;
  CBOp[CB_OpCodes::SET_1_iHL] = &Z80::SET_1_iHL;
  CBOp[CB_OpCodes::SET_1_A] = &Z80::SET_1_A;
  CBOp[CB_OpCodes::SET_2_B] = &Z80::SET_2_B;
  CBOp[CB_OpCodes::SET_2_C] = &Z80::SET_2_C;
  CBOp[CB_OpCodes::SET_2_D] = &Z80::SET_2_D;
  CBOp[CB_OpCodes::SET_2_E] = &Z80::SET_2_E;
  CBOp[CB_OpCodes::SET_2_H] = &Z80::SET_2_H;
  CBOp[CB_OpCodes::SET_2_L] = &Z80::SET_2_L;
  CBOp[CB_OpCodes::SET_2_iHL] = &Z80::SET_2_iHL;
  CBOp[CB_OpCodes::SET_2_A] = &Z80::SET_2_A;
  CBOp[CB_OpCodes::SET_3_B] = &Z80::SET_3_B;
  CBOp[CB_OpCodes::SET_3_C] = &Z80::SET_3_C;
  CBOp[CB_OpCodes::SET_3_D] = &Z80::SET_3_D;
  CBOp[CB_OpCodes::SET_3_E] = &Z80::SET_3_E;
  CBOp[CB_OpCodes::SET_3_H] = &Z80::SET_3_H;
  CBOp[CB_OpCodes::SET_3_L] = &Z80::SET_3_L;
  CBOp[CB_OpCodes::SET_3_iHL] = &Z80::SET_3_iHL;
  CBOp[CB_OpCodes::SET_3_A] = &Z80::SET_3_A;
  CBOp[CB_OpCodes::SET_4_B] = &Z80::SET_4_B;
  CBOp[CB_OpCodes::SET_4_C] = &Z80::SET_4_C;
  CBOp[CB_OpCodes::SET_4_D] = &Z80::SET_4_D;
  CBOp[CB_OpCodes::SET_4_E] = &Z80::SET_4_E;
  CBOp[CB_OpCodes::SET_4_H] = &Z80::SET_4_H;
  CBOp[CB_OpCodes::SET_4_L] = &Z80::SET_4_L;
  CBOp[CB_OpCodes::SET_4_iHL] = &Z80::SET_4_iHL;
  CBOp[CB_OpCodes::SET_4_A] = &Z80::SET_4_A;
  CBOp[CB_OpCodes::SET_5_B] = &Z80::SET_5_B;
  CBOp[CB_OpCodes::SET_5_C] = &Z80::SET_5_C;
  CBOp[CB_OpCodes::SET_5_D] = &Z80::SET_5_D;
  CBOp[CB_OpCodes::SET_5_E] = &Z80::SET_5_E;
  CBOp[CB_OpCodes::SET_5_H] = &Z80::SET_5_H;
  CBOp[CB_OpCodes::SET_5_L] = &Z80::SET_5_L;
  CBOp[CB_OpCodes::SET_5_iHL] = &Z80::SET_5_iHL;
  CBOp[CB_OpCodes::SET_5_A] = &Z80::SET_5_A;
  CBOp[CB_OpCodes::SET_6_B] = &Z80::SET_6_B;
  CBOp[CB_OpCodes::SET_6_C] = &Z80::SET_6_C;
  CBOp[CB_OpCodes::SET_6_D] = &Z80::SET_6_D;
  CBOp[CB_OpCodes::SET_6_E] = &Z80::SET_6_E;
  CBOp[CB_OpCodes::SET_6_H] = &Z80::SET_6_H;
  CBOp[CB_OpCodes::SET_6_L] = &Z80::SET_6_L;
  CBOp[CB_OpCodes::SET_6_iHL] = &Z80::SET_6_iHL;
  CBOp[CB_OpCodes::SET_6_A] = &Z80::SET_6_A;
  CBOp[CB_OpCodes::SET_7_B] = &Z80::SET_7_B;
  CBOp[CB_OpCodes::SET_7_C] = &Z80::SET_7_C;
  CBOp[CB_OpCodes::SET_7_D] = &Z80::SET_7_D;
  CBOp[CB_OpCodes::SET_7_E] = &Z80::SET_7_E;
  CBOp[CB_OpCodes::SET_7_H] = &Z80::SET_7_H;
  CBOp[CB_OpCodes::SET_7_L] = &Z80::SET_7_L;
  CBOp[CB_OpCodes::SET_7_iHL] = &Z80::SET_7_iHL;
  CBOp[CB_OpCodes::SET_7_A] = &Z80::SET_7_A;

  memset(flagTable, 0x00, 256);
  flagTable[0] |= ZERO_BIT;
  for(uint16_t i = 0; i < 256; i++)
  {
    uint8_t bitsSet = 0;
    if(i & 0x01)
      bitsSet++;
    if(i & 0x02)
      bitsSet++;
    if(i & 0x04)
      bitsSet++;
    if(i & 0x08)
      bitsSet++;
    if(i & 0x10)
      bitsSet++;
    if(i & 0x20)
      bitsSet++;
    if(i & 0x40)
      bitsSet++;
    if(i & 0x80)
      bitsSet++;

    if(bitsSet & 0x01)
      flagTable[i] |= PARITY_OVERFLOW_BIT;
    flagTable[i] |= 0x80;
  }
}

Z80::~Z80(void)
{
  delete[] flagTable;
  delete[] CBOp;
  delete[] Op;  
}

void Z80::Execute(void)
{
  uint8_t instruction = memory->ReadByte(PC++);
  (this->*Op[instruction])();
}

void Z80::NOP(void)
{
	TRACE("NOP");
}               

void Z80::LD_BC_word(void)
{

	TRACE("LD_BC_word"); 

	mainRegisters.bc.c = memory->ReadByte(PC++);
	mainRegisters.bc.b = memory->ReadByte(PC++);
}
        
void Z80::LD_iBC_A(void)
{

	TRACE("LD_iBC_A");


	memory->WriteByte(mainRegisters.bc.bc, mainRegisters.af.a);
}          

void Z80::INC_BC(void)
{

	TRACE("INC_BC");

	++mainRegisters.bc.bc;
}
            
void Z80::INC_B(void)
{
  TRACE("INC_B");
  mainRegisters.bc.b++;
  mainRegisters.af.f &= ~(SUBTRACT_BIT | SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.bc.b);
  if(mainRegisters.bc.b == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.bc.b == 0x80)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.bc.b == 0x10)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}

void Z80::DEC_B(void)
{
  TRACE("DEC_B");
  --mainRegisters.bc.b;
  mainRegisters.af.f &= ~(SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.bc.b);
  if(mainRegisters.bc.b == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.bc.b == 0x7F)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.bc.b == 0x0F)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}             

void Z80::LD_B_byte(void)
{

	TRACE("LD_B_byte"); 

	mainRegisters.bc.b = memory->ReadByte(PC++);
}    

void Z80::RLCA(void)
{
  TRACE("RLCA");
  mainRegisters.af.f &= ~(CARRY_BIT | HALF_CARRY_BIT | SUBTRACT_BIT);
  mainRegisters.af.a = (mainRegisters.af.a << 1) | (mainRegisters.af.a >> 7);
  mainRegisters.af.f |= mainRegisters.af.a & CARRY_BIT;
}

void Z80::EX_AF_AF(void)
{
  TRACE("EX_AF_AF");
  uint16_t af = mainRegisters.af.af;
  mainRegisters.af.af = alternateRegisters.af.af;
  alternateRegisters.af.af = af;
}

void Z80::ADD_HL_BC(void){}

void Z80::LD_A_iBC(void)
{

	TRACE("LD_A_iBC"); 

	mainRegisters.af.a = memory->ReadByte(mainRegisters.bc.bc);
}

void Z80::DEC_BC(void)
{

	TRACE("DEC_BC"); 

	--mainRegisters.bc.bc;
}

void Z80::INC_C(void)
{
  TRACE("INC_C");
  ++mainRegisters.bc.c;
  mainRegisters.af.f &= ~(SUBTRACT_BIT | SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.bc.c);
  if(mainRegisters.bc.c == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.bc.c == 0x80)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.bc.c == 0x10)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}
void Z80::DEC_C(void)
{
  TRACE("DEC_C");
  --mainRegisters.bc.c;
  mainRegisters.af.f &= ~(SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.bc.c);
  if(mainRegisters.bc.c == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.bc.c == 0x7F)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.bc.c == 0x0F)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}

void Z80::LD_C_byte(void)
{

	TRACE("LD_C_byte"); 

	mainRegisters.bc.c = memory->ReadByte(PC++);
}

void Z80::RRCA(void)
{
  
}

void Z80::DJNZ(void){}

void Z80::LD_DE_word(void)
{

	TRACE("LD_DE_word"); 

	mainRegisters.de.e = memory->ReadByte(PC++);
	mainRegisters.de.d = memory->ReadByte(PC++);
}

void Z80::LD_iDE_A(void)
{

	TRACE("LD_iDE_A"); 


	memory->WriteByte(mainRegisters.de.de, mainRegisters.af.a);
}

void Z80::INC_DE(void)
{

	TRACE("INC_DE"); 

	++mainRegisters.de.de;
}

void Z80::INC_D(void)
{
  TRACE("INC_D");
  ++mainRegisters.de.d;
  mainRegisters.af.f &= ~(SUBTRACT_BIT | SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.de.d);
  if(mainRegisters.de.d == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.de.d == 0x80)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.de.d == 0x10)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}
void Z80::DEC_D(void)
{
  TRACE("DEC_D");
  --mainRegisters.de.d;
  mainRegisters.af.f &= ~(SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.de.d);
  if(mainRegisters.de.d == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.de.d == 0x7F)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.de.d == 0x0F)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}

void Z80::LD_D_byte(void)
{

	TRACE("LD_D_byte"); 

	mainRegisters.de.d = memory->ReadByte(PC++);
}

void Z80::RLA(void){}

void Z80::JR(void)
{

	TRACE("JR"); 

	int8_t offset = memory->ReadByte(PC++);
	PC += offset;
}

void Z80::ADD_HL_DE(void){}

void Z80::LD_A_iDE(void)
{

	TRACE("LD_A_iDE"); 

	mainRegisters.af.a = memory->ReadByte(mainRegisters.de.de);
}

void Z80::DEC_DE(void)
{

	TRACE("DEC_DE"); 

	--mainRegisters.de.de;
}

void Z80::INC_E(void)
{
  TRACE("INC_E");
  ++mainRegisters.de.e;
  mainRegisters.af.f &= ~(SUBTRACT_BIT | SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.de.e);
  if(mainRegisters.de.e == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.de.e == 0x80)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.de.e == 0x10)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}

void Z80::DEC_E(void)
{
  TRACE("DEC_E");
  --mainRegisters.de.e;
  mainRegisters.af.f &= ~(SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.de.e);
  if(mainRegisters.de.e == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.de.e == 0x7F)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.de.e == 0x0F)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}

void Z80::LD_E_byte(void)
{

	TRACE("LD_E_byte"); 

	mainRegisters.de.e = memory->ReadByte(PC++);
}

void Z80::RRA(void){}

void Z80::JR_NZ(void)
{

	TRACE("JR_NZ"); 

	if(mainRegisters.af.f & ZERO_BIT)
	{
		++PC;
	}
	else
	{
		int8_t offset = memory->ReadByte(PC++);
		PC += offset;
	}
}

void Z80::LD_HL_word(void)
{

	TRACE("LD_HL_word"); 

	mainRegisters.hl.l = memory->ReadByte(PC++);
	mainRegisters.hl.h = memory->ReadByte(PC++);
}

void Z80::LD_iNN_HL(void)
{

	TRACE("LD_iNN_HL");

	uint16_t address = memory->ReadByte(PC++);
	address |= (memory->ReadByte(PC++) << 8);
	memory->WriteByte(address, mainRegisters.hl.l);
	++address;
	memory->WriteByte(address, mainRegisters.hl.h);
}

void Z80::INC_HL(void)
{

	TRACE("INC_HL"); 

	++mainRegisters.hl.hl;
}

void Z80::INC_H(void)
{
  TRACE("INC_H");
  ++mainRegisters.hl.h;
  mainRegisters.af.f &= ~(SUBTRACT_BIT | SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.hl.h);
  if(mainRegisters.hl.h == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.hl.h == 0x80)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.hl.h == 0x10)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}
void Z80::DEC_H(void)
{
  TRACE("DEC_H");
  --mainRegisters.hl.h;
  mainRegisters.af.f &= ~(SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.hl.h);
  if(mainRegisters.hl.h == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.hl.h == 0x7F)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.hl.h == 0x0F)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}

void Z80::LD_H_byte(void)
{

	TRACE("LD_H_byte"); 

	mainRegisters.hl.h = memory->ReadByte(PC++);
}

void Z80::DAA(void){}

void Z80::JR_Z(void)
{

	TRACE("JR_Z"); 

	if(mainRegisters.af.f & ZERO_BIT)
	{
		int8_t offset = memory->ReadByte(PC++);
		PC += offset;
	}
	else
	{
		++PC;
	}
}

void Z80::ADD_HL_HL(void){}

void Z80::LD_HL_iNN(void)
{

	TRACE("LD_HL_iNN"); 

	mainRegisters.hl.l = memory->ReadByte(PC++);
	mainRegisters.hl.h = memory->ReadByte(PC++);
}

void Z80::DEC_HL(void)
{

	TRACE("DEC_HL"); 

	--mainRegisters.hl.hl;
}

void Z80::INC_L(void)
{
  TRACE("INC_L");
  ++mainRegisters.hl.l;
  mainRegisters.af.f &= ~(SUBTRACT_BIT | SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.hl.l);
  if(mainRegisters.hl.l == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.hl.l == 0x80)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.hl.l == 0x10)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}

void Z80::DEC_L(void)
{
  TRACE("DEC_L");
  --mainRegisters.hl.l;
  mainRegisters.af.f &= ~(SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.hl.l);
  if(mainRegisters.hl.l == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.hl.l == 0x7F)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.hl.l == 0x0F)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}

void Z80::LD_L_byte(void)
{

	TRACE("LD_L_byte"); 

	mainRegisters.hl.l = memory->ReadByte(PC++);
}

void Z80::CPL(void){}

void Z80::JR_NC(void)
{

	TRACE("JR_NC"); 

	if(mainRegisters.af.f & CARRY_BIT)
	{
		++PC;
	}
	else
	{
		int8_t offset = memory->ReadByte(PC++);
		PC += offset;
	}
}

void Z80::LD_SP_word(void)
{

	TRACE("LD_SP_word"); 


	indexRegisters.sp = memory->ReadByte(PC++);
	indexRegisters.sp |= (memory->ReadByte(PC++) << 8);
}

void Z80::LD_iNN_A(void)
{

	TRACE("LD_iNN_A"); 

	uint16_t address = memory->ReadByte(PC++);
	address |= (memory->ReadByte(PC++) << 8);
	memory->WriteByte(address, mainRegisters.af.a);
}

void Z80::INC_SP(void)
{

	TRACE("INC_SP"); 

	++indexRegisters.sp;
}

void Z80::INC_iHL(void){}
void Z80::DEC_iHL(void){}

void Z80::LD_iHL_byte(void)
{

	TRACE("LD_iHL_byte"); 

	memory->WriteByte(mainRegisters.hl.hl, memory->ReadByte(PC++));
}

void Z80::SCF(void)
{
  TRACE("SCF");
  mainRegisters.af.f &= ~(HALF_CARRY_BIT | SUBTRACT_BIT);
  mainRegisters.af.f |= CARRY_BIT;
}

void Z80::JR_C(void)
{

	TRACE("JR_C"); 

	if(mainRegisters.af.f & CARRY_BIT)
	{
		int8_t offset = memory->ReadByte(PC++);
		PC += offset;
	}
	else
	{
		++PC;
	}
}

void Z80::ADD_HL_SP(void){}

void Z80::LD_A_iNN(void)
{

	TRACE("LD_A_iNN"); 

	uint16_t address = memory->ReadByte(PC++);
	address |= (memory->ReadByte(PC++) << 8);
	mainRegisters.af.a = memory->ReadByte(address);
}

void Z80::DEC_SP(void)
{

	TRACE("DEC_SP"); 

	--indexRegisters.sp;
}

void Z80::INC_A(void)
{
  TRACE("INC_A");
  mainRegisters.af.a++;
  mainRegisters.af.f &= ~(SUBTRACT_BIT | SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.af.a);
  if(mainRegisters.af.a == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.af.a == 0x80)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.af.a == 0x10)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}

void Z80::DEC_A(void)
{
  TRACE("DEC_A");
  --mainRegisters.af.a;
  mainRegisters.af.f &= ~(SIGN_BIT | ZERO_BIT | PARITY_OVERFLOW_BIT | HALF_CARRY_BIT);
  mainRegisters.af.f |= (SIGN_BIT & mainRegisters.af.a);
  if(mainRegisters.af.a == 0) 
    mainRegisters.af.f |= (ZERO_BIT);
  if(mainRegisters.af.a == 0x7F)
    mainRegisters.af.f |= PARITY_OVERFLOW_BIT;
  if(mainRegisters.af.a == 0x0F)
    mainRegisters.af.f |= HALF_CARRY_BIT;
}

void Z80::LD_A_byte(void)
{

	TRACE("LD_A_byte"); 

	mainRegisters.af.a = memory->ReadByte(PC++);
}

void Z80::CCF(void)
{
  TRACE("CCF");
  
  uint8_t temp = mainRegisters.af.f;
  temp &= CARRY_BIT;

  mainRegisters.af.f &= ~(HALF_CARRY_BIT | CARRY_BIT);
  mainRegisters.af.f |= (temp << 4);
  mainRegisters.af.f |= ((~temp) & CARRY_BIT);
}

void Z80::LD_B_B(void)
{
  TRACE("LD_B_B");
  mainRegisters.bc.b = mainRegisters.bc.b;
}

void Z80::LD_B_C(void)
{
  TRACE("LD_B_C");
  mainRegisters.bc.b = mainRegisters.bc.c;
}

void Z80::LD_B_D(void)
{
  TRACE("LD_B_D");
  mainRegisters.bc.b = mainRegisters.de.d;
}

void Z80::LD_B_E(void)
{
  TRACE("LD_B_E");
  mainRegisters.bc.b = mainRegisters.de.e;
}

void Z80::LD_B_H(void)
{
  TRACE("LD_B_H");
  mainRegisters.bc.b = mainRegisters.hl.h;
}

void Z80::LD_B_L(void)
{
  TRACE("LD_B_L");
  mainRegisters.bc.b = mainRegisters.hl.l;
}

void Z80::LD_B_iHL(void)
{
  TRACE("LD_B_iHL");
  mainRegisters.bc.b = memory->ReadByte(mainRegisters.hl.hl);
}

void Z80::LD_B_A(void)
{
  TRACE("LD_B_A");
  mainRegisters.bc.b = mainRegisters.af.a;
}

void Z80::LD_C_B(void)
{
  TRACE("LD_C_B");
  mainRegisters.bc.c = mainRegisters.bc.b;
}

void Z80::LD_C_C(void)
{
  TRACE("LD_C_C");
  mainRegisters.bc.c = mainRegisters.bc.c;
}

void Z80::LD_C_D(void)
{
  TRACE("LD_C_D");
  mainRegisters.bc.c = mainRegisters.de.d;
}

void Z80::LD_C_E(void)
{
  TRACE("LD_C_E");
  mainRegisters.bc.c = mainRegisters.de.e;
}

void Z80::LD_C_H(void)
{
  TRACE("LD_C_H");
  mainRegisters.bc.c = mainRegisters.hl.h;
}

void Z80::LD_C_L(void)
{
  TRACE("LD_C_L");
  mainRegisters.bc.c = mainRegisters.hl.l;
}

void Z80::LD_C_iHL(void)
{
  TRACE("LD_C_iHL");
  mainRegisters.bc.c = memory->ReadByte(mainRegisters.hl.hl);
}

void Z80::LD_C_A(void)
{
  TRACE("LD_C_A");
  mainRegisters.bc.c = mainRegisters.af.a;
}

void Z80::LD_D_B(void)
{
  TRACE("LD_D_B");
  mainRegisters.de.d = mainRegisters.bc.b;
}

void Z80::LD_D_C(void)
{
  TRACE("LD_D_C");
  mainRegisters.de.d = mainRegisters.bc.c;
}

void Z80::LD_D_D(void)
{
  TRACE("LD_D_D");
  mainRegisters.de.d = mainRegisters.de.d;
}

void Z80::LD_D_E(void)
{
  TRACE("LD_D_E");
  mainRegisters.de.d = mainRegisters.de.e;
}

void Z80::LD_D_H(void)
{
  TRACE("LD_D_H");
  mainRegisters.de.d = mainRegisters.hl.h;
}

void Z80::LD_D_L(void)
{
  TRACE("LD_D_L");
  mainRegisters.de.d = mainRegisters.hl.l;
}

void Z80::LD_D_iHL(void)
{
  TRACE("LD_D_iHL");
  mainRegisters.de.d = memory->ReadByte(mainRegisters.hl.hl);
}

void Z80::LD_D_A(void)
{
  TRACE("LD_D_A");
  mainRegisters.de.d = mainRegisters.af.a;
}

void Z80::LD_E_B(void)
{
  TRACE("LD_E_B");
  mainRegisters.de.e = mainRegisters.bc.b;
}

void Z80::LD_E_C(void)
{
  TRACE("LD_E_C");
  mainRegisters.de.e = mainRegisters.bc.c;
}

void Z80::LD_E_D(void)
{
  TRACE("LD_E_D");
  mainRegisters.de.e = mainRegisters.de.d;
}

void Z80::LD_E_E(void)
{
  TRACE("LD_E_E");
  mainRegisters.de.e = mainRegisters.de.e;
}

void Z80::LD_E_H(void)
{
  TRACE("LD_E_H");
  mainRegisters.de.e = mainRegisters.hl.h;
}

void Z80::LD_E_L(void)
{
  TRACE("LD_E_L");
  mainRegisters.de.e = mainRegisters.hl.l;
}

void Z80::LD_E_iHL(void)
{
  TRACE("LD_E_iHL");
  mainRegisters.de.e = memory->ReadByte(mainRegisters.hl.hl);
}

void Z80::LD_E_A(void)
{
  TRACE("LD_E_A");
  mainRegisters.de.e = mainRegisters.af.a;
}

void Z80::LD_H_B(void)
{
  TRACE("LD_H_B");
  mainRegisters.hl.h = mainRegisters.bc.b;
}

void Z80::LD_H_C(void)
{
  TRACE("LD_H_C");
  mainRegisters.hl.h = mainRegisters.bc.c;
}

void Z80::LD_H_D(void)
{
  TRACE("LD_H_D");
  mainRegisters.hl.h = mainRegisters.de.d;
}

void Z80::LD_H_E(void)
{
  TRACE("LD_H_E");
  mainRegisters.hl.h = mainRegisters.de.e;
}

void Z80::LD_H_H(void)
{
  TRACE("LD_H_H");
  mainRegisters.hl.h = mainRegisters.hl.h;
}

void Z80::LD_H_L(void)
{
  TRACE("LD_H_L");
  mainRegisters.hl.h = mainRegisters.hl.l;
}

void Z80::LD_H_iHL(void)
{
  TRACE("LD_H_iHL");
  mainRegisters.hl.h = memory->ReadByte(mainRegisters.hl.hl);
}

void Z80::LD_H_A(void)
{
  TRACE("LD_H_A");
  mainRegisters.hl.h = mainRegisters.af.a;
}

void Z80::LD_L_B(void)
{
  TRACE("LD_L_B");
  mainRegisters.hl.l = mainRegisters.bc.b;
}

void Z80::LD_L_C(void)
{
  TRACE("LD_L_C");
  mainRegisters.hl.l = mainRegisters.bc.c;
}

void Z80::LD_L_D(void)
{
  TRACE("LD_L_D");
  mainRegisters.hl.l = mainRegisters.de.d;
}

void Z80::LD_L_E(void)
{
  TRACE("LD_L_E");
  mainRegisters.hl.l = mainRegisters.de.e;
}

void Z80::LD_L_H(void)
{
  TRACE("LD_L_H");
  mainRegisters.hl.l = mainRegisters.hl.h;
}

void Z80::LD_L_L(void)
{
  TRACE("LD_L_L");
  mainRegisters.hl.l = mainRegisters.hl.l;
}

void Z80::LD_L_iHL(void)
{
  TRACE("LD_L_iHL");
  mainRegisters.hl.l = memory->ReadByte(mainRegisters.hl.hl);
}

void Z80::LD_L_A(void)
{
  TRACE("LD_L_A");
  mainRegisters.hl.l = mainRegisters.af.a;
}

void Z80::LD_iHL_B(void)
{
  TRACE("LD_iHL_B");
  memory->WriteByte(mainRegisters.hl.hl, mainRegisters.bc.b);
}

void Z80::LD_iHL_C(void)
{
  TRACE("LD_iHL_C");
  memory->WriteByte(mainRegisters.hl.hl, mainRegisters.bc.c);
}

void Z80::LD_iHL_D(void)
{
  TRACE("LD_iHL_D");
  memory->WriteByte(mainRegisters.hl.hl, mainRegisters.de.d);
}

void Z80::LD_iHL_E(void)
{
  TRACE("LD_iHL_E");
  memory->WriteByte(mainRegisters.hl.hl, mainRegisters.de.e);
}

void Z80::LD_iHL_H(void)
{
  TRACE("LD_iHL_H");
  memory->WriteByte(mainRegisters.hl.hl, mainRegisters.hl.h);
}

void Z80::LD_iHL_L(void)
{
  TRACE("LD_iHL_L");
  memory->WriteByte(mainRegisters.hl.hl, mainRegisters.hl.l);
}

void Z80::HALT(void)
{
  TRACE("HALT");
  //while(1);
}

void Z80::LD_iHL_A(void)
{
  TRACE("LD_iHL_A");
  memory->WriteByte(mainRegisters.hl.hl, mainRegisters.af.a);
}

void Z80::LD_A_B(void)
{
  TRACE("LD_A_B");
  mainRegisters.af.a = mainRegisters.bc.b;
}

void Z80::LD_A_C(void)
{
  TRACE("LD_A_C");
  mainRegisters.af.a = mainRegisters.bc.c;
}

void Z80::LD_A_D(void)
{
  TRACE("LD_A_D");
  mainRegisters.af.a = mainRegisters.de.d;
}

void Z80::LD_A_E(void)
{
  TRACE("LD_A_E");
  mainRegisters.af.a = mainRegisters.de.e;
}

void Z80::LD_A_H(void)
{
  TRACE("LD_A_H");
  mainRegisters.af.a = mainRegisters.hl.h;
}

void Z80::LD_A_L(void)
{
  TRACE("LD_A_L");
  mainRegisters.af.a = mainRegisters.hl.l;
}

void Z80::LD_A_iHL(void)
{
  TRACE("LD_A_iHL");
  mainRegisters.af.a = memory->ReadByte(mainRegisters.hl.hl);
}

void Z80::LD_A_A(void)
{
  TRACE("LD_A_A");
  mainRegisters.af.a = mainRegisters.af.a;
}

void Z80::ADD_A_B(void)
{
  TRACE("ADD_A_B");
  uint16_t sum = mainRegisters.af.a + mainRegisters.bc.b;
  mainRegisters.af.f = flagTable[(sum & 0xFF)] & ~PARITY_OVERFLOW_BIT;
  mainRegisters.af.f |= ((sum & 0x0100) >> 8);
  mainRegisters.af.f |= ((mainRegisters.af.a ^ sum ^ mainRegisters.bc.b) & HALF_CARRY_BIT);
  mainRegisters.af.f |= (((mainRegisters.af.a ^ mainRegisters.bc.b ^ 0x80) & (mainRegisters.bc.b ^ sum) & 0x80) >> 5);
  mainRegisters.af.a = sum & 0xFF;
}

void Z80::ADD_A_C(void)
{
  TRACE("ADD_A_C");
  uint16_t sum = mainRegisters.af.a + mainRegisters.bc.c;
  mainRegisters.af.f = flagTable[(sum & 0xFF)] & ~PARITY_OVERFLOW_BIT;
  mainRegisters.af.f |= ((sum & 0x0100) >> 8);
  mainRegisters.af.f |= ((mainRegisters.af.a ^ sum ^ mainRegisters.bc.c) & HALF_CARRY_BIT);
  mainRegisters.af.f |= (((mainRegisters.af.a ^ mainRegisters.bc.c ^ 0x80) & (mainRegisters.bc.c ^ sum) & 0x80) >> 5);
  mainRegisters.af.a = sum & 0xFF;
}

void Z80::ADD_A_D(void)
{
  TRACE("ADD_A_D");
  uint16_t sum = mainRegisters.af.a + mainRegisters.de.d;
  mainRegisters.af.f = flagTable[(sum & 0xFF)] & ~PARITY_OVERFLOW_BIT;
  mainRegisters.af.f |= ((sum & 0x0100) >> 8);
  mainRegisters.af.f |= ((mainRegisters.af.a ^ sum ^ mainRegisters.de.d) & HALF_CARRY_BIT);
  mainRegisters.af.f |= (((mainRegisters.af.a ^ mainRegisters.de.d ^ 0x80) & (mainRegisters.de.d ^ sum) & 0x80) >> 5);
  mainRegisters.af.a = sum & 0xFF;
}

void Z80::ADD_A_E(void)
{
  TRACE("ADD_A_E");
  uint16_t sum = mainRegisters.af.a + mainRegisters.de.e;
  mainRegisters.af.f = flagTable[(sum & 0xFF)] & ~PARITY_OVERFLOW_BIT;
  mainRegisters.af.f |= ((sum & 0x0100) >> 8);
  mainRegisters.af.f |= ((mainRegisters.af.a ^ sum ^ mainRegisters.de.e) & HALF_CARRY_BIT);
  mainRegisters.af.f |= (((mainRegisters.af.a ^ mainRegisters.de.e ^ 0x80) & (mainRegisters.de.e ^ sum) & 0x80) >> 5);
  mainRegisters.af.a = sum & 0xFF;
}

void Z80::ADD_A_H(void)
{
  TRACE("ADD_A_H");
  uint16_t sum = mainRegisters.af.a + mainRegisters.hl.h;
  mainRegisters.af.f = flagTable[(sum & 0xFF)] & ~PARITY_OVERFLOW_BIT;
  mainRegisters.af.f |= ((sum & 0x0100) >> 8);
  mainRegisters.af.f |= ((mainRegisters.af.a ^ sum ^ mainRegisters.hl.h) & HALF_CARRY_BIT);
  mainRegisters.af.f |= (((mainRegisters.af.a ^ mainRegisters.hl.h ^ 0x80) & (mainRegisters.hl.h ^ sum) & 0x80) >> 5);
  mainRegisters.af.a = sum & 0xFF;
}

void Z80::ADD_A_L(void)
{
  TRACE("ADD_A_L");
  uint16_t sum = mainRegisters.af.a + mainRegisters.hl.l;
  mainRegisters.af.f = flagTable[(sum & 0xFF)] & ~PARITY_OVERFLOW_BIT;
  mainRegisters.af.f |= ((sum & 0x0100) >> 8);
  mainRegisters.af.f |= ((mainRegisters.af.a ^ sum ^ mainRegisters.hl.l) & HALF_CARRY_BIT);
  mainRegisters.af.f |= (((mainRegisters.af.a ^ mainRegisters.hl.l ^ 0x80) & (mainRegisters.hl.l ^ sum) & 0x80) >> 5);
  mainRegisters.af.a = sum & 0xFF;
}

void Z80::ADD_A_iHL(void)
{
  TRACE("ADD_A_iHL");
  uint8_t iHL = memory->ReadByte(mainRegisters.hl.hl);
  uint16_t sum = mainRegisters.af.a + iHL;
  mainRegisters.af.f = flagTable[(sum & 0xFF)] & ~PARITY_OVERFLOW_BIT;
  mainRegisters.af.f |= ((sum & 0x0100) >> 8);
  mainRegisters.af.f |= ((mainRegisters.af.a ^ sum ^ iHL) & HALF_CARRY_BIT);
  mainRegisters.af.f |= (((mainRegisters.af.a ^ iHL ^ 0x80) & (iHL ^ sum) & 0x80) >> 5);
  mainRegisters.af.a = sum & 0xFF;
}

void Z80::ADD_A_A(void)
{
  TRACE("ADD_A_A");
  uint16_t sum = mainRegisters.af.a + mainRegisters.af.a;
  mainRegisters.af.f = flagTable[(sum & 0xFF)] & ~PARITY_OVERFLOW_BIT;
  mainRegisters.af.f |= ((sum & 0x0100) >> 8);
  mainRegisters.af.f |= ((mainRegisters.af.a ^ sum ^ mainRegisters.af.a) & HALF_CARRY_BIT);
  mainRegisters.af.f |= (((mainRegisters.af.a ^ mainRegisters.af.a ^ 0x80) & (mainRegisters.af.a ^ sum) & 0x80) >> 5);
  mainRegisters.af.a = sum & 0xFF;
}

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

void Z80::AND_B(void)
{
  TRACE("AND_B");
  mainRegisters.af.a &= mainRegisters.bc.b;
  mainRegisters.af.f = flagTable[mainRegisters.af.a] | HALF_CARRY_BIT;
}

void Z80::AND_C(void)
{
  TRACE("AND_C");
  mainRegisters.af.a &= mainRegisters.bc.c;
  mainRegisters.af.f = flagTable[mainRegisters.af.a] | HALF_CARRY_BIT;
}

void Z80::AND_D(void)
{
  TRACE("AND_D");
  mainRegisters.af.a &= mainRegisters.de.d;
  mainRegisters.af.f = flagTable[mainRegisters.af.a] | HALF_CARRY_BIT;
}

void Z80::AND_E(void)
{
  TRACE("AND_E");
  mainRegisters.af.a &= mainRegisters.de.e;
  mainRegisters.af.f = flagTable[mainRegisters.af.a] | HALF_CARRY_BIT;
}

void Z80::AND_H(void)
{
  TRACE("AND_H");
  mainRegisters.af.a &= mainRegisters.hl.h;
  mainRegisters.af.f = flagTable[mainRegisters.af.a] | HALF_CARRY_BIT;
}

void Z80::AND_L(void)
{
  TRACE("AND_L");
  mainRegisters.af.a &= mainRegisters.hl.l;
  mainRegisters.af.f = flagTable[mainRegisters.af.a] | HALF_CARRY_BIT;
}

void Z80::AND_iHL(void)
{
  TRACE("AND_iHL");
  mainRegisters.af.a &= memory->ReadByte(mainRegisters.hl.hl);
  mainRegisters.af.f = flagTable[mainRegisters.af.a] | HALF_CARRY_BIT;
}

void Z80::AND_A(void)
{
  TRACE("AND_A");
  mainRegisters.af.a &= mainRegisters.af.a;
  mainRegisters.af.f = flagTable[mainRegisters.af.a] | HALF_CARRY_BIT;
}

void Z80::XOR_B(void)
{
  TRACE("XOR_B");
  mainRegisters.af.a ^= mainRegisters.bc.b;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::XOR_C(void)
{
  TRACE("XOR_C");
  mainRegisters.af.a ^= mainRegisters.bc.c;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::XOR_D(void)
{
  TRACE("XOR_D");
  mainRegisters.af.a ^= mainRegisters.de.d;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::XOR_E(void)
{
  TRACE("XOR_E");
  mainRegisters.af.a ^= mainRegisters.de.e;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::XOR_H(void)
{
  TRACE("XOR_H");
  mainRegisters.af.a ^= mainRegisters.hl.h;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::XOR_L(void)
{
  TRACE("XOR_L");
  mainRegisters.af.a ^= mainRegisters.hl.l;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::XOR_iHL(void)
{
  TRACE("XOR_iHL");
  mainRegisters.af.a ^= memory->ReadByte(mainRegisters.hl.hl);
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}
void Z80::XOR_A(void)
{
  TRACE("XOR_A");
  mainRegisters.af.a ^= mainRegisters.af.a;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::OR_B(void)
{
  TRACE("OR_B");
  mainRegisters.af.a |= mainRegisters.bc.b;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::OR_C(void)
{
  TRACE("OR_C");
  mainRegisters.af.a |= mainRegisters.bc.c;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::OR_D(void)
{
  TRACE("OR_D");
  mainRegisters.af.a |= mainRegisters.de.d;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::OR_E(void)
{
  TRACE("OR_E");
  mainRegisters.af.a |= mainRegisters.de.e;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::OR_H(void)
{
  TRACE("OR_H");
  mainRegisters.af.a |= mainRegisters.hl.h;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::OR_L(void)
{
  TRACE("OR_L");
  mainRegisters.af.a |= mainRegisters.hl.l;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::OR_iHL(void)
{
  TRACE("OR_iHL");
  mainRegisters.af.a |= memory->ReadByte(mainRegisters.hl.hl);
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::OR_A(void)
{
  TRACE("OR_A");
  mainRegisters.af.a |= mainRegisters.af.a;
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::CP_B(void){}
void Z80::CP_C(void){}
void Z80::CP_D(void){}
void Z80::CP_E(void){}
void Z80::CP_H(void){}
void Z80::CP_L(void){}
void Z80::CP_iHL(void){}
void Z80::CP_A(void){}
void Z80::RET_NZ(void){}

void Z80::POP_BC(void)
{

	TRACE("POP_BC"); 

	mainRegisters.bc.c = memory->ReadByte(indexRegisters.sp++);
	mainRegisters.bc.b = memory->ReadByte(indexRegisters.sp++);
}

void Z80::JP_NZ_word(void)
{

	TRACE("JP_NZ_word"); 

	if(mainRegisters.af.f & ZERO_BIT)
	{
		PC += 2;
	}
	else
	{
		uint16_t address = memory->ReadByte(PC++);
		address |= (memory->ReadByte(PC++) << 8);
		PC = address;
	}
}

void Z80::JP_word(void)
{

	TRACE("JP_word"); 

	uint16_t address = memory->ReadByte(PC++);
	address |= (memory->ReadByte(PC++) << 8);
	PC = address;
}

void Z80::CALL_NZ_word(void){}

void Z80::PUSH_BC(void)
{

	TRACE("PUSH_BC"); 

	memory->WriteByte(--indexRegisters.sp, mainRegisters.bc.b);
	memory->WriteByte(--indexRegisters.sp, mainRegisters.bc.c);
}

void Z80::ADD_A_byte(void)
{
  TRACE("ADD_A_byte");
  uint8_t byte = memory->ReadByte(PC++);
  uint16_t sum = mainRegisters.af.a + byte;
  mainRegisters.af.f = flagTable[(sum & 0xFF)] & ~PARITY_OVERFLOW_BIT;
  mainRegisters.af.f |= ((sum & 0x0100) >> 8);
  mainRegisters.af.f |= ((mainRegisters.af.a ^ sum ^ byte) & HALF_CARRY_BIT);
  mainRegisters.af.f |= (((mainRegisters.af.a ^ byte ^ 0x80) & (byte ^ sum) & 0x80) >> 5);
  mainRegisters.af.a = sum & 0xFF;
}

void Z80::RST_00(void)
{
  TRACE("RST_00");
  memory->WriteByte(--indexRegisters.sp, ((PC & 0xFF00) >> 8));
  memory->WriteByte(--indexRegisters.sp, PC & 0xFF);
  PC = 0x0000;
}

void Z80::RET_Z(void){}
void Z80::RET(void){}

void Z80::JP_Z_word(void)
{

	TRACE("JP_Z_word"); 

	if(mainRegisters.af.f & ZERO_BIT)
	{
		uint16_t address = memory->ReadByte(PC++);
		address |= (memory->ReadByte(PC++) << 8);
		PC = address;
	}
	else
	{
		PC += 2;
	}
}

void Z80::CB(void)
{
  TRACE("CB");
  uint8_t instruction = memory->ReadByte(PC++);
  (this->*CBOp[instruction])();
}

void Z80::CALL_Z_word(void){}
void Z80::CALL_word(void){}
void Z80::ADC_A_byte(void){}

void Z80::RST_08(void)
{
  TRACE("RST_08");
  memory->WriteByte(--indexRegisters.sp, ((PC & 0xFF00) >> 8));
  memory->WriteByte(--indexRegisters.sp, PC & 0xFF);
  PC = 0x0008;
}

void Z80::RET_NC(void){}

void Z80::POP_DE(void)
{

	TRACE("POP_DE"); 

	mainRegisters.de.e = memory->ReadByte(indexRegisters.sp++);
	mainRegisters.de.d = memory->ReadByte(indexRegisters.sp++);
}

void Z80::JP_NC_word(void)
{

	TRACE("JP_NC_word"); 

	if(mainRegisters.af.f & CARRY_BIT)
	{
		PC += 2;
	}
	else
	{
		uint16_t address = memory->ReadByte(PC++);
		address |= (memory->ReadByte(PC++) << 8);
		PC = address;
	}
}

void Z80::OUT_iNN_A(void){}
void Z80::CALL_NC_word(void){}

void Z80::PUSH_DE(void)
{

	TRACE("PUSH_DE"); 

	memory->WriteByte(--indexRegisters.sp, mainRegisters.de.d);
	memory->WriteByte(--indexRegisters.sp, mainRegisters.de.e);
}

void Z80::SUB_A_byte(void){}

void Z80::RST_10(void)
{
  TRACE("RST_10");
  memory->WriteByte(--indexRegisters.sp, ((PC & 0xFF00) >> 8));
  memory->WriteByte(--indexRegisters.sp, PC & 0xFF);
  PC = 0x0010;
}
void Z80::RET_C(void){}

void Z80::EXX(void)
{
  TRACE("EXX");
  uint16_t swap = mainRegisters.bc.bc;
  mainRegisters.bc.bc = alternateRegisters.bc.bc;
  alternateRegisters.bc.bc = swap;
  swap = mainRegisters.de.de;
  mainRegisters.de.de = alternateRegisters.de.de;
  alternateRegisters.de.de = swap;
  swap = mainRegisters.hl.hl;
  mainRegisters.hl.hl = alternateRegisters.hl.hl;
  alternateRegisters.hl.hl = swap;
}

void Z80::JP_C_word(void)
{
	TRACE("JP_C_word"); 


	if(mainRegisters.af.f & CARRY_BIT)
	{
		uint16_t address = memory->ReadByte(PC++);
		address |= (memory->ReadByte(PC++) << 8);
		PC = address;
	}
	else
	{
		PC += 2;
	}
}

void Z80::IN_A_iNN(void){}
void Z80::CALL_C_word(void){}
void Z80::DD(void){}
void Z80::SBC_A_byte(void){}

void Z80::RST_18(void)
{
  TRACE("RST_18");
  memory->WriteByte(--indexRegisters.sp, ((PC & 0xFF00) >> 8));
  memory->WriteByte(--indexRegisters.sp, PC & 0xFF);
  PC = 0x0018;
}

void Z80::RET_PO(void){}

void Z80::POP_HL(void)
{

	TRACE("POP_HL"); 

	mainRegisters.hl.l = memory->ReadByte(indexRegisters.sp++);
	mainRegisters.hl.h = memory->ReadByte(indexRegisters.sp++);
}

void Z80::JP_PO_word(void)
{

	TRACE("JP_PO_word"); 

	if(mainRegisters.af.f & PARITY_OVERFLOW_BIT)
	{
		PC += 2;
	}
	else
	{
		uint16_t address = memory->ReadByte(PC++);
		address |= (memory->ReadByte(PC++) << 8);
		PC = address;
	}
}

void Z80::EX_iSP_HL(void)
{

	TRACE("EX_iSP_HL"); 

	uint8_t temp = memory->ReadByte(indexRegisters.sp);
	memory->WriteByte(indexRegisters.sp, mainRegisters.hl.l);
	mainRegisters.hl.l = temp;

	temp = memory->ReadByte(indexRegisters.sp + 1);
	memory->WriteByte(indexRegisters.sp + 1, mainRegisters.hl.h);
	mainRegisters.hl.h = temp;
}

void Z80::CALL_PO_word(void){}

void Z80::PUSH_HL(void)
{

	TRACE("PUSH_HL"); 

	memory->WriteByte(--indexRegisters.sp, mainRegisters.hl.h);
	memory->WriteByte(--indexRegisters.sp, mainRegisters.hl.l);
}

void Z80::AND_byte(void)
{
  TRACE("AND_byte");
  mainRegisters.af.a &= memory->ReadByte(PC++);
  mainRegisters.af.f = flagTable[mainRegisters.af.a] | HALF_CARRY_BIT;
}

void Z80::RST_20(void)
{
  TRACE("RST_20");
  memory->WriteByte(--indexRegisters.sp, ((PC & 0xFF00) >> 8));
  memory->WriteByte(--indexRegisters.sp, PC & 0xFF);
  PC = 0x0020;
}

void Z80::RET_PE(void){}

void Z80::JP_iHL(void)
{

	TRACE("JP_iHL"); 

	PC = mainRegisters.hl.hl;
}

void Z80::JP_PE_word(void)
{

	TRACE("JP_PE_word"); 

	if(mainRegisters.af.f & PARITY_OVERFLOW_BIT)
	{
		uint16_t address = memory->ReadByte(PC++);
		address |= (memory->ReadByte(PC++) << 8);
		PC = address;
	}
	else
	{
		PC += 2;
	}
}

void Z80::EX_DE_HL(void)
{
  TRACE("EX_DE_HL");
  uint16_t de = mainRegisters.de.de;
  mainRegisters.de.de = mainRegisters.hl.hl;
  mainRegisters.hl.hl = de;
}

void Z80::CALL_PE_word(void){}
void Z80::ED(void){}

void Z80::XOR_byte(void)
{
  TRACE("XOR_byte");
  mainRegisters.af.a ^= memory->ReadByte(PC++);
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::RST_28(void)
{
  TRACE("RST_28");
  memory->WriteByte(--indexRegisters.sp, ((PC & 0xFF00) >> 8));
  memory->WriteByte(--indexRegisters.sp, PC & 0xFF);
  PC = 0x0028;
}

void Z80::RET_P(void){}

void Z80::POP_AF(void)
{

	TRACE("POP_AF"); 

	mainRegisters.af.f = memory->ReadByte(indexRegisters.sp++);
	mainRegisters.af.a = memory->ReadByte(indexRegisters.sp++);
}

void Z80::JP_P_word(void)
{

	TRACE("JP_P_word"); 

	if(mainRegisters.af.f & SIGN_BIT)
	{
		PC += 2;
	}
	else
	{
		uint16_t address = memory->ReadByte(PC++);
		address |= (memory->ReadByte(PC) << 8);
		PC = address;
	}
}

void Z80::DI(void){}
void Z80::CALL_P_word(void){}

void Z80::PUSH_AF(void)
{

	TRACE("PUSH_AF"); 

	memory->WriteByte(--indexRegisters.sp, mainRegisters.af.a);
	memory->WriteByte(--indexRegisters.sp, mainRegisters.af.f);
}

void Z80::OR_byte(void)
{
  TRACE("OR_byte");
  mainRegisters.af.a |= memory->ReadByte(PC++);
  mainRegisters.af.f = flagTable[mainRegisters.af.a];
}

void Z80::RST_30(void)
{
  TRACE("RST_30");
  memory->WriteByte(--indexRegisters.sp, ((PC & 0xFF00) >> 8));
  memory->WriteByte(--indexRegisters.sp, PC & 0xFF);
  PC = 0x0030;
}

void Z80::RET_M(void){}

void Z80::LD_SP_HL(void)
{

	TRACE("LD_SP_HL"); 

	indexRegisters.sp = mainRegisters.hl.hl;
}

void Z80::JP_M_word(void)
{

	TRACE("JP_M_word"); 

	if(mainRegisters.af.f & CARRY_BIT)
	{
		uint16_t address = memory->ReadByte(PC++);
		address |= (memory->ReadByte(PC) << 8);
		PC = address;
	}
	else
	{
		PC += 2;
	}
}

void Z80::EI(void){}
void Z80::CALL_M_word(void){}
void Z80::FD(void){}
void Z80::CP_byte(void){}

void Z80::RST_38(void)
{
  TRACE("RST_38");
  memory->WriteByte(--indexRegisters.sp, ((PC & 0xFF00) >> 8));
  memory->WriteByte(--indexRegisters.sp, PC & 0xFF);
  PC = 0x0038;
}

void Z80::RLC_B(void){}
void Z80::RLC_C(void){}
void Z80::RLC_D(void){}
void Z80::RLC_E(void){}
void Z80::RLC_H(void){}
void Z80::RLC_L(void){}
void Z80::RLC_iHL(void){}
void Z80::RLC_A(void){}
void Z80::RRC_B(void){}
void Z80::RRC_C(void){}
void Z80::RRC_D(void){}
void Z80::RRC_E(void){}
void Z80::RRC_H(void){}
void Z80::RRC_L(void){}
void Z80::RRC_iHL(void){}
void Z80::RRC_A(void){}
void Z80::RL_B(void){}
void Z80::RL_C(void){}
void Z80::RL_D(void){}
void Z80::RL_E(void){}
void Z80::RL_H(void){}
void Z80::RL_L(void){}
void Z80::RL_iHL(void){}
void Z80::RL_A(void){}
void Z80::RR_B(void){}
void Z80::RR_C(void){}
void Z80::RR_D(void){}
void Z80::RR_E(void){}
void Z80::RR_H(void){}
void Z80::RR_L(void){}
void Z80::RR_iHL(void){}
void Z80::RR_A(void){}
void Z80::SLA_B(void){}
void Z80::SLA_C(void){}
void Z80::SLA_D(void){}
void Z80::SLA_E(void){}
void Z80::SLA_H(void){}
void Z80::SLA_L(void){}
void Z80::SLA_iHL(void){}
void Z80::SLA_A(void){}
void Z80::SRA_B(void){}
void Z80::SRA_C(void){}
void Z80::SRA_D(void){}
void Z80::SRA_E(void){}
void Z80::SRA_H(void){}
void Z80::SRA_L(void){}
void Z80::SRA_iHL(void){}
void Z80::SRA_A(void){}
void Z80::SLS_B(void){}
void Z80::SLS_C(void){}
void Z80::SLS_D(void){}
void Z80::SLS_E(void){}
void Z80::SLS_H(void){}
void Z80::SLS_L(void){}
void Z80::SLS_iHL(void){}
void Z80::SLS_A(void){}
void Z80::SRL_B(void){}
void Z80::SRL_C(void){}
void Z80::SRL_D(void){}
void Z80::SRL_E(void){}
void Z80::SRL_H(void){}
void Z80::SRL_L(void){}
void Z80::SRL_iHL(void){}
void Z80::SRL_A(void){}
void Z80::BIT_0_B(void){}
void Z80::BIT_0_C(void){}
void Z80::BIT_0_D(void){}
void Z80::BIT_0_E(void){}
void Z80::BIT_0_H(void){}
void Z80::BIT_0_L(void){}
void Z80::BIT_0_iHL(void){}
void Z80::BIT_0_A(void){}
void Z80::BIT_1_B(void){}
void Z80::BIT_1_C(void){}
void Z80::BIT_1_D(void){}
void Z80::BIT_1_E(void){}
void Z80::BIT_1_H(void){}
void Z80::BIT_1_L(void){}
void Z80::BIT_1_iHL(void){}
void Z80::BIT_1_A(void){}
void Z80::BIT_2_B(void){}
void Z80::BIT_2_C(void){}
void Z80::BIT_2_D(void){}
void Z80::BIT_2_E(void){}
void Z80::BIT_2_H(void){}
void Z80::BIT_2_L(void){}
void Z80::BIT_2_iHL(void){}
void Z80::BIT_2_A(void){}
void Z80::BIT_3_B(void){}
void Z80::BIT_3_C(void){}
void Z80::BIT_3_D(void){}
void Z80::BIT_3_E(void){}
void Z80::BIT_3_H(void){}
void Z80::BIT_3_L(void){}
void Z80::BIT_3_iHL(void){}
void Z80::BIT_3_A(void){}
void Z80::BIT_4_B(void){}
void Z80::BIT_4_C(void){}
void Z80::BIT_4_D(void){}
void Z80::BIT_4_E(void){}
void Z80::BIT_4_H(void){}
void Z80::BIT_4_L(void){}
void Z80::BIT_4_iHL(void){}
void Z80::BIT_4_A(void){}
void Z80::BIT_5_B(void){}
void Z80::BIT_5_C(void){}
void Z80::BIT_5_D(void){}
void Z80::BIT_5_E(void){}
void Z80::BIT_5_H(void){}
void Z80::BIT_5_L(void){}
void Z80::BIT_5_iHL(void){}
void Z80::BIT_5_A(void){}
void Z80::BIT_6_B(void){}
void Z80::BIT_6_C(void){}
void Z80::BIT_6_D(void){}
void Z80::BIT_6_E(void){}
void Z80::BIT_6_H(void){}
void Z80::BIT_6_L(void){}
void Z80::BIT_6_iHL(void){}
void Z80::BIT_6_A(void){}
void Z80::BIT_7_B(void){}
void Z80::BIT_7_C(void){}
void Z80::BIT_7_D(void){}
void Z80::BIT_7_E(void){}
void Z80::BIT_7_H(void){}
void Z80::BIT_7_L(void){}
void Z80::BIT_7_iHL(void){}
void Z80::BIT_7_A(void){}

void Z80::RES_0_B(void)
{
  TRACE("RES_0_B");
  mainRegisters.bc.b &= ~BIT_0;
}

void Z80::RES_0_C(void)
{
  TRACE("RES_0_C");
  mainRegisters.bc.c &= ~BIT_0;
}

void Z80::RES_0_D(void)
{
  TRACE("RES_0_D");
  mainRegisters.de.d &= ~BIT_0;
}

void Z80::RES_0_E(void)
{
  TRACE("RES_0_E");
  mainRegisters.de.e &= ~BIT_0;
}

void Z80::RES_0_H(void)
{
  TRACE("RES_0_H");
  mainRegisters.hl.h &= ~BIT_0;
}

void Z80::RES_0_L(void)
{
  TRACE("RES_0_L");
  mainRegisters.hl.l &= ~BIT_0;
}

void Z80::RES_0_iHL(void)
{
  TRACE("RES_0_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value &= ~BIT_0;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::RES_0_A(void)
{
  TRACE("RES_0_A");
  mainRegisters.af.a &= ~BIT_0;
}

void Z80::RES_1_B(void)
{
  TRACE("RES_1_B");
  mainRegisters.bc.b &= ~BIT_1;
}

void Z80::RES_1_C(void)
{
  TRACE("RES_1_C");
  mainRegisters.bc.c &= ~BIT_1;
}

void Z80::RES_1_D(void)
{
  TRACE("RES_1_D");
  mainRegisters.de.d &= ~BIT_1;
}

void Z80::RES_1_E(void)
{
  TRACE("RES_1_E");
  mainRegisters.de.e &= ~BIT_1;
}

void Z80::RES_1_H(void)
{
  TRACE("RES_1_H");
  mainRegisters.hl.h &= ~BIT_1;
}

void Z80::RES_1_L(void)
{
  TRACE("RES_1_L");
  mainRegisters.hl.l &= ~BIT_1;
}

void Z80::RES_1_iHL(void)
{
  TRACE("RES_1_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value &= ~BIT_1;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::RES_1_A(void)
{
  TRACE("RES_1_A");
  mainRegisters.af.a &= ~BIT_1;
}

void Z80::RES_2_B(void)
{
  TRACE("RES_2_B");
  mainRegisters.bc.b &= ~BIT_2;
}

void Z80::RES_2_C(void)
{
  TRACE("RES_2_C");
  mainRegisters.bc.c &= ~BIT_2;
}

void Z80::RES_2_D(void)
{
  TRACE("RES_2_D");
  mainRegisters.de.d &= ~BIT_2;
}

void Z80::RES_2_E(void)
{
  TRACE("RES_2_E");
  mainRegisters.de.e &= ~BIT_2;
}

void Z80::RES_2_H(void)
{
  TRACE("RES_2_H");
  mainRegisters.hl.h &= ~BIT_2;
}

void Z80::RES_2_L(void)
{
  TRACE("RES_2_L");
  mainRegisters.hl.l &= ~BIT_2;
}

void Z80::RES_2_iHL(void)
{
  TRACE("RES_2_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value &= ~BIT_2;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::RES_2_A(void)
{
  TRACE("RES_2_A");
  mainRegisters.af.a &= ~BIT_2;
}

void Z80::RES_3_B(void)
{
  TRACE("RES_3_B");
  mainRegisters.bc.b &= ~BIT_3;
}

void Z80::RES_3_C(void)
{
  TRACE("RES_3_C");
  mainRegisters.bc.c &= ~BIT_3;
}

void Z80::RES_3_D(void)
{
  TRACE("RES_3_D");
  mainRegisters.de.d &= ~BIT_3;
}

void Z80::RES_3_E(void)
{
  TRACE("RES_3_E");
  mainRegisters.de.e &= ~BIT_3;
}

void Z80::RES_3_H(void)
{
  TRACE("RES_3_H");
  mainRegisters.hl.h &= ~BIT_3;
}

void Z80::RES_3_L(void)
{
  TRACE("RES_3_L");
  mainRegisters.hl.l &= ~BIT_3;
}

void Z80::RES_3_iHL(void)
{
  TRACE("RES_3_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value &= ~BIT_3;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::RES_3_A(void)
{
  TRACE("RES_3_A");
  mainRegisters.af.a &= ~BIT_3;
}

void Z80::RES_4_B(void)
{
  TRACE("RES_4_B");
  mainRegisters.bc.b &= ~BIT_4;
}

void Z80::RES_4_C(void)
{
  TRACE("RES_4_C");
  mainRegisters.bc.c &= ~BIT_4;
}

void Z80::RES_4_D(void)
{
  TRACE("RES_4_D");
  mainRegisters.de.d &= ~BIT_4;
}

void Z80::RES_4_E(void)
{
  TRACE("RES_4_E");
  mainRegisters.de.e &= ~BIT_4;
}

void Z80::RES_4_H(void)
{
  TRACE("RES_4_H");
  mainRegisters.hl.h &= ~BIT_4;
}

void Z80::RES_4_L(void)
{
  TRACE("RES_4_L");
  mainRegisters.hl.l &= ~BIT_4;
}

void Z80::RES_4_iHL(void)
{
  TRACE("RES_4_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value &= ~BIT_4;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::RES_4_A(void)
{
  TRACE("RES_4_A");
  mainRegisters.af.a &= ~BIT_4;
}

void Z80::RES_5_B(void)
{
  TRACE("RES_5_B");
  mainRegisters.bc.b &= ~BIT_5;
}

void Z80::RES_5_C(void)
{
  TRACE("RES_5_C");
  mainRegisters.bc.c &= ~BIT_5;
}

void Z80::RES_5_D(void)
{
  TRACE("RES_5_D");
  mainRegisters.de.d &= ~BIT_5;
}

void Z80::RES_5_E(void)
{
  TRACE("RES_5_E");
  mainRegisters.de.e &= ~BIT_5;
}

void Z80::RES_5_H(void)
{
  TRACE("RES_5_H");
  mainRegisters.hl.h &= ~BIT_5;
}

void Z80::RES_5_L(void)
{
  TRACE("RES_5_L");
  mainRegisters.hl.l &= ~BIT_5;
}

void Z80::RES_5_iHL(void)
{
  TRACE("RES_5_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value &= ~BIT_5;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::RES_5_A(void)
{
  TRACE("RES_5_A");
  mainRegisters.af.a &= ~BIT_5;
}

void Z80::RES_6_B(void)
{
  TRACE("RES_6_B");
  mainRegisters.bc.b &= ~BIT_6;
}

void Z80::RES_6_C(void)
{
  TRACE("RES_6_C");
  mainRegisters.bc.c &= ~BIT_6;
}

void Z80::RES_6_D(void)
{
  TRACE("RES_6_D");
  mainRegisters.de.d &= ~BIT_6;
}

void Z80::RES_6_E(void)
{
  TRACE("RES_6_E");
  mainRegisters.de.e &= ~BIT_6;
}

void Z80::RES_6_H(void)
{
  TRACE("RES_6_H");
  mainRegisters.hl.h &= ~BIT_6;
}

void Z80::RES_6_L(void)
{
  TRACE("RES_6_L");
  mainRegisters.hl.l &= ~BIT_6;
}

void Z80::RES_6_iHL(void)
{
  TRACE("RES_6_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value &= ~BIT_6;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::RES_6_A(void)
{
  TRACE("RES_6_A");
  mainRegisters.af.a &= ~BIT_6;
}

void Z80::RES_7_B(void)
{
  TRACE("RES_7_B");
  mainRegisters.bc.b &= ~BIT_7;
}

void Z80::RES_7_C(void)
{
  TRACE("RES_7_C");
  mainRegisters.bc.c &= ~BIT_7;
}

void Z80::RES_7_D(void)
{
  TRACE("RES_7_D");
  mainRegisters.de.d &= ~BIT_7;
}

void Z80::RES_7_E(void)
{
  TRACE("RES_7_E");
  mainRegisters.de.e &= ~BIT_7;
}

void Z80::RES_7_H(void)
{
  TRACE("RES_7_H");
  mainRegisters.hl.h &= ~BIT_7;
}

void Z80::RES_7_L(void)
{
  TRACE("RES_7_L");
  mainRegisters.hl.l &= ~BIT_7;
}

void Z80::RES_7_iHL(void)
{
  TRACE("RES_7_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value &= ~BIT_7;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::RES_7_A(void)
{
  TRACE("RES_7_A");
  mainRegisters.af.a &= ~BIT_7;
}

void Z80::SET_0_B(void)
{
  TRACE("SET_0_B");
  mainRegisters.bc.b |= BIT_0;
}

void Z80::SET_0_C(void)
{
  TRACE("SET_0_C");
  mainRegisters.bc.c |= BIT_0;
}

void Z80::SET_0_D(void)
{
  TRACE("SET_0_D");
  mainRegisters.de.d |= BIT_0;
}

void Z80::SET_0_E(void)
{
  TRACE("SET_0_E");
  mainRegisters.de.e |= BIT_0;
}

void Z80::SET_0_H(void)
{
  TRACE("SET_0_H");
  mainRegisters.hl.h |= BIT_0;
}

void Z80::SET_0_L(void)
{
  TRACE("SET_0_L");
  mainRegisters.hl.l |= BIT_0;
}

void Z80::SET_0_iHL(void)
{
  TRACE("SET_0_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value |= BIT_0;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::SET_0_A(void)
{
  TRACE("SET_0_A");
  mainRegisters.af.a |= BIT_0;
}

void Z80::SET_1_B(void)
{
  TRACE("SET_1_B");
  mainRegisters.bc.b |= BIT_1;
}

void Z80::SET_1_C(void)
{
  TRACE("SET_1_C");
  mainRegisters.bc.c |= BIT_1;
}

void Z80::SET_1_D(void)
{
  TRACE("SET_1_D");
  mainRegisters.de.d |= BIT_1;
}

void Z80::SET_1_E(void)
{
  TRACE("SET_1_E");
  mainRegisters.de.e |= BIT_1;
}

void Z80::SET_1_H(void)
{
  TRACE("SET_1_H");
  mainRegisters.hl.h |= BIT_1;
}

void Z80::SET_1_L(void)
{
  TRACE("SET_1_L");
  mainRegisters.hl.l |= BIT_1;
}

void Z80::SET_1_iHL(void)
{
  TRACE("SET_1_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value |= BIT_1;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::SET_1_A(void)
{
  TRACE("SET_1_A");
  mainRegisters.af.a |= BIT_1;
}

void Z80::SET_2_B(void)
{
  TRACE("SET_2_B");
  mainRegisters.bc.b |= BIT_2;
}

void Z80::SET_2_C(void)
{
  TRACE("SET_2_C");
  mainRegisters.bc.c |= BIT_2;
}

void Z80::SET_2_D(void)
{
  TRACE("SET_2_D");
  mainRegisters.de.d |= BIT_2;
}

void Z80::SET_2_E(void)
{
  TRACE("SET_2_E");
  mainRegisters.de.e |= BIT_2;
}

void Z80::SET_2_H(void)
{
  TRACE("SET_2_H");
  mainRegisters.hl.h |= BIT_2;
}

void Z80::SET_2_L(void)
{
  TRACE("SET_2_L");
  mainRegisters.hl.l |= BIT_2;
}

void Z80::SET_2_iHL(void)
{
  TRACE("SET_2_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value |= BIT_2;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::SET_2_A(void)
{
  TRACE("SET_2_A");
  mainRegisters.af.a |= BIT_2;
}

void Z80::SET_3_B(void)
{
  TRACE("SET_3_B");
  mainRegisters.bc.b |= BIT_3;
}

void Z80::SET_3_C(void)
{
  TRACE("SET_3_C");
  mainRegisters.bc.c |= BIT_3;
}

void Z80::SET_3_D(void)
{
  TRACE("SET_3_D");
  mainRegisters.de.d |= BIT_3;
}

void Z80::SET_3_E(void)
{
  TRACE("SET_3_E");
  mainRegisters.de.e |= BIT_3;
}

void Z80::SET_3_H(void)
{
  TRACE("SET_3_H");
  mainRegisters.hl.h |= BIT_3;
}

void Z80::SET_3_L(void)
{
  TRACE("SET_3_L");
  mainRegisters.hl.l |= BIT_3;
}

void Z80::SET_3_iHL(void)
{
  TRACE("SET_3_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value |= BIT_3;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::SET_3_A(void)
{
  TRACE("SET_3_A");
  mainRegisters.af.a |= BIT_3;
}

void Z80::SET_4_B(void)
{
  TRACE("SET_4_B");
  mainRegisters.bc.b |= BIT_4;
}

void Z80::SET_4_C(void)
{
  TRACE("SET_4_C");
  mainRegisters.bc.c |= BIT_4;
}

void Z80::SET_4_D(void)
{
  TRACE("SET_4_D");
  mainRegisters.de.d |= BIT_4;
}

void Z80::SET_4_E(void)
{
  TRACE("SET_4_E");
  mainRegisters.de.e |= BIT_4;
}

void Z80::SET_4_H(void)
{
  TRACE("SET_4_H");
  mainRegisters.hl.h |= BIT_4;
}

void Z80::SET_4_L(void)
{
  TRACE("SET_4_L");
  mainRegisters.hl.l |= BIT_4;
}

void Z80::SET_4_iHL(void)
{
  TRACE("SET_4_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value |= BIT_4;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::SET_4_A(void)
{
  TRACE("SET_4_A");
  mainRegisters.af.a |= BIT_4;
}

void Z80::SET_5_B(void)
{
  TRACE("SET_5_B");
  mainRegisters.bc.b |= BIT_5;
}

void Z80::SET_5_C(void)
{
  TRACE("SET_5_C");
  mainRegisters.bc.c |= BIT_5;
}

void Z80::SET_5_D(void)
{
  TRACE("SET_5_D");
  mainRegisters.de.d |= BIT_5;
}

void Z80::SET_5_E(void)
{
  TRACE("SET_5_E");
  mainRegisters.de.e |= BIT_5;
}

void Z80::SET_5_H(void)
{
  TRACE("SET_5_H");
  mainRegisters.hl.h |= BIT_5;
}

void Z80::SET_5_L(void)
{
  TRACE("SET_5_L");
  mainRegisters.hl.l |= BIT_5;
}

void Z80::SET_5_iHL(void)
{
  TRACE("SET_5_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value |= BIT_5;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::SET_5_A(void)
{
  TRACE("SET_5_A");
  mainRegisters.af.a |= BIT_5;
}

void Z80::SET_6_B(void)
{
  TRACE("SET_6_B");
  mainRegisters.bc.b |= BIT_6;
}

void Z80::SET_6_C(void)
{
  TRACE("SET_6_C");
  mainRegisters.bc.c |= BIT_6;
}

void Z80::SET_6_D(void)
{
  TRACE("SET_6_D");
  mainRegisters.de.d |= BIT_6;
}

void Z80::SET_6_E(void)
{
  TRACE("SET_6_E");
  mainRegisters.de.e |= BIT_6;
}

void Z80::SET_6_H(void)
{
  TRACE("SET_6_H");
  mainRegisters.hl.h |= BIT_6;
}

void Z80::SET_6_L(void)
{
  TRACE("SET_6_L");
  mainRegisters.hl.l |= BIT_6;
}

void Z80::SET_6_iHL(void)
{
  TRACE("SET_6_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value |= BIT_6;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::SET_6_A(void)
{
  TRACE("SET_6_A");
  mainRegisters.af.a |= BIT_6;
}

void Z80::SET_7_B(void)
{
  TRACE("SET_7_B");
  mainRegisters.bc.b |= BIT_7;
}

void Z80::SET_7_C(void)
{
  TRACE("SET_7_C");
  mainRegisters.bc.c |= BIT_7;
}

void Z80::SET_7_D(void)
{
  TRACE("SET_7_D");
  mainRegisters.de.d |= BIT_7;
}

void Z80::SET_7_E(void)
{
  TRACE("SET_7_E");
  mainRegisters.de.e |= BIT_7;
}

void Z80::SET_7_H(void)
{
  TRACE("SET_7_H");
  mainRegisters.hl.h |= BIT_7;
}

void Z80::SET_7_L(void)
{
  TRACE("SET_7_L");
  mainRegisters.hl.l |= BIT_7;
}

void Z80::SET_7_iHL(void)
{
  TRACE("SET_7_iHL");
  uint8_t value = memory->ReadByte(mainRegisters.hl.hl);
  value |= BIT_7;
  memory->WriteByte(mainRegisters.hl.hl, value);
}

void Z80::SET_7_A(void)
{
  TRACE("SET_7_A");
  mainRegisters.af.a |= BIT_7;
}
