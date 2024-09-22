#ifndef __ULPDEBUG_H__
#define __ULPDEBUG_H__

#if CONFIG_IDF_TARGET_ESP32
#include "esp32/ulp.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "soc/rtc_cntl_reg.h"
#include "esp32s3/ulp.h"
#include "esp32s3/rom/ets_sys.h"
#endif

const char* ulpRegStr[] = {
  "R0",
  "R1",
  "R2",
  "R3",
};

const char* ulpPeriphStr[] = {
  "RTC_CNTL",
  "RTC_IO",
  "SARADC",
};

const char* ulpSarStr[] = {
  "RTC_CNTL",
  "RTC_IO",
  "SARADC",
};

const char* ulpSignStr[] = {
  "+",
  "-",
};

const char* ulpCmpStr[] = {
  "LT",
  "G",
  "E"
};
const char* ulpCmpSStr[] = {
  "L",
  "GE",
  "LE"
};
const char* ulpCmp2Str[] = {
  "R0 < ",
  "R0 > ",
  "R0 == "
};
const char* ulpCmp2SStr[] = {
  "SR < ",
  "SR >= ",
  "SR <= "
};

void ulpDisassemblerDump(int pos) {
  ulp_insn_t *prog = (ulp_insn_t*)&RTC_SLOW_MEM[pos];

  if(prog->halt.opcode==0 && RTC_SLOW_MEM[pos] !=0){
    ets_printf("%04X : %08X DATA %5d\t\t\t\t// ST ADDR:0x%04X\n", pos, RTC_SLOW_MEM[pos], (int16_t)(RTC_SLOW_MEM[pos] & 0xFFFF), RTC_SLOW_MEM[pos] >> 21);
    return;
  }

  ets_printf("%04X : %08X PROG ", pos, RTC_SLOW_MEM[pos]);

  switch( prog->halt.opcode ){
    case 0:               // NOP
                          ets_printf("NOP\t\t\t\t// NOP");
                          break;

    case OPCODE_WR_REG:   // REG_WR
                          ets_printf("REG_WR 0x%04X, %3d, %3d, %3d\t// %s[%d:%d] = %d", prog->wr_reg.addr*sizeof(uint32_t), prog->wr_reg.high, prog->wr_reg.low, prog->wr_reg.data, ulpPeriphStr[prog->wr_reg.periph_sel], prog->wr_reg.high, prog->wr_reg.low, prog->wr_reg.data );
                          break;

    case OPCODE_RD_REG:   // REG_RD
                          ets_printf("REG_RD 0x%04X, %3d, %3d\t// R0 = %s[%d:%d]", prog->rd_reg.addr, prog->rd_reg.high, prog->rd_reg.low, ulpPeriphStr[prog->rd_reg.periph_sel], prog->rd_reg.high, prog->rd_reg.low );
                          break;
    
    case OPCODE_I2C:      // I2C
                          if( prog->i2c.rw == 0 ){
                            ets_printf("I2C_RD 0x%02X, %3d, %3d, 0x%02X", prog->i2c.i2c_addr, prog->i2c.high_bits, prog->i2c.low_bits, prog->i2c.i2c_sel);
                          } else {
                            ets_printf("I2C_WR 0x%02X, 0x%02X, %3d, %3d, 0x%02X", prog->i2c.i2c_addr, prog->i2c.data, prog->i2c.high_bits, prog->i2c.low_bits, prog->i2c.i2c_sel);
                          }
                          break;
    
    case OPCODE_DELAY:    // WAIT
                          ets_printf("WAIT   %3d\t\t\t// Delay %d", prog->delay.cycles, prog->delay.cycles );
                          break;
    
    case OPCODE_ADC:      // ADC
                          ets_printf("ADC    %s, %s, %3d, %d", ulpPeriphStr[prog->adc.dreg], ulpSarStr[prog->adc.sar_sel], prog->adc.mux, prog->adc.cycles );
                          break;
    
    case OPCODE_ST:       // ST
                          ets_printf("ST     %s, %s, %3d\t\t\t// MEM[%s+%d] = %s", ulpRegStr[prog->st.dreg], ulpRegStr[prog->st.sreg], prog->st.offset, ulpRegStr[prog->st.sreg], prog->st.offset, ulpRegStr[prog->st.dreg] );
                          break;
    
    case OPCODE_ALU:      // ALU
                            // #define ALU_SEL_ADD 0           /*!< Addition */
                            // #define ALU_SEL_SUB 1           /*!< Subtraction */
                            // #define ALU_SEL_AND 2           /*!< Logical AND */
                            // #define ALU_SEL_OR  3           /*!< Logical OR */
                            // #define ALU_SEL_MOV 4           /*!< Copy value (immediate to destination register or source register to destination register */
                            // #define ALU_SEL_LSH 5           /*!< Shift left by given number of bits */
                            // #define ALU_SEL_RSH 6           /*!< Shift right by given number of bits */
                            // #define ALU_SEL_STAGE_INC 0     /*!< Increment stage count register */
                            // #define ALU_SEL_STAGE_DEC 1     /*!< Decrement stage count register */
                            // #define ALU_SEL_STAGE_RST 2     /*!< Reset stage count register */
                        if (prog->alu_reg.sub_opcode!=SUB_OPCODE_ALU_CNT) {
                          if(prog->alu_reg.sel == ALU_SEL_ADD){
                            // ADD
                            if(prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_REG){
                              // reg
                              ets_printf("ADD    %s, %s, %s\t\t\t// %s = %s + %s", ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg], ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg]);
                            }
                            if (prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_IMM) {
                              // imm
                              ets_printf("ADD    %s, %s, %3d\t\t\t// %s = %s + %d", ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm, ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm);
                            }
                          } else if(prog->alu_reg.sel == ALU_SEL_SUB){
                            // SUB
                            if(prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_REG){
                              // reg
                              ets_printf("SUB    %s, %s, %s\t\t\t// %s = %s - %s", ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg], ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg]);
                            }
                            if (prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_IMM) {
                              // imm
                              ets_printf("SUB    %s, %s, %3d\t\t\t// %s = %s - %d", ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm, ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm);
                            }
                          } else if(prog->alu_reg.sel == ALU_SEL_AND){
                            // AND
                            if(prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_REG){
                              // reg
                              ets_printf("AND    %s, %s, %s\t\t\t// %s = %s & %s", ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg], ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg]);
                            } 
                            if (prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_IMM) {
                              // imm
                              ets_printf("AND    %s, %s, %3d\t\t\t// %s = %s & %d", ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm, ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm);
                            }
                          } else if(prog->alu_reg.sel == ALU_SEL_OR){
                            // OR
                            if(prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_REG){
                              // reg
                              ets_printf("OR     %s, %s, %s\t\t\t// %s = %s | %s", ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg], ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg]);
                            }
                            if (prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_IMM) {
                              // imm
                              ets_printf("OR     %s, %s, %3d\t\t\t// %s = %s | %d", ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm, ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm);
                            }
                          } else if(prog->alu_reg.sel == ALU_SEL_LSH){
                            // LSH
                            if(prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_REG){
                              // reg
                              ets_printf("LSH    %s, %s, %s\t\t\t// %s = %s << %s", ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg], ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg]);
                            }
                            if (prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_IMM) {
                              // imm
                              ets_printf("LSH    %s, %s, %3d\t\t\t// %s = %s << %d", ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm, ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm);
                            }
                          } else if(prog->alu_reg.sel == ALU_SEL_RSH){
                            // RSH
                            if(prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_REG){
                              // reg
                              ets_printf("RSH    %s, %s, %s\t\t\t// %s = %s >> %s", ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg], ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.treg]);
                            }
                            if (prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_IMM) {
                              // imm
                              ets_printf("RSH    %s, %s, %3d\t\t\t// %s = %s >> %d", ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm, ulpRegStr[prog->alu_imm.dreg], ulpRegStr[prog->alu_imm.sreg], (int16_t)prog->alu_imm.imm);
                            }
                          } else if(prog->alu_reg.sel == ALU_SEL_MOV){
                            // MOVE
                            if(prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_REG){
                              // reg
                              ets_printf("MOVE   %s, %s\t\t\t// %s = %s", ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg], ulpRegStr[prog->alu_reg.dreg], ulpRegStr[prog->alu_reg.sreg]);
                            }
                            if (prog->alu_reg.sub_opcode == SUB_OPCODE_ALU_IMM) {
                              // imm
                              ets_printf("MOVE   %s, %3d\t\t\t// %s = %d", ulpRegStr[prog->alu_imm.dreg], (int16_t)prog->alu_imm.imm, ulpRegStr[prog->alu_imm.dreg], (int16_t)prog->alu_imm.imm);
                            }
                          } 
                        } else { 
                          if (prog->alu_reg.sel == ALU_SEL_STAGE_RST) {
                              ets_printf ("I_STAGE_RST %d",(int16_t)prog->alu_imm.imm);
                          } else if (prog->alu_reg.sel == ALU_SEL_STAGE_INC) {
                              ets_printf ("I_STAGE_INC +%d",(int16_t)prog->alu_imm.imm);
                          } else if (prog->alu_reg.sel == ALU_SEL_STAGE_DEC) {
                              ets_printf ("I_STAGE_DEC -%d",(int16_t)prog->alu_imm.imm);
                          }
                        }
                        break;
    case OPCODE_BRANCH:   // JUML
                          if(prog->b.sub_opcode == SUB_OPCODE_B){
                            // SUB_OPCODE_B(JUMP)
                            ets_printf("JUMPR  %s%3d, %3d, %s\t\t// IF %s%d THAN GOTO 0x%04X", ulpSignStr[prog->b.sign], prog->b.offset, prog->b.imm, ulpCmpStr[prog->b.cmp], ulpCmp2Str[prog->b.cmp], prog->b.imm, prog->b.sign == 0 ? pos + prog->b.offset : pos - prog->b.offset);
                          } else if (prog->b.sub_opcode == SUB_OPCODE_BS) {
                            ets_printf("JUMPS  %s%3d, %3d, %s\t\t// IF %s%d THAN GOTO 0x%04X", ulpSignStr[prog->b.sign], prog->b.offset, prog->b.imm, ulpCmpSStr[prog->b.cmp], ulpCmp2SStr[prog->b.cmp], prog->b.imm, prog->b.sign == 0 ? pos + prog->b.offset : pos - prog->b.offset);
                          } else if (prog->b.sub_opcode == SUB_OPCODE_BX) {
                            // SUB_OPCODE_BX(DIRECT JUMP)
                            if( prog->bx.type == BX_JUMP_TYPE_DIRECT ){
                              // BX_JUMP_TYPE_DIRECT
                              if( prog->bx.reg == 1 ){
                                // reg
                                ets_printf("JUMP   %s\t\t\t// GOTO %s", ulpRegStr[prog->bx.dreg], ulpRegStr[prog->bx.dreg]);
                              } else {
                                // imm
                                ets_printf("JUMP   0x%04X\t\t\t// GOTO 0x%04X", prog->bx.addr, prog->bx.addr);
                              }                              
                            } else if( prog->bx.type == BX_JUMP_TYPE_ZERO ) {
                              // EQ
                              if( prog->bx.reg == 1 ){
                                // reg
                                ets_printf("JUMP   %s, EQ", ulpRegStr[prog->bx.dreg]);
                              } else {
                                // imm
                                ets_printf("JUMP   0x%04X, EQ", prog->bx.addr);
                              }                              
                            } else if( prog->bx.type == BX_JUMP_TYPE_OVF ){
                              // OV
                              if( prog->bx.reg == 1 ){
                                // reg
                                ets_printf("JUMP   %s, OV", ulpRegStr[prog->bx.dreg]);
                              } else {
                                // imm
                                ets_printf("JUMP   0x%04X, OV", prog->bx.addr);
                              }                              
                            }
                          }
                          break;
    
    case OPCODE_END:      // WAKE
                          ets_printf("WAKE\t\t\t\t// WAKE");
                          break;
    
    case OPCODE_TSENS:    // TSENS
                          ets_printf("TSENS  %s, %3d", ulpRegStr[prog->tsens.dreg], prog->tsens.wait_delay);
                          break;
    
    case OPCODE_HALT:     // HALT
                          ets_printf("HALT\t\t\t\t// HALT");
                          break;
    
    case OPCODE_LD:       // LD
                          ets_printf("LD     %s, %s, %3d\t\t\t// %s = MEM[%s+%d]", ulpRegStr[prog->ld.dreg], ulpRegStr[prog->ld.sreg], prog->ld.offset, ulpRegStr[prog->ld.dreg], ulpRegStr[prog->ld.sreg], prog->ld.offset);
                          break;
    
    case OPCODE_MACRO:    // MACRO
                          ets_printf("MACRO");
                          break;

    default:              // Unknown
                          ets_printf("Unknown");
                          break;
  }
  ets_printf("\n");
}

void ulpDump(int start = 0, int end = 2047, int programAddr = 0) {
  ets_printf("====================================\n");
  for ( int i = start ; i < end ; i++ ) {
    ulpDisassemblerDump(i);

    if( i < programAddr )
      continue;

    if( i < (end-5) && RTC_SLOW_MEM[i+1] == 0 && RTC_SLOW_MEM[i+2] == 0 && RTC_SLOW_MEM[i+3] == 0 && RTC_SLOW_MEM[i+4] == 0 ){
      ets_printf ("reached the end of the program\n");
      break;
    }
  }
}

#endif // __ULPDEBUG_H__