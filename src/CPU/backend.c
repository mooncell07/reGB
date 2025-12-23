#define __AUTOGEN__

// This file is auto generated, do not modify it manually.
// All the changes go in `codegen` package and/or the `export.sh` script.

#include "backend.h"

#include <stdint.h>
#include <sys/types.h>

#include "utils.h"

#define AFp self->regs.AF.pair
#define BCp self->regs.BC.pair
#define DEp self->regs.DE.pair
#define HLp self->regs.HL.pair
#define SP self->regs.SP
#define PC self->regs.PC

#define A self->regs.AF.hi
#define F self->regs.AF.lo
#define B self->regs.BC.hi
#define C self->regs.BC.lo
#define D self->regs.DE.hi
#define E self->regs.DE.lo
#define H self->regs.HL.hi
#define L self->regs.HL.lo

#define COPY_FLAG(v, pos) F = (F & ~(1 << pos)) | (((v) & 1) << pos)

#define Cf(v) COPY_FLAG(v, ftC)
#define Hf(v) COPY_FLAG(v, ftH)
#define Nf(v) COPY_FLAG(v, ftN)
#define Zf(v) COPY_FLAG(v, ftZ)

#define getFlag(ft) BT(F, ft)

static inline uint8_t next8(State* self) {
  return read8(self->bus, PC++, true, true);
}

static inline uint16_t next16(State* self) {
  uint8_t lo = next8(self);
  uint8_t hi = next8(self);
  return JOIN(hi, lo);
}

static inline uint16_t pop(State* self) {
  uint8_t lo = read8(self->bus, SP++, true, true);
  uint8_t hi = read8(self->bus, SP++, true, true);
  return JOIN(hi, lo);
}

void push(State* self, uint16_t data) {
  write8(self->bus, --SP, MSB(data), true);
  write8(self->bus, --SP, LSB(data), true);
}

// --------------- OPCODE HANDLERS ---------------

void opNOP(State* self) {
  // 0x0 - NOP (control/misc) < T(4) | Tbr(4) >
  return;
}
void opLD_BC_U16(State* self) {
  // 0x1 - LD BC,u16 (x16/lsm) < T(12) | Tbr(12) >
  BCp = next16(self);
}
void opLD_deref_BC_A(State* self) {
  // 0x2 - LD (BC),A (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, BCp, A, true);
}
void opINC_BC(State* self) {
  // 0x3 - INC BC (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = BCp, src = 1;
  uint16_t res = dst + src;
  BCp = res;

  internal(self->bus);
}
void opINC_B(State* self) {
  // 0x4 - INC B (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = B, src = 1;
  uint8_t res = dst + src;
  B = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
}
void opDEC_B(State* self) {
  // 0x5 - DEC B (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = B, src = 1;
  uint8_t res = dst - src;
  B = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
}
void opLD_B_U8(State* self) {
  // 0x6 - LD B,u8 (x8/lsm) < T(8) | Tbr(8) >
  B = next8(self);
}
void opRLCA(State* self) {
  // 0x7 - RLCA (x8/rsb) < T(4) | Tbr(4) >
  uint8_t dst = A;
  uint8_t res = rotateLeftBits(dst, 1) | getFlag(ftC) >> 7;
  A = res;
  Zf(false);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opLD_deref_U16_SP(State* self) {
  // 0x8 - LD (u16),SP (x16/lsm) < T(20) | Tbr(20) >
  write16(self->bus, next16(self), SP);
}
void opADD_HL_BC(State* self) {
  // 0x9 - ADD HL,BC (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = HLp, src = BCp;
  uint16_t res = dst + src;
  HLp = res;
  internal(self->bus);
  Nf(false);
  Hf((dst & 0xFFF) + (src & 0xFFF) > 0xFFF);
  Cf(((uint32_t)dst + (uint32_t)src) > 0xFFFF);
}
void opLD_A_deref_BC(State* self) {
  // 0xa - LD A,(BC) (x8/lsm) < T(8) | Tbr(8) >
  A = read8(self->bus, BCp, true, true);
}
void opDEC_BC(State* self) {
  // 0xb - DEC BC (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = BCp, src = 1;
  uint16_t res = dst - src;
  BCp = res;

  internal(self->bus);
}
void opINC_C(State* self) {
  // 0xc - INC C (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = C, src = 1;
  uint8_t res = dst + src;
  C = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
}
void opDEC_C(State* self) {
  // 0xd - DEC C (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = C, src = 1;
  uint8_t res = dst - src;
  C = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
}
void opLD_C_U8(State* self) {
  // 0xe - LD C,u8 (x8/lsm) < T(8) | Tbr(8) >
  C = next8(self);
}
void opRRCA(State* self) {
  // 0xf - RRCA (x8/rsb) < T(4) | Tbr(4) >
  uint8_t dst = A;
  uint8_t res = rotateRightBits(dst, 1) | (0x80 & getFlag(ftC));
  A = res;
  Zf(false);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSTOP(State* self) {
  // 0x10 - STOP (control/misc) < T(4) | Tbr(4) >
  self->halted = true;
}
void opLD_DE_U16(State* self) {
  // 0x11 - LD DE,u16 (x16/lsm) < T(12) | Tbr(12) >
  DEp = next16(self);
}
void opLD_deref_DE_A(State* self) {
  // 0x12 - LD (DE),A (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, DEp, A, true);
}
void opINC_DE(State* self) {
  // 0x13 - INC DE (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = DEp, src = 1;
  uint16_t res = dst + src;
  DEp = res;

  internal(self->bus);
}
void opINC_D(State* self) {
  // 0x14 - INC D (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = D, src = 1;
  uint8_t res = dst + src;
  D = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
}
void opDEC_D(State* self) {
  // 0x15 - DEC D (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = D, src = 1;
  uint8_t res = dst - src;
  D = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
}
void opLD_D_U8(State* self) {
  // 0x16 - LD D,u8 (x8/lsm) < T(8) | Tbr(8) >
  D = next8(self);
}
void opRLA(State* self) {
  // 0x17 - RLA (x8/rsb) < T(4) | Tbr(4) >
  uint8_t dst = A;
  uint8_t res = (dst << 1) | getFlag(ftC);
  A = res;
  Zf(false);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opJR_I8(State* self) {
  // 0x18 - JR i8 (control/br) < T(12) | Tbr(12) >
  uint16_t address = (uint16_t)(int8_t)next8(self);
  PC += address;
  internal(self->bus);
}
void opADD_HL_DE(State* self) {
  // 0x19 - ADD HL,DE (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = HLp, src = DEp;
  uint16_t res = dst + src;
  HLp = res;
  internal(self->bus);
  Nf(false);
  Hf((dst & 0xFFF) + (src & 0xFFF) > 0xFFF);
  Cf(((uint32_t)dst + (uint32_t)src) > 0xFFFF);
}
void opLD_A_deref_DE(State* self) {
  // 0x1a - LD A,(DE) (x8/lsm) < T(8) | Tbr(8) >
  A = read8(self->bus, DEp, true, true);
}
void opDEC_DE(State* self) {
  // 0x1b - DEC DE (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = DEp, src = 1;
  uint16_t res = dst - src;
  DEp = res;

  internal(self->bus);
}
void opINC_E(State* self) {
  // 0x1c - INC E (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = E, src = 1;
  uint8_t res = dst + src;
  E = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
}
void opDEC_E(State* self) {
  // 0x1d - DEC E (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = E, src = 1;
  uint8_t res = dst - src;
  E = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
}
void opLD_E_U8(State* self) {
  // 0x1e - LD E,u8 (x8/lsm) < T(8) | Tbr(8) >
  E = next8(self);
}
void opRRA(State* self) {
  // 0x1f - RRA (x8/rsb) < T(4) | Tbr(4) >
  uint8_t dst = A;
  uint8_t res = (dst >> 1) | getFlag(ftC) << 7;
  A = res;
  Zf(false);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opJR_NZ_I8(State* self) {
  // 0x20 - JR NZ,i8 (control/br) < T(8) | Tbr(12) >
  uint16_t address = (uint16_t)(int8_t)next8(self);
  if (!getFlag(ftZ)) {
    PC += address;
    internal(self->bus);
  }
}
void opLD_HL_U16(State* self) {
  // 0x21 - LD HL,u16 (x16/lsm) < T(12) | Tbr(12) >
  HLp = next16(self);
}
void opLD_deref_HLi_A(State* self) {
  // 0x22 - LD (HL+),A (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, (HLp++), A, true);
}
void opINC_HL(State* self) {
  // 0x23 - INC HL (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = HLp, src = 1;
  uint16_t res = dst + src;
  HLp = res;

  internal(self->bus);
}
void opINC_H(State* self) {
  // 0x24 - INC H (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = H, src = 1;
  uint8_t res = dst + src;
  H = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
}
void opDEC_H(State* self) {
  // 0x25 - DEC H (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = H, src = 1;
  uint8_t res = dst - src;
  H = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
}
void opLD_H_U8(State* self) {
  // 0x26 - LD H,u8 (x8/lsm) < T(8) | Tbr(8) >
  H = next8(self);
}
void opDAA(State* self) {
  // 0x27 - DAA (x8/alu) < T(4) | Tbr(4) >
  uint8_t a = A;
  if (!getFlag(ftN)) {
    if (getFlag(ftH) | ((A & 0x0F) > 9)) { a += 6; }
    if (getFlag(ftC) | (A > 0x99)) {
      a += 0x60;
      Cf(true);
      ;
    }
  } else {
    if (getFlag(ftC)) { a -= 0x60; }
    if (getFlag(ftH)) { a -= 6; }
  }
  Zf(a == 0);
  Hf(false);
  ;
  A = a;
}
void opJR_Z_I8(State* self) {
  // 0x28 - JR Z,i8 (control/br) < T(8) | Tbr(12) >
  uint16_t address = (uint16_t)(int8_t)next8(self);
  if (getFlag(ftZ)) {
    PC += address;
    internal(self->bus);
  }
}
void opADD_HL_HL(State* self) {
  // 0x29 - ADD HL,HL (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = HLp, src = HLp;
  uint16_t res = dst + src;
  HLp = res;
  internal(self->bus);
  Nf(false);
  Hf((dst & 0xFFF) + (src & 0xFFF) > 0xFFF);
  Cf(((uint32_t)dst + (uint32_t)src) > 0xFFFF);
}
void opLD_A_deref_HLi(State* self) {
  // 0x2a - LD A,(HL+) (x8/lsm) < T(8) | Tbr(8) >
  A = read8(self->bus, (HLp++), true, true);
}
void opDEC_HL(State* self) {
  // 0x2b - DEC HL (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = HLp, src = 1;
  uint16_t res = dst - src;
  HLp = res;

  internal(self->bus);
}
void opINC_L(State* self) {
  // 0x2c - INC L (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = L, src = 1;
  uint8_t res = dst + src;
  L = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
}
void opDEC_L(State* self) {
  // 0x2d - DEC L (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = L, src = 1;
  uint8_t res = dst - src;
  L = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
}
void opLD_L_U8(State* self) {
  // 0x2e - LD L,u8 (x8/lsm) < T(8) | Tbr(8) >
  L = next8(self);
}
void opCPL(State* self) {
  // 0x2f - CPL (x8/alu) < T(4) | Tbr(4) >
  A = ~A;
  Nf(true);
  Hf(true);
}
void opJR_NC_I8(State* self) {
  // 0x30 - JR NC,i8 (control/br) < T(8) | Tbr(12) >
  uint16_t address = (uint16_t)(int8_t)next8(self);
  if (!getFlag(ftC)) {
    PC += address;
    internal(self->bus);
  }
}
void opLD_SP_U16(State* self) {
  // 0x31 - LD SP,u16 (x16/lsm) < T(12) | Tbr(12) >
  SP = next16(self);
}
void opLD_deref_HLd_A(State* self) {
  // 0x32 - LD (HL-),A (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, (HLp--), A, true);
}
void opINC_SP(State* self) {
  // 0x33 - INC SP (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = SP, src = 1;
  uint16_t res = dst + src;
  SP = res;

  internal(self->bus);
}
void opINC_deref_HL(State* self) {
  // 0x34 - INC (HL) (x8/alu) < T(12) | Tbr(12) >
  uint8_t dst = read8(self->bus, HLp, true, true), src = 1;
  uint8_t res = dst + src;
  write8(self->bus, HLp, res, true);
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
}
void opDEC_deref_HL(State* self) {
  // 0x35 - DEC (HL) (x8/alu) < T(12) | Tbr(12) >
  uint8_t dst = read8(self->bus, HLp, true, true), src = 1;
  uint8_t res = dst - src;
  write8(self->bus, HLp, res, true);
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
}
void opLD_deref_HL_U8(State* self) {
  // 0x36 - LD (HL),u8 (x8/lsm) < T(12) | Tbr(12) >
  write8(self->bus, HLp, next8(self), true);
}
void opSCF(State* self) {
  // 0x37 - SCF (x8/alu) < T(4) | Tbr(4) >
  Nf(false);
  Hf(false);
  Cf(true);
}
void opJR_C_I8(State* self) {
  // 0x38 - JR C,i8 (control/br) < T(8) | Tbr(12) >
  uint16_t address = (uint16_t)(int8_t)next8(self);
  if (getFlag(ftC)) {
    PC += address;
    internal(self->bus);
  }
}
void opADD_HL_SP(State* self) {
  // 0x39 - ADD HL,SP (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = HLp, src = SP;
  uint16_t res = dst + src;
  HLp = res;
  internal(self->bus);
  Nf(false);
  Hf((dst & 0xFFF) + (src & 0xFFF) > 0xFFF);
  Cf(((uint32_t)dst + (uint32_t)src) > 0xFFFF);
}
void opLD_A_deref_HLd(State* self) {
  // 0x3a - LD A,(HL-) (x8/lsm) < T(8) | Tbr(8) >
  A = read8(self->bus, (HLp--), true, true);
}
void opDEC_SP(State* self) {
  // 0x3b - DEC SP (x16/alu) < T(8) | Tbr(8) >
  uint16_t dst = SP, src = 1;
  uint16_t res = dst - src;
  SP = res;

  internal(self->bus);
}
void opINC_A(State* self) {
  // 0x3c - INC A (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = 1;
  uint8_t res = dst + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
}
void opDEC_A(State* self) {
  // 0x3d - DEC A (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = 1;
  uint8_t res = dst - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
}
void opLD_A_U8(State* self) {
  // 0x3e - LD A,u8 (x8/lsm) < T(8) | Tbr(8) >
  A = next8(self);
}
void opCCF(State* self) {
  // 0x3f - CCF (x8/alu) < T(4) | Tbr(4) >
  Nf(false);
  Hf(false);
  Cf(~getFlag(ftC));
}
void opLD_B_B(State* self) {
  // 0x40 - LD B,B (x8/lsm) < T(4) | Tbr(4) >
  B = B;
}
void opLD_B_C(State* self) {
  // 0x41 - LD B,C (x8/lsm) < T(4) | Tbr(4) >
  B = C;
}
void opLD_B_D(State* self) {
  // 0x42 - LD B,D (x8/lsm) < T(4) | Tbr(4) >
  B = D;
}
void opLD_B_E(State* self) {
  // 0x43 - LD B,E (x8/lsm) < T(4) | Tbr(4) >
  B = E;
}
void opLD_B_H(State* self) {
  // 0x44 - LD B,H (x8/lsm) < T(4) | Tbr(4) >
  B = H;
}
void opLD_B_L(State* self) {
  // 0x45 - LD B,L (x8/lsm) < T(4) | Tbr(4) >
  B = L;
}
void opLD_B_deref_HL(State* self) {
  // 0x46 - LD B,(HL) (x8/lsm) < T(8) | Tbr(8) >
  B = read8(self->bus, HLp, true, true);
}
void opLD_B_A(State* self) {
  // 0x47 - LD B,A (x8/lsm) < T(4) | Tbr(4) >
  B = A;
}
void opLD_C_B(State* self) {
  // 0x48 - LD C,B (x8/lsm) < T(4) | Tbr(4) >
  C = B;
}
void opLD_C_C(State* self) {
  // 0x49 - LD C,C (x8/lsm) < T(4) | Tbr(4) >
  C = C;
}
void opLD_C_D(State* self) {
  // 0x4a - LD C,D (x8/lsm) < T(4) | Tbr(4) >
  C = D;
}
void opLD_C_E(State* self) {
  // 0x4b - LD C,E (x8/lsm) < T(4) | Tbr(4) >
  C = E;
}
void opLD_C_H(State* self) {
  // 0x4c - LD C,H (x8/lsm) < T(4) | Tbr(4) >
  C = H;
}
void opLD_C_L(State* self) {
  // 0x4d - LD C,L (x8/lsm) < T(4) | Tbr(4) >
  C = L;
}
void opLD_C_deref_HL(State* self) {
  // 0x4e - LD C,(HL) (x8/lsm) < T(8) | Tbr(8) >
  C = read8(self->bus, HLp, true, true);
}
void opLD_C_A(State* self) {
  // 0x4f - LD C,A (x8/lsm) < T(4) | Tbr(4) >
  C = A;
}
void opLD_D_B(State* self) {
  // 0x50 - LD D,B (x8/lsm) < T(4) | Tbr(4) >
  D = B;
}
void opLD_D_C(State* self) {
  // 0x51 - LD D,C (x8/lsm) < T(4) | Tbr(4) >
  D = C;
}
void opLD_D_D(State* self) {
  // 0x52 - LD D,D (x8/lsm) < T(4) | Tbr(4) >
  D = D;
}
void opLD_D_E(State* self) {
  // 0x53 - LD D,E (x8/lsm) < T(4) | Tbr(4) >
  D = E;
}
void opLD_D_H(State* self) {
  // 0x54 - LD D,H (x8/lsm) < T(4) | Tbr(4) >
  D = H;
}
void opLD_D_L(State* self) {
  // 0x55 - LD D,L (x8/lsm) < T(4) | Tbr(4) >
  D = L;
}
void opLD_D_deref_HL(State* self) {
  // 0x56 - LD D,(HL) (x8/lsm) < T(8) | Tbr(8) >
  D = read8(self->bus, HLp, true, true);
}
void opLD_D_A(State* self) {
  // 0x57 - LD D,A (x8/lsm) < T(4) | Tbr(4) >
  D = A;
}
void opLD_E_B(State* self) {
  // 0x58 - LD E,B (x8/lsm) < T(4) | Tbr(4) >
  E = B;
}
void opLD_E_C(State* self) {
  // 0x59 - LD E,C (x8/lsm) < T(4) | Tbr(4) >
  E = C;
}
void opLD_E_D(State* self) {
  // 0x5a - LD E,D (x8/lsm) < T(4) | Tbr(4) >
  E = D;
}
void opLD_E_E(State* self) {
  // 0x5b - LD E,E (x8/lsm) < T(4) | Tbr(4) >
  E = E;
}
void opLD_E_H(State* self) {
  // 0x5c - LD E,H (x8/lsm) < T(4) | Tbr(4) >
  E = H;
}
void opLD_E_L(State* self) {
  // 0x5d - LD E,L (x8/lsm) < T(4) | Tbr(4) >
  E = L;
}
void opLD_E_deref_HL(State* self) {
  // 0x5e - LD E,(HL) (x8/lsm) < T(8) | Tbr(8) >
  E = read8(self->bus, HLp, true, true);
}
void opLD_E_A(State* self) {
  // 0x5f - LD E,A (x8/lsm) < T(4) | Tbr(4) >
  E = A;
}
void opLD_H_B(State* self) {
  // 0x60 - LD H,B (x8/lsm) < T(4) | Tbr(4) >
  H = B;
}
void opLD_H_C(State* self) {
  // 0x61 - LD H,C (x8/lsm) < T(4) | Tbr(4) >
  H = C;
}
void opLD_H_D(State* self) {
  // 0x62 - LD H,D (x8/lsm) < T(4) | Tbr(4) >
  H = D;
}
void opLD_H_E(State* self) {
  // 0x63 - LD H,E (x8/lsm) < T(4) | Tbr(4) >
  H = E;
}
void opLD_H_H(State* self) {
  // 0x64 - LD H,H (x8/lsm) < T(4) | Tbr(4) >
  H = H;
}
void opLD_H_L(State* self) {
  // 0x65 - LD H,L (x8/lsm) < T(4) | Tbr(4) >
  H = L;
}
void opLD_H_deref_HL(State* self) {
  // 0x66 - LD H,(HL) (x8/lsm) < T(8) | Tbr(8) >
  H = read8(self->bus, HLp, true, true);
}
void opLD_H_A(State* self) {
  // 0x67 - LD H,A (x8/lsm) < T(4) | Tbr(4) >
  H = A;
}
void opLD_L_B(State* self) {
  // 0x68 - LD L,B (x8/lsm) < T(4) | Tbr(4) >
  L = B;
}
void opLD_L_C(State* self) {
  // 0x69 - LD L,C (x8/lsm) < T(4) | Tbr(4) >
  L = C;
}
void opLD_L_D(State* self) {
  // 0x6a - LD L,D (x8/lsm) < T(4) | Tbr(4) >
  L = D;
}
void opLD_L_E(State* self) {
  // 0x6b - LD L,E (x8/lsm) < T(4) | Tbr(4) >
  L = E;
}
void opLD_L_H(State* self) {
  // 0x6c - LD L,H (x8/lsm) < T(4) | Tbr(4) >
  L = H;
}
void opLD_L_L(State* self) {
  // 0x6d - LD L,L (x8/lsm) < T(4) | Tbr(4) >
  L = L;
}
void opLD_L_deref_HL(State* self) {
  // 0x6e - LD L,(HL) (x8/lsm) < T(8) | Tbr(8) >
  L = read8(self->bus, HLp, true, true);
}
void opLD_L_A(State* self) {
  // 0x6f - LD L,A (x8/lsm) < T(4) | Tbr(4) >
  L = A;
}
void opLD_deref_HL_B(State* self) {
  // 0x70 - LD (HL),B (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, HLp, B, true);
}
void opLD_deref_HL_C(State* self) {
  // 0x71 - LD (HL),C (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, HLp, C, true);
}
void opLD_deref_HL_D(State* self) {
  // 0x72 - LD (HL),D (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, HLp, D, true);
}
void opLD_deref_HL_E(State* self) {
  // 0x73 - LD (HL),E (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, HLp, E, true);
}
void opLD_deref_HL_H(State* self) {
  // 0x74 - LD (HL),H (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, HLp, H, true);
}
void opLD_deref_HL_L(State* self) {
  // 0x75 - LD (HL),L (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, HLp, L, true);
}
void opHALT(State* self) {
  // 0x76 - HALT (control/misc) < T(4) | Tbr(4) >
  self->halted = true;
}
void opLD_deref_HL_A(State* self) {
  // 0x77 - LD (HL),A (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, HLp, A, true);
}
void opLD_A_B(State* self) {
  // 0x78 - LD A,B (x8/lsm) < T(4) | Tbr(4) >
  A = B;
}
void opLD_A_C(State* self) {
  // 0x79 - LD A,C (x8/lsm) < T(4) | Tbr(4) >
  A = C;
}
void opLD_A_D(State* self) {
  // 0x7a - LD A,D (x8/lsm) < T(4) | Tbr(4) >
  A = D;
}
void opLD_A_E(State* self) {
  // 0x7b - LD A,E (x8/lsm) < T(4) | Tbr(4) >
  A = E;
}
void opLD_A_H(State* self) {
  // 0x7c - LD A,H (x8/lsm) < T(4) | Tbr(4) >
  A = H;
}
void opLD_A_L(State* self) {
  // 0x7d - LD A,L (x8/lsm) < T(4) | Tbr(4) >
  A = L;
}
void opLD_A_deref_HL(State* self) {
  // 0x7e - LD A,(HL) (x8/lsm) < T(8) | Tbr(8) >
  A = read8(self->bus, HLp, true, true);
}
void opLD_A_A(State* self) {
  // 0x7f - LD A,A (x8/lsm) < T(4) | Tbr(4) >
  A = A;
}
void opADD_A_B(State* self) {
  // 0x80 - ADD A,B (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = B;
  uint8_t res = dst + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf(dst > res);
}
void opADD_A_C(State* self) {
  // 0x81 - ADD A,C (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = C;
  uint8_t res = dst + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf(dst > res);
}
void opADD_A_D(State* self) {
  // 0x82 - ADD A,D (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = D;
  uint8_t res = dst + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf(dst > res);
}
void opADD_A_E(State* self) {
  // 0x83 - ADD A,E (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = E;
  uint8_t res = dst + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf(dst > res);
}
void opADD_A_H(State* self) {
  // 0x84 - ADD A,H (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = H;
  uint8_t res = dst + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf(dst > res);
}
void opADD_A_L(State* self) {
  // 0x85 - ADD A,L (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = L;
  uint8_t res = dst + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf(dst > res);
}
void opADD_A_deref_HL(State* self) {
  // 0x86 - ADD A,(HL) (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = read8(self->bus, HLp, true, true);
  uint8_t res = dst + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf(dst > res);
}
void opADD_A_A(State* self) {
  // 0x87 - ADD A,A (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = A;
  uint8_t res = dst + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf(dst > res);
}
void opADC_A_B(State* self) {
  // 0x88 - ADC A,B (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = B;
  uint8_t res = dst + c + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(((dst & 0xF) + (src & 0xF) + c) > 0xF);
  Cf(((uint16_t)dst + (uint16_t)src + (uint16_t)c) > 0xFF);
}
void opADC_A_C(State* self) {
  // 0x89 - ADC A,C (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = C;
  uint8_t res = dst + c + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(((dst & 0xF) + (src & 0xF) + c) > 0xF);
  Cf(((uint16_t)dst + (uint16_t)src + (uint16_t)c) > 0xFF);
}
void opADC_A_D(State* self) {
  // 0x8a - ADC A,D (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = D;
  uint8_t res = dst + c + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(((dst & 0xF) + (src & 0xF) + c) > 0xF);
  Cf(((uint16_t)dst + (uint16_t)src + (uint16_t)c) > 0xFF);
}
void opADC_A_E(State* self) {
  // 0x8b - ADC A,E (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = E;
  uint8_t res = dst + c + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(((dst & 0xF) + (src & 0xF) + c) > 0xF);
  Cf(((uint16_t)dst + (uint16_t)src + (uint16_t)c) > 0xFF);
}
void opADC_A_H(State* self) {
  // 0x8c - ADC A,H (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = H;
  uint8_t res = dst + c + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(((dst & 0xF) + (src & 0xF) + c) > 0xF);
  Cf(((uint16_t)dst + (uint16_t)src + (uint16_t)c) > 0xFF);
}
void opADC_A_L(State* self) {
  // 0x8d - ADC A,L (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = L;
  uint8_t res = dst + c + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(((dst & 0xF) + (src & 0xF) + c) > 0xF);
  Cf(((uint16_t)dst + (uint16_t)src + (uint16_t)c) > 0xFF);
}
void opADC_A_deref_HL(State* self) {
  // 0x8e - ADC A,(HL) (x8/alu) < T(8) | Tbr(8) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = read8(self->bus, HLp, true, true);
  uint8_t res = dst + c + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(((dst & 0xF) + (src & 0xF) + c) > 0xF);
  Cf(((uint16_t)dst + (uint16_t)src + (uint16_t)c) > 0xFF);
}
void opADC_A_A(State* self) {
  // 0x8f - ADC A,A (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = A;
  uint8_t res = dst + c + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(((dst & 0xF) + (src & 0xF) + c) > 0xF);
  Cf(((uint16_t)dst + (uint16_t)src + (uint16_t)c) > 0xFF);
}
void opSUB_A_B(State* self) {
  // 0x90 - SUB A,B (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = B;
  uint8_t res = dst - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opSUB_A_C(State* self) {
  // 0x91 - SUB A,C (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = C;
  uint8_t res = dst - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opSUB_A_D(State* self) {
  // 0x92 - SUB A,D (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = D;
  uint8_t res = dst - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opSUB_A_E(State* self) {
  // 0x93 - SUB A,E (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = E;
  uint8_t res = dst - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opSUB_A_H(State* self) {
  // 0x94 - SUB A,H (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = H;
  uint8_t res = dst - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opSUB_A_L(State* self) {
  // 0x95 - SUB A,L (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = L;
  uint8_t res = dst - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opSUB_A_deref_HL(State* self) {
  // 0x96 - SUB A,(HL) (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = read8(self->bus, HLp, true, true);
  uint8_t res = dst - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opSUB_A_A(State* self) {
  // 0x97 - SUB A,A (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = A;
  uint8_t res = dst - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opSBC_A_B(State* self) {
  // 0x98 - SBC A,B (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = B;
  uint8_t res = dst - c - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf(((src & 0xF) + c) > (dst & 0xF));
  Cf(((uint16_t)src + c) > dst);
}
void opSBC_A_C(State* self) {
  // 0x99 - SBC A,C (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = C;
  uint8_t res = dst - c - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf(((src & 0xF) + c) > (dst & 0xF));
  Cf(((uint16_t)src + c) > dst);
}
void opSBC_A_D(State* self) {
  // 0x9a - SBC A,D (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = D;
  uint8_t res = dst - c - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf(((src & 0xF) + c) > (dst & 0xF));
  Cf(((uint16_t)src + c) > dst);
}
void opSBC_A_E(State* self) {
  // 0x9b - SBC A,E (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = E;
  uint8_t res = dst - c - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf(((src & 0xF) + c) > (dst & 0xF));
  Cf(((uint16_t)src + c) > dst);
}
void opSBC_A_H(State* self) {
  // 0x9c - SBC A,H (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = H;
  uint8_t res = dst - c - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf(((src & 0xF) + c) > (dst & 0xF));
  Cf(((uint16_t)src + c) > dst);
}
void opSBC_A_L(State* self) {
  // 0x9d - SBC A,L (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = L;
  uint8_t res = dst - c - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf(((src & 0xF) + c) > (dst & 0xF));
  Cf(((uint16_t)src + c) > dst);
}
void opSBC_A_deref_HL(State* self) {
  // 0x9e - SBC A,(HL) (x8/alu) < T(8) | Tbr(8) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = read8(self->bus, HLp, true, true);
  uint8_t res = dst - c - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf(((src & 0xF) + c) > (dst & 0xF));
  Cf(((uint16_t)src + c) > dst);
}
void opSBC_A_A(State* self) {
  // 0x9f - SBC A,A (x8/alu) < T(4) | Tbr(4) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = A;
  uint8_t res = dst - c - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf(((src & 0xF) + c) > (dst & 0xF));
  Cf(((uint16_t)src + c) > dst);
}
void opAND_A_B(State* self) {
  // 0xa0 - AND A,B (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = B;
  uint8_t res = dst & src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(true);
  Cf(false);
}
void opAND_A_C(State* self) {
  // 0xa1 - AND A,C (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = C;
  uint8_t res = dst & src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(true);
  Cf(false);
}
void opAND_A_D(State* self) {
  // 0xa2 - AND A,D (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = D;
  uint8_t res = dst & src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(true);
  Cf(false);
}
void opAND_A_E(State* self) {
  // 0xa3 - AND A,E (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = E;
  uint8_t res = dst & src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(true);
  Cf(false);
}
void opAND_A_H(State* self) {
  // 0xa4 - AND A,H (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = H;
  uint8_t res = dst & src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(true);
  Cf(false);
}
void opAND_A_L(State* self) {
  // 0xa5 - AND A,L (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = L;
  uint8_t res = dst & src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(true);
  Cf(false);
}
void opAND_A_deref_HL(State* self) {
  // 0xa6 - AND A,(HL) (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = read8(self->bus, HLp, true, true);
  uint8_t res = dst & src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(true);
  Cf(false);
}
void opAND_A_A(State* self) {
  // 0xa7 - AND A,A (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = A;
  uint8_t res = dst & src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(true);
  Cf(false);
}
void opXOR_A_B(State* self) {
  // 0xa8 - XOR A,B (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = B;
  uint8_t res = dst ^ src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opXOR_A_C(State* self) {
  // 0xa9 - XOR A,C (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = C;
  uint8_t res = dst ^ src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opXOR_A_D(State* self) {
  // 0xaa - XOR A,D (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = D;
  uint8_t res = dst ^ src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opXOR_A_E(State* self) {
  // 0xab - XOR A,E (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = E;
  uint8_t res = dst ^ src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opXOR_A_H(State* self) {
  // 0xac - XOR A,H (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = H;
  uint8_t res = dst ^ src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opXOR_A_L(State* self) {
  // 0xad - XOR A,L (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = L;
  uint8_t res = dst ^ src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opXOR_A_deref_HL(State* self) {
  // 0xae - XOR A,(HL) (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = read8(self->bus, HLp, true, true);
  uint8_t res = dst ^ src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opXOR_A_A(State* self) {
  // 0xaf - XOR A,A (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = A;
  uint8_t res = dst ^ src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opOR_A_B(State* self) {
  // 0xb0 - OR A,B (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = B;
  uint8_t res = dst | src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opOR_A_C(State* self) {
  // 0xb1 - OR A,C (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = C;
  uint8_t res = dst | src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opOR_A_D(State* self) {
  // 0xb2 - OR A,D (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = D;
  uint8_t res = dst | src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opOR_A_E(State* self) {
  // 0xb3 - OR A,E (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = E;
  uint8_t res = dst | src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opOR_A_H(State* self) {
  // 0xb4 - OR A,H (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = H;
  uint8_t res = dst | src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opOR_A_L(State* self) {
  // 0xb5 - OR A,L (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = L;
  uint8_t res = dst | src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opOR_A_deref_HL(State* self) {
  // 0xb6 - OR A,(HL) (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = read8(self->bus, HLp, true, true);
  uint8_t res = dst | src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opOR_A_A(State* self) {
  // 0xb7 - OR A,A (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = A;
  uint8_t res = dst | src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opCP_A_B(State* self) {
  // 0xb8 - CP A,B (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = B;
  uint8_t res = dst - src;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opCP_A_C(State* self) {
  // 0xb9 - CP A,C (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = C;
  uint8_t res = dst - src;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opCP_A_D(State* self) {
  // 0xba - CP A,D (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = D;
  uint8_t res = dst - src;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opCP_A_E(State* self) {
  // 0xbb - CP A,E (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = E;
  uint8_t res = dst - src;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opCP_A_H(State* self) {
  // 0xbc - CP A,H (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = H;
  uint8_t res = dst - src;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opCP_A_L(State* self) {
  // 0xbd - CP A,L (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = L;
  uint8_t res = dst - src;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opCP_A_deref_HL(State* self) {
  // 0xbe - CP A,(HL) (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = read8(self->bus, HLp, true, true);
  uint8_t res = dst - src;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opCP_A_A(State* self) {
  // 0xbf - CP A,A (x8/alu) < T(4) | Tbr(4) >
  uint8_t dst = A, src = A;
  uint8_t res = dst - src;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opRET_NZ(State* self) {
  // 0xc0 - RET NZ (control/br) < T(8) | Tbr(20) >
  if (!getFlag(ftZ)) {
    PC = pop(self);
    internal(self->bus);
  }
  internal(self->bus);
}
void opPOP_BC(State* self) {
  // 0xc1 - POP BC (x16/lsm) < T(12) | Tbr(12) >
  uint16_t res = pop(self);
  BCp = res;
}
void opJP_NZ_U16(State* self) {
  // 0xc2 - JP NZ,u16 (control/br) < T(12) | Tbr(16) >
  uint16_t address = next16(self);
  if (!getFlag(ftZ)) {
    PC = address;
    internal(self->bus);
  }
}
void opJP_U16(State* self) {
  // 0xc3 - JP u16 (control/br) < T(16) | Tbr(16) >
  uint16_t address = next16(self);
  PC = address;
  internal(self->bus);
}
void opCALL_NZ_U16(State* self) {
  // 0xc4 - CALL NZ,u16 (control/br) < T(12) | Tbr(24) >
  uint16_t address = next16(self);
  if (!getFlag(ftZ)) {
    push(self, PC);
    internal(self->bus);
    PC = address;
  }
}
void opPUSH_BC(State* self) {
  // 0xc5 - PUSH BC (x16/lsm) < T(16) | Tbr(16) >
  internal(self->bus);
  push(self, BCp);
}
void opADD_A_U8(State* self) {
  // 0xc6 - ADD A,u8 (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = next8(self);
  uint8_t res = dst + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf(dst > res);
}
void opRST_00H(State* self) {
  // 0xc7 - RST 00h (control/br) < T(16) | Tbr(16) >
  internal(self->bus);
  uint16_t address = 0x00;
  push(self, PC);
  PC = address;
}
void opRET_Z(State* self) {
  // 0xc8 - RET Z (control/br) < T(8) | Tbr(20) >
  if (getFlag(ftZ)) {
    PC = pop(self);
    internal(self->bus);
  }
  internal(self->bus);
}
void opRET(State* self) {
  // 0xc9 - RET (control/br) < T(16) | Tbr(16) >
  PC = pop(self);
  internal(self->bus);
}
void opJP_Z_U16(State* self) {
  // 0xca - JP Z,u16 (control/br) < T(12) | Tbr(16) >
  uint16_t address = next16(self);
  if (getFlag(ftZ)) {
    PC = address;
    internal(self->bus);
  }
}
void opPREFIX_CB(State* self) {
  // 0xcb - PREFIX CB (control/misc) < T(4) | Tbr(4) >
}
void opCALL_Z_U16(State* self) {
  // 0xcc - CALL Z,u16 (control/br) < T(12) | Tbr(24) >
  uint16_t address = next16(self);
  if (getFlag(ftZ)) {
    push(self, PC);
    internal(self->bus);
    PC = address;
  }
}
void opCALL_U16(State* self) {
  // 0xcd - CALL u16 (control/br) < T(24) | Tbr(24) >
  internal(self->bus);
  uint16_t address = next16(self);
  push(self, PC);
  PC = address;
}
void opADC_A_U8(State* self) {
  // 0xce - ADC A,u8 (x8/alu) < T(8) | Tbr(8) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = next8(self);
  uint8_t res = dst + c + src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(((dst & 0xF) + (src & 0xF) + c) > 0xF);
  Cf(((uint16_t)dst + (uint16_t)src + (uint16_t)c) > 0xFF);
}
void opRST_08H(State* self) {
  // 0xcf - RST 08h (control/br) < T(16) | Tbr(16) >
  internal(self->bus);
  uint16_t address = 0x08;
  push(self, PC);
  PC = address;
}
void opRET_NC(State* self) {
  // 0xd0 - RET NC (control/br) < T(8) | Tbr(20) >
  if (!getFlag(ftC)) {
    PC = pop(self);
    internal(self->bus);
  }
  internal(self->bus);
}
void opPOP_DE(State* self) {
  // 0xd1 - POP DE (x16/lsm) < T(12) | Tbr(12) >
  uint16_t res = pop(self);
  DEp = res;
}
void opJP_NC_U16(State* self) {
  // 0xd2 - JP NC,u16 (control/br) < T(12) | Tbr(16) >
  uint16_t address = next16(self);
  if (!getFlag(ftC)) {
    PC = address;
    internal(self->bus);
  }
}

void opCALL_NC_U16(State* self) {
  // 0xd4 - CALL NC,u16 (control/br) < T(12) | Tbr(24) >
  uint16_t address = next16(self);
  if (!getFlag(ftC)) {
    push(self, PC);
    internal(self->bus);
    PC = address;
  }
}
void opPUSH_DE(State* self) {
  // 0xd5 - PUSH DE (x16/lsm) < T(16) | Tbr(16) >
  internal(self->bus);
  push(self, DEp);
}
void opSUB_A_U8(State* self) {
  // 0xd6 - SUB A,u8 (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = next8(self);
  uint8_t res = dst - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opRST_10H(State* self) {
  // 0xd7 - RST 10h (control/br) < T(16) | Tbr(16) >
  internal(self->bus);
  uint16_t address = 0x10;
  push(self, PC);
  PC = address;
}
void opRET_C(State* self) {
  // 0xd8 - RET C (control/br) < T(8) | Tbr(20) >
  if (getFlag(ftC)) {
    PC = pop(self);
    internal(self->bus);
  }
  internal(self->bus);
}
void opRETI(State* self) {
  // 0xd9 - RETI (control/br) < T(16) | Tbr(16) >
  self->IMERising = true;
  opRET(self);
}
void opJP_C_U16(State* self) {
  // 0xda - JP C,u16 (control/br) < T(12) | Tbr(16) >
  uint16_t address = next16(self);
  if (getFlag(ftC)) {
    PC = address;
    internal(self->bus);
  }
}

void opCALL_C_U16(State* self) {
  // 0xdc - CALL C,u16 (control/br) < T(12) | Tbr(24) >
  uint16_t address = next16(self);
  if (getFlag(ftC)) {
    push(self, PC);
    internal(self->bus);
    PC = address;
  }
}

void opSBC_A_U8(State* self) {
  // 0xde - SBC A,u8 (x8/alu) < T(8) | Tbr(8) >
  uint8_t c = getFlag(ftC);
  uint8_t dst = A, src = next8(self);
  uint8_t res = dst - c - src;
  A = res;
  Zf(!res);
  Nf(true);
  Hf(((src & 0xF) + c) > (dst & 0xF));
  Cf(((uint16_t)src + c) > dst);
}
void opRST_18H(State* self) {
  // 0xdf - RST 18h (control/br) < T(16) | Tbr(16) >
  internal(self->bus);
  uint16_t address = 0x18;
  push(self, PC);
  PC = address;
}
void opLD_deref_FF00_U8_A(State* self) {
  // 0xe0 - LD (FF00+u8),A (x8/lsm) < T(12) | Tbr(12) >
  write8(self->bus, (0xFF00 + next8(self)), A, true);
}
void opPOP_HL(State* self) {
  // 0xe1 - POP HL (x16/lsm) < T(12) | Tbr(12) >
  uint16_t res = pop(self);
  HLp = res;
}
void opLD_deref_FF00_C_A(State* self) {
  // 0xe2 - LD (FF00+C),A (x8/lsm) < T(8) | Tbr(8) >
  write8(self->bus, (0xFF00 + C), A, true);
}

void opPUSH_HL(State* self) {
  // 0xe5 - PUSH HL (x16/lsm) < T(16) | Tbr(16) >
  internal(self->bus);
  push(self, HLp);
}
void opAND_A_U8(State* self) {
  // 0xe6 - AND A,u8 (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = next8(self);
  uint8_t res = dst & src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(true);
  Cf(false);
}
void opRST_20H(State* self) {
  // 0xe7 - RST 20h (control/br) < T(16) | Tbr(16) >
  internal(self->bus);
  uint16_t address = 0x20;
  push(self, PC);
  PC = address;
}
void opADD_SP_I8(State* self) {
  // 0xe8 - ADD SP,i8 (x16/alu) < T(16) | Tbr(16) >
  uint16_t dst = SP, src = (uint16_t)(int8_t)next8(self);
  uint16_t res = src + dst;
  SP = res;
  internal(self->bus);
  Zf(false);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf((dst & 0xFF) + (src & 0xFF) > 0xFF);
  internal(self->bus);
}
void opJP_HL(State* self) {
  // 0xe9 - JP HL (control/br) < T(4) | Tbr(4) >
  PC = HLp;
}
void opLD_deref_U16_A(State* self) {
  // 0xea - LD (u16),A (x8/lsm) < T(16) | Tbr(16) >
  write8(self->bus, next16(self), A, true);
}

void opXOR_A_U8(State* self) {
  // 0xee - XOR A,u8 (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = next8(self);
  uint8_t res = dst ^ src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opRST_28H(State* self) {
  // 0xef - RST 28h (control/br) < T(16) | Tbr(16) >
  internal(self->bus);
  uint16_t address = 0x28;
  push(self, PC);
  PC = address;
}
void opLD_A_deref_FF00_U8(State* self) {
  // 0xf0 - LD A,(FF00+u8) (x8/lsm) < T(12) | Tbr(12) >
  A = read8(self->bus, (0xFF00 + next8(self)), true, true);
}
void opPOP_AF(State* self) {
  // 0xf1 - POP AF (x16/lsm) < T(12) | Tbr(12) >
  uint16_t res = pop(self);
  A = MSB(res);
  F = res & 0xFFF0;
}
void opLD_A_deref_FF00_C(State* self) {
  // 0xf2 - LD A,(FF00+C) (x8/lsm) < T(8) | Tbr(8) >
  A = read8(self->bus, (0xFF00 + C), true, true);
}
void opDI(State* self) {
  // 0xf3 - DI (control/misc) < T(4) | Tbr(4) >
  self->IME = false;
}

void opPUSH_AF(State* self) {
  // 0xf5 - PUSH AF (x16/lsm) < T(16) | Tbr(16) >
  internal(self->bus);
  push(self, AFp);
}
void opOR_A_U8(State* self) {
  // 0xf6 - OR A,u8 (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = next8(self);
  uint8_t res = dst | src;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opRST_30H(State* self) {
  // 0xf7 - RST 30h (control/br) < T(16) | Tbr(16) >
  internal(self->bus);
  uint16_t address = 0x30;
  push(self, PC);
  PC = address;
}
void opLD_HL_SP_I8(State* self) {
  // 0xf8 - LD HL,SP+i8 (x16/alu) < T(12) | Tbr(12) >
  uint16_t dst = SP, src = (uint16_t)(int8_t)next8(self);
  uint16_t res = src + dst;
  HLp = res;
  internal(self->bus);
  Zf(false);
  Nf(false);
  Hf((dst & 0xF) + (src & 0xF) >= 0x10);
  Cf((dst & 0xFF) + (src & 0xFF) > 0xFF);
}
void opLD_SP_HL(State* self) {
  // 0xf9 - LD SP,HL (x16/lsm) < T(8) | Tbr(8) >
  internal(self->bus);
  SP = HLp;
}
void opLD_A_deref_U16(State* self) {
  // 0xfa - LD A,(u16) (x8/lsm) < T(16) | Tbr(16) >
  A = read8(self->bus, next16(self), true, true);
}
void opEI(State* self) {
  // 0xfb - EI (control/misc) < T(4) | Tbr(4) >
  self->IMERising = true;
}

void opCP_A_U8(State* self) {
  // 0xfe - CP A,u8 (x8/alu) < T(8) | Tbr(8) >
  uint8_t dst = A, src = next8(self);
  uint8_t res = dst - src;
  Zf(!res);
  Nf(true);
  Hf((dst & 0xF) < (src & 0xF));
  Cf(src > dst);
}
void opRST_38H(State* self) {
  // 0xff - RST 38h (control/br) < T(16) | Tbr(16) >
  internal(self->bus);
  uint16_t address = 0x38;
  push(self, PC);
  PC = address;
}
void opRLC_B(State* self) {
  // 0x0 - RLC B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = rotateLeftBits(dst, 1) | getFlag(ftC) >> 7;
  B = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRLC_C(State* self) {
  // 0x1 - RLC C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = rotateLeftBits(dst, 1) | getFlag(ftC) >> 7;
  C = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRLC_D(State* self) {
  // 0x2 - RLC D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = rotateLeftBits(dst, 1) | getFlag(ftC) >> 7;
  D = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRLC_E(State* self) {
  // 0x3 - RLC E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = rotateLeftBits(dst, 1) | getFlag(ftC) >> 7;
  E = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRLC_H(State* self) {
  // 0x4 - RLC H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = rotateLeftBits(dst, 1) | getFlag(ftC) >> 7;
  H = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRLC_L(State* self) {
  // 0x5 - RLC L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = rotateLeftBits(dst, 1) | getFlag(ftC) >> 7;
  L = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRLC_deref_HL(State* self) {
  // 0x6 - RLC (HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = rotateLeftBits(dst, 1) | getFlag(ftC) >> 7;
  write8(self->bus, HLp, res, true);
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRLC_A(State* self) {
  // 0x7 - RLC A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = rotateLeftBits(dst, 1) | getFlag(ftC) >> 7;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRRC_B(State* self) {
  // 0x8 - RRC B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = rotateRightBits(dst, 1) | (0x80 & getFlag(ftC));
  B = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRRC_C(State* self) {
  // 0x9 - RRC C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = rotateRightBits(dst, 1) | (0x80 & getFlag(ftC));
  C = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRRC_D(State* self) {
  // 0xa - RRC D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = rotateRightBits(dst, 1) | (0x80 & getFlag(ftC));
  D = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRRC_E(State* self) {
  // 0xb - RRC E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = rotateRightBits(dst, 1) | (0x80 & getFlag(ftC));
  E = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRRC_H(State* self) {
  // 0xc - RRC H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = rotateRightBits(dst, 1) | (0x80 & getFlag(ftC));
  H = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRRC_L(State* self) {
  // 0xd - RRC L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = rotateRightBits(dst, 1) | (0x80 & getFlag(ftC));
  L = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRRC_deref_HL(State* self) {
  // 0xe - RRC (HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = rotateRightBits(dst, 1) | (0x80 & getFlag(ftC));
  write8(self->bus, HLp, res, true);
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRRC_A(State* self) {
  // 0xf - RRC A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = rotateRightBits(dst, 1) | (0x80 & getFlag(ftC));
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRL_B(State* self) {
  // 0x10 - RL B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = (dst << 1) | getFlag(ftC);
  B = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRL_C(State* self) {
  // 0x11 - RL C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = (dst << 1) | getFlag(ftC);
  C = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRL_D(State* self) {
  // 0x12 - RL D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = (dst << 1) | getFlag(ftC);
  D = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRL_E(State* self) {
  // 0x13 - RL E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = (dst << 1) | getFlag(ftC);
  E = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRL_H(State* self) {
  // 0x14 - RL H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = (dst << 1) | getFlag(ftC);
  H = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRL_L(State* self) {
  // 0x15 - RL L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = (dst << 1) | getFlag(ftC);
  L = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRL_deref_HL(State* self) {
  // 0x16 - RL (HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = (dst << 1) | getFlag(ftC);
  write8(self->bus, HLp, res, true);
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRL_A(State* self) {
  // 0x17 - RL A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = (dst << 1) | getFlag(ftC);
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opRR_B(State* self) {
  // 0x18 - RR B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = (dst >> 1) | (getFlag(ftC) << 7);
  B = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRR_C(State* self) {
  // 0x19 - RR C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = (dst >> 1) | (getFlag(ftC) << 7);
  C = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRR_D(State* self) {
  // 0x1a - RR D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = (dst >> 1) | (getFlag(ftC) << 7);
  D = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRR_E(State* self) {
  // 0x1b - RR E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = (dst >> 1) | (getFlag(ftC) << 7);
  E = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRR_H(State* self) {
  // 0x1c - RR H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = (dst >> 1) | (getFlag(ftC) << 7);
  H = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRR_L(State* self) {
  // 0x1d - RR L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = (dst >> 1) | (getFlag(ftC) << 7);
  L = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRR_deref_HL(State* self) {
  // 0x1e - RR (HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = (dst >> 1) | (getFlag(ftC) << 7);
  write8(self->bus, HLp, res, true);
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opRR_A(State* self) {
  // 0x1f - RR A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = (dst >> 1) | (getFlag(ftC) << 7);
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSLA_B(State* self) {
  // 0x20 - SLA B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = (dst << 1);
  B = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opSLA_C(State* self) {
  // 0x21 - SLA C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = (dst << 1);
  C = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opSLA_D(State* self) {
  // 0x22 - SLA D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = (dst << 1);
  D = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opSLA_E(State* self) {
  // 0x23 - SLA E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = (dst << 1);
  E = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opSLA_H(State* self) {
  // 0x24 - SLA H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = (dst << 1);
  H = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opSLA_L(State* self) {
  // 0x25 - SLA L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = (dst << 1);
  L = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opSLA_deref_HL(State* self) {
  // 0x26 - SLA (HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = (dst << 1);
  write8(self->bus, HLp, res, true);
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opSLA_A(State* self) {
  // 0x27 - SLA A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = (dst << 1);
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst >> 7);
}
void opSRA_B(State* self) {
  // 0x28 - SRA B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = (dst >> 1) | (dst & 0x80);
  B = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRA_C(State* self) {
  // 0x29 - SRA C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = (dst >> 1) | (dst & 0x80);
  C = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRA_D(State* self) {
  // 0x2a - SRA D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = (dst >> 1) | (dst & 0x80);
  D = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRA_E(State* self) {
  // 0x2b - SRA E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = (dst >> 1) | (dst & 0x80);
  E = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRA_H(State* self) {
  // 0x2c - SRA H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = (dst >> 1) | (dst & 0x80);
  H = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRA_L(State* self) {
  // 0x2d - SRA L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = (dst >> 1) | (dst & 0x80);
  L = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRA_deref_HL(State* self) {
  // 0x2e - SRA (HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = (dst >> 1) | (dst & 0x80);
  write8(self->bus, HLp, res, true);
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRA_A(State* self) {
  // 0x2f - SRA A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = (dst >> 1) | (dst & 0x80);
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSWAP_B(State* self) {
  // 0x30 - SWAP B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = rotateLeftBits(dst, 4);
  B = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opSWAP_C(State* self) {
  // 0x31 - SWAP C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = rotateLeftBits(dst, 4);
  C = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opSWAP_D(State* self) {
  // 0x32 - SWAP D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = rotateLeftBits(dst, 4);
  D = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opSWAP_E(State* self) {
  // 0x33 - SWAP E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = rotateLeftBits(dst, 4);
  E = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opSWAP_H(State* self) {
  // 0x34 - SWAP H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = rotateLeftBits(dst, 4);
  H = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opSWAP_L(State* self) {
  // 0x35 - SWAP L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = rotateLeftBits(dst, 4);
  L = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opSWAP_deref_HL(State* self) {
  // 0x36 - SWAP (HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = rotateLeftBits(dst, 4);
  write8(self->bus, HLp, res, true);
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opSWAP_A(State* self) {
  // 0x37 - SWAP A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = rotateLeftBits(dst, 4);
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(false);
}
void opSRL_B(State* self) {
  // 0x38 - SRL B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = dst >> 1;
  B = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRL_C(State* self) {
  // 0x39 - SRL C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = dst >> 1;
  C = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRL_D(State* self) {
  // 0x3a - SRL D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = dst >> 1;
  D = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRL_E(State* self) {
  // 0x3b - SRL E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = dst >> 1;
  E = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRL_H(State* self) {
  // 0x3c - SRL H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = dst >> 1;
  H = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRL_L(State* self) {
  // 0x3d - SRL L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = dst >> 1;
  L = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRL_deref_HL(State* self) {
  // 0x3e - SRL (HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = dst >> 1;
  write8(self->bus, HLp, res, true);
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opSRL_A(State* self) {
  // 0x3f - SRL A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = dst >> 1;
  A = res;
  Zf(!res);
  Nf(false);
  Hf(false);
  Cf(dst & 1);
}
void opBIT_0_B(State* self) {
  // 0x40 - BIT 0,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = BT(dst, 0);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_0_C(State* self) {
  // 0x41 - BIT 0,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = BT(dst, 0);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_0_D(State* self) {
  // 0x42 - BIT 0,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = BT(dst, 0);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_0_E(State* self) {
  // 0x43 - BIT 0,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = BT(dst, 0);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_0_H(State* self) {
  // 0x44 - BIT 0,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = BT(dst, 0);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_0_L(State* self) {
  // 0x45 - BIT 0,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = BT(dst, 0);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_0_deref_HL(State* self) {
  // 0x46 - BIT 0,(HL) (x8/rsb) < T(12) | Tbr(12) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = BT(dst, 0);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_0_A(State* self) {
  // 0x47 - BIT 0,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = BT(dst, 0);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_1_B(State* self) {
  // 0x48 - BIT 1,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = BT(dst, 1);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_1_C(State* self) {
  // 0x49 - BIT 1,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = BT(dst, 1);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_1_D(State* self) {
  // 0x4a - BIT 1,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = BT(dst, 1);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_1_E(State* self) {
  // 0x4b - BIT 1,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = BT(dst, 1);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_1_H(State* self) {
  // 0x4c - BIT 1,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = BT(dst, 1);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_1_L(State* self) {
  // 0x4d - BIT 1,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = BT(dst, 1);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_1_deref_HL(State* self) {
  // 0x4e - BIT 1,(HL) (x8/rsb) < T(12) | Tbr(12) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = BT(dst, 1);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_1_A(State* self) {
  // 0x4f - BIT 1,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = BT(dst, 1);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_2_B(State* self) {
  // 0x50 - BIT 2,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = BT(dst, 2);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_2_C(State* self) {
  // 0x51 - BIT 2,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = BT(dst, 2);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_2_D(State* self) {
  // 0x52 - BIT 2,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = BT(dst, 2);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_2_E(State* self) {
  // 0x53 - BIT 2,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = BT(dst, 2);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_2_H(State* self) {
  // 0x54 - BIT 2,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = BT(dst, 2);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_2_L(State* self) {
  // 0x55 - BIT 2,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = BT(dst, 2);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_2_deref_HL(State* self) {
  // 0x56 - BIT 2,(HL) (x8/rsb) < T(12) | Tbr(12) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = BT(dst, 2);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_2_A(State* self) {
  // 0x57 - BIT 2,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = BT(dst, 2);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_3_B(State* self) {
  // 0x58 - BIT 3,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = BT(dst, 3);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_3_C(State* self) {
  // 0x59 - BIT 3,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = BT(dst, 3);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_3_D(State* self) {
  // 0x5a - BIT 3,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = BT(dst, 3);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_3_E(State* self) {
  // 0x5b - BIT 3,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = BT(dst, 3);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_3_H(State* self) {
  // 0x5c - BIT 3,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = BT(dst, 3);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_3_L(State* self) {
  // 0x5d - BIT 3,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = BT(dst, 3);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_3_deref_HL(State* self) {
  // 0x5e - BIT 3,(HL) (x8/rsb) < T(12) | Tbr(12) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = BT(dst, 3);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_3_A(State* self) {
  // 0x5f - BIT 3,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = BT(dst, 3);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_4_B(State* self) {
  // 0x60 - BIT 4,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = BT(dst, 4);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_4_C(State* self) {
  // 0x61 - BIT 4,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = BT(dst, 4);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_4_D(State* self) {
  // 0x62 - BIT 4,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = BT(dst, 4);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_4_E(State* self) {
  // 0x63 - BIT 4,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = BT(dst, 4);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_4_H(State* self) {
  // 0x64 - BIT 4,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = BT(dst, 4);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_4_L(State* self) {
  // 0x65 - BIT 4,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = BT(dst, 4);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_4_deref_HL(State* self) {
  // 0x66 - BIT 4,(HL) (x8/rsb) < T(12) | Tbr(12) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = BT(dst, 4);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_4_A(State* self) {
  // 0x67 - BIT 4,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = BT(dst, 4);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_5_B(State* self) {
  // 0x68 - BIT 5,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = BT(dst, 5);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_5_C(State* self) {
  // 0x69 - BIT 5,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = BT(dst, 5);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_5_D(State* self) {
  // 0x6a - BIT 5,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = BT(dst, 5);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_5_E(State* self) {
  // 0x6b - BIT 5,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = BT(dst, 5);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_5_H(State* self) {
  // 0x6c - BIT 5,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = BT(dst, 5);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_5_L(State* self) {
  // 0x6d - BIT 5,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = BT(dst, 5);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_5_deref_HL(State* self) {
  // 0x6e - BIT 5,(HL) (x8/rsb) < T(12) | Tbr(12) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = BT(dst, 5);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_5_A(State* self) {
  // 0x6f - BIT 5,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = BT(dst, 5);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_6_B(State* self) {
  // 0x70 - BIT 6,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = BT(dst, 6);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_6_C(State* self) {
  // 0x71 - BIT 6,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = BT(dst, 6);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_6_D(State* self) {
  // 0x72 - BIT 6,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = BT(dst, 6);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_6_E(State* self) {
  // 0x73 - BIT 6,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = BT(dst, 6);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_6_H(State* self) {
  // 0x74 - BIT 6,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = BT(dst, 6);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_6_L(State* self) {
  // 0x75 - BIT 6,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = BT(dst, 6);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_6_deref_HL(State* self) {
  // 0x76 - BIT 6,(HL) (x8/rsb) < T(12) | Tbr(12) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = BT(dst, 6);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_6_A(State* self) {
  // 0x77 - BIT 6,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = BT(dst, 6);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_7_B(State* self) {
  // 0x78 - BIT 7,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = BT(dst, 7);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_7_C(State* self) {
  // 0x79 - BIT 7,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = BT(dst, 7);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_7_D(State* self) {
  // 0x7a - BIT 7,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = BT(dst, 7);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_7_E(State* self) {
  // 0x7b - BIT 7,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = BT(dst, 7);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_7_H(State* self) {
  // 0x7c - BIT 7,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = BT(dst, 7);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_7_L(State* self) {
  // 0x7d - BIT 7,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = BT(dst, 7);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_7_deref_HL(State* self) {
  // 0x7e - BIT 7,(HL) (x8/rsb) < T(12) | Tbr(12) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = BT(dst, 7);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opBIT_7_A(State* self) {
  // 0x7f - BIT 7,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = BT(dst, 7);
  Zf(!res);
  Nf(false);
  Hf(true);
}
void opRES_0_B(State* self) {
  // 0x80 - RES 0,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = clearBit(dst, 0);
  B = res;
}
void opRES_0_C(State* self) {
  // 0x81 - RES 0,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = clearBit(dst, 0);
  C = res;
}
void opRES_0_D(State* self) {
  // 0x82 - RES 0,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = clearBit(dst, 0);
  D = res;
}
void opRES_0_E(State* self) {
  // 0x83 - RES 0,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = clearBit(dst, 0);
  E = res;
}
void opRES_0_H(State* self) {
  // 0x84 - RES 0,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = clearBit(dst, 0);
  H = res;
}
void opRES_0_L(State* self) {
  // 0x85 - RES 0,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = clearBit(dst, 0);
  L = res;
}
void opRES_0_deref_HL(State* self) {
  // 0x86 - RES 0,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = clearBit(dst, 0);
  write8(self->bus, HLp, res, true);
}
void opRES_0_A(State* self) {
  // 0x87 - RES 0,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = clearBit(dst, 0);
  A = res;
}
void opRES_1_B(State* self) {
  // 0x88 - RES 1,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = clearBit(dst, 1);
  B = res;
}
void opRES_1_C(State* self) {
  // 0x89 - RES 1,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = clearBit(dst, 1);
  C = res;
}
void opRES_1_D(State* self) {
  // 0x8a - RES 1,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = clearBit(dst, 1);
  D = res;
}
void opRES_1_E(State* self) {
  // 0x8b - RES 1,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = clearBit(dst, 1);
  E = res;
}
void opRES_1_H(State* self) {
  // 0x8c - RES 1,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = clearBit(dst, 1);
  H = res;
}
void opRES_1_L(State* self) {
  // 0x8d - RES 1,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = clearBit(dst, 1);
  L = res;
}
void opRES_1_deref_HL(State* self) {
  // 0x8e - RES 1,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = clearBit(dst, 1);
  write8(self->bus, HLp, res, true);
}
void opRES_1_A(State* self) {
  // 0x8f - RES 1,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = clearBit(dst, 1);
  A = res;
}
void opRES_2_B(State* self) {
  // 0x90 - RES 2,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = clearBit(dst, 2);
  B = res;
}
void opRES_2_C(State* self) {
  // 0x91 - RES 2,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = clearBit(dst, 2);
  C = res;
}
void opRES_2_D(State* self) {
  // 0x92 - RES 2,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = clearBit(dst, 2);
  D = res;
}
void opRES_2_E(State* self) {
  // 0x93 - RES 2,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = clearBit(dst, 2);
  E = res;
}
void opRES_2_H(State* self) {
  // 0x94 - RES 2,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = clearBit(dst, 2);
  H = res;
}
void opRES_2_L(State* self) {
  // 0x95 - RES 2,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = clearBit(dst, 2);
  L = res;
}
void opRES_2_deref_HL(State* self) {
  // 0x96 - RES 2,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = clearBit(dst, 2);
  write8(self->bus, HLp, res, true);
}
void opRES_2_A(State* self) {
  // 0x97 - RES 2,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = clearBit(dst, 2);
  A = res;
}
void opRES_3_B(State* self) {
  // 0x98 - RES 3,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = clearBit(dst, 3);
  B = res;
}
void opRES_3_C(State* self) {
  // 0x99 - RES 3,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = clearBit(dst, 3);
  C = res;
}
void opRES_3_D(State* self) {
  // 0x9a - RES 3,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = clearBit(dst, 3);
  D = res;
}
void opRES_3_E(State* self) {
  // 0x9b - RES 3,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = clearBit(dst, 3);
  E = res;
}
void opRES_3_H(State* self) {
  // 0x9c - RES 3,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = clearBit(dst, 3);
  H = res;
}
void opRES_3_L(State* self) {
  // 0x9d - RES 3,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = clearBit(dst, 3);
  L = res;
}
void opRES_3_deref_HL(State* self) {
  // 0x9e - RES 3,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = clearBit(dst, 3);
  write8(self->bus, HLp, res, true);
}
void opRES_3_A(State* self) {
  // 0x9f - RES 3,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = clearBit(dst, 3);
  A = res;
}
void opRES_4_B(State* self) {
  // 0xa0 - RES 4,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = clearBit(dst, 4);
  B = res;
}
void opRES_4_C(State* self) {
  // 0xa1 - RES 4,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = clearBit(dst, 4);
  C = res;
}
void opRES_4_D(State* self) {
  // 0xa2 - RES 4,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = clearBit(dst, 4);
  D = res;
}
void opRES_4_E(State* self) {
  // 0xa3 - RES 4,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = clearBit(dst, 4);
  E = res;
}
void opRES_4_H(State* self) {
  // 0xa4 - RES 4,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = clearBit(dst, 4);
  H = res;
}
void opRES_4_L(State* self) {
  // 0xa5 - RES 4,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = clearBit(dst, 4);
  L = res;
}
void opRES_4_deref_HL(State* self) {
  // 0xa6 - RES 4,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = clearBit(dst, 4);
  write8(self->bus, HLp, res, true);
}
void opRES_4_A(State* self) {
  // 0xa7 - RES 4,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = clearBit(dst, 4);
  A = res;
}
void opRES_5_B(State* self) {
  // 0xa8 - RES 5,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = clearBit(dst, 5);
  B = res;
}
void opRES_5_C(State* self) {
  // 0xa9 - RES 5,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = clearBit(dst, 5);
  C = res;
}
void opRES_5_D(State* self) {
  // 0xaa - RES 5,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = clearBit(dst, 5);
  D = res;
}
void opRES_5_E(State* self) {
  // 0xab - RES 5,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = clearBit(dst, 5);
  E = res;
}
void opRES_5_H(State* self) {
  // 0xac - RES 5,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = clearBit(dst, 5);
  H = res;
}
void opRES_5_L(State* self) {
  // 0xad - RES 5,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = clearBit(dst, 5);
  L = res;
}
void opRES_5_deref_HL(State* self) {
  // 0xae - RES 5,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = clearBit(dst, 5);
  write8(self->bus, HLp, res, true);
}
void opRES_5_A(State* self) {
  // 0xaf - RES 5,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = clearBit(dst, 5);
  A = res;
}
void opRES_6_B(State* self) {
  // 0xb0 - RES 6,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = clearBit(dst, 6);
  B = res;
}
void opRES_6_C(State* self) {
  // 0xb1 - RES 6,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = clearBit(dst, 6);
  C = res;
}
void opRES_6_D(State* self) {
  // 0xb2 - RES 6,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = clearBit(dst, 6);
  D = res;
}
void opRES_6_E(State* self) {
  // 0xb3 - RES 6,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = clearBit(dst, 6);
  E = res;
}
void opRES_6_H(State* self) {
  // 0xb4 - RES 6,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = clearBit(dst, 6);
  H = res;
}
void opRES_6_L(State* self) {
  // 0xb5 - RES 6,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = clearBit(dst, 6);
  L = res;
}
void opRES_6_deref_HL(State* self) {
  // 0xb6 - RES 6,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = clearBit(dst, 6);
  write8(self->bus, HLp, res, true);
}
void opRES_6_A(State* self) {
  // 0xb7 - RES 6,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = clearBit(dst, 6);
  A = res;
}
void opRES_7_B(State* self) {
  // 0xb8 - RES 7,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = clearBit(dst, 7);
  B = res;
}
void opRES_7_C(State* self) {
  // 0xb9 - RES 7,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = clearBit(dst, 7);
  C = res;
}
void opRES_7_D(State* self) {
  // 0xba - RES 7,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = clearBit(dst, 7);
  D = res;
}
void opRES_7_E(State* self) {
  // 0xbb - RES 7,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = clearBit(dst, 7);
  E = res;
}
void opRES_7_H(State* self) {
  // 0xbc - RES 7,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = clearBit(dst, 7);
  H = res;
}
void opRES_7_L(State* self) {
  // 0xbd - RES 7,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = clearBit(dst, 7);
  L = res;
}
void opRES_7_deref_HL(State* self) {
  // 0xbe - RES 7,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = clearBit(dst, 7);
  write8(self->bus, HLp, res, true);
}
void opRES_7_A(State* self) {
  // 0xbf - RES 7,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = clearBit(dst, 7);
  A = res;
}
void opSET_0_B(State* self) {
  // 0xc0 - SET 0,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = setBit(dst, 0);
  B = res;
}
void opSET_0_C(State* self) {
  // 0xc1 - SET 0,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = setBit(dst, 0);
  C = res;
}
void opSET_0_D(State* self) {
  // 0xc2 - SET 0,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = setBit(dst, 0);
  D = res;
}
void opSET_0_E(State* self) {
  // 0xc3 - SET 0,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = setBit(dst, 0);
  E = res;
}
void opSET_0_H(State* self) {
  // 0xc4 - SET 0,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = setBit(dst, 0);
  H = res;
}
void opSET_0_L(State* self) {
  // 0xc5 - SET 0,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = setBit(dst, 0);
  L = res;
}
void opSET_0_deref_HL(State* self) {
  // 0xc6 - SET 0,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = setBit(dst, 0);
  write8(self->bus, HLp, res, true);
}
void opSET_0_A(State* self) {
  // 0xc7 - SET 0,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = setBit(dst, 0);
  A = res;
}
void opSET_1_B(State* self) {
  // 0xc8 - SET 1,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = setBit(dst, 1);
  B = res;
}
void opSET_1_C(State* self) {
  // 0xc9 - SET 1,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = setBit(dst, 1);
  C = res;
}
void opSET_1_D(State* self) {
  // 0xca - SET 1,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = setBit(dst, 1);
  D = res;
}
void opSET_1_E(State* self) {
  // 0xcb - SET 1,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = setBit(dst, 1);
  E = res;
}
void opSET_1_H(State* self) {
  // 0xcc - SET 1,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = setBit(dst, 1);
  H = res;
}
void opSET_1_L(State* self) {
  // 0xcd - SET 1,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = setBit(dst, 1);
  L = res;
}
void opSET_1_deref_HL(State* self) {
  // 0xce - SET 1,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = setBit(dst, 1);
  write8(self->bus, HLp, res, true);
}
void opSET_1_A(State* self) {
  // 0xcf - SET 1,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = setBit(dst, 1);
  A = res;
}
void opSET_2_B(State* self) {
  // 0xd0 - SET 2,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = setBit(dst, 2);
  B = res;
}
void opSET_2_C(State* self) {
  // 0xd1 - SET 2,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = setBit(dst, 2);
  C = res;
}
void opSET_2_D(State* self) {
  // 0xd2 - SET 2,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = setBit(dst, 2);
  D = res;
}
void opSET_2_E(State* self) {
  // 0xd3 - SET 2,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = setBit(dst, 2);
  E = res;
}
void opSET_2_H(State* self) {
  // 0xd4 - SET 2,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = setBit(dst, 2);
  H = res;
}
void opSET_2_L(State* self) {
  // 0xd5 - SET 2,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = setBit(dst, 2);
  L = res;
}
void opSET_2_deref_HL(State* self) {
  // 0xd6 - SET 2,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = setBit(dst, 2);
  write8(self->bus, HLp, res, true);
}
void opSET_2_A(State* self) {
  // 0xd7 - SET 2,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = setBit(dst, 2);
  A = res;
}
void opSET_3_B(State* self) {
  // 0xd8 - SET 3,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = setBit(dst, 3);
  B = res;
}
void opSET_3_C(State* self) {
  // 0xd9 - SET 3,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = setBit(dst, 3);
  C = res;
}
void opSET_3_D(State* self) {
  // 0xda - SET 3,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = setBit(dst, 3);
  D = res;
}
void opSET_3_E(State* self) {
  // 0xdb - SET 3,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = setBit(dst, 3);
  E = res;
}
void opSET_3_H(State* self) {
  // 0xdc - SET 3,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = setBit(dst, 3);
  H = res;
}
void opSET_3_L(State* self) {
  // 0xdd - SET 3,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = setBit(dst, 3);
  L = res;
}
void opSET_3_deref_HL(State* self) {
  // 0xde - SET 3,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = setBit(dst, 3);
  write8(self->bus, HLp, res, true);
}
void opSET_3_A(State* self) {
  // 0xdf - SET 3,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = setBit(dst, 3);
  A = res;
}
void opSET_4_B(State* self) {
  // 0xe0 - SET 4,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = setBit(dst, 4);
  B = res;
}
void opSET_4_C(State* self) {
  // 0xe1 - SET 4,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = setBit(dst, 4);
  C = res;
}
void opSET_4_D(State* self) {
  // 0xe2 - SET 4,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = setBit(dst, 4);
  D = res;
}
void opSET_4_E(State* self) {
  // 0xe3 - SET 4,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = setBit(dst, 4);
  E = res;
}
void opSET_4_H(State* self) {
  // 0xe4 - SET 4,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = setBit(dst, 4);
  H = res;
}
void opSET_4_L(State* self) {
  // 0xe5 - SET 4,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = setBit(dst, 4);
  L = res;
}
void opSET_4_deref_HL(State* self) {
  // 0xe6 - SET 4,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = setBit(dst, 4);
  write8(self->bus, HLp, res, true);
}
void opSET_4_A(State* self) {
  // 0xe7 - SET 4,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = setBit(dst, 4);
  A = res;
}
void opSET_5_B(State* self) {
  // 0xe8 - SET 5,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = setBit(dst, 5);
  B = res;
}
void opSET_5_C(State* self) {
  // 0xe9 - SET 5,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = setBit(dst, 5);
  C = res;
}
void opSET_5_D(State* self) {
  // 0xea - SET 5,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = setBit(dst, 5);
  D = res;
}
void opSET_5_E(State* self) {
  // 0xeb - SET 5,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = setBit(dst, 5);
  E = res;
}
void opSET_5_H(State* self) {
  // 0xec - SET 5,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = setBit(dst, 5);
  H = res;
}
void opSET_5_L(State* self) {
  // 0xed - SET 5,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = setBit(dst, 5);
  L = res;
}
void opSET_5_deref_HL(State* self) {
  // 0xee - SET 5,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = setBit(dst, 5);
  write8(self->bus, HLp, res, true);
}
void opSET_5_A(State* self) {
  // 0xef - SET 5,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = setBit(dst, 5);
  A = res;
}
void opSET_6_B(State* self) {
  // 0xf0 - SET 6,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = setBit(dst, 6);
  B = res;
}
void opSET_6_C(State* self) {
  // 0xf1 - SET 6,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = setBit(dst, 6);
  C = res;
}
void opSET_6_D(State* self) {
  // 0xf2 - SET 6,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = setBit(dst, 6);
  D = res;
}
void opSET_6_E(State* self) {
  // 0xf3 - SET 6,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = setBit(dst, 6);
  E = res;
}
void opSET_6_H(State* self) {
  // 0xf4 - SET 6,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = setBit(dst, 6);
  H = res;
}
void opSET_6_L(State* self) {
  // 0xf5 - SET 6,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = setBit(dst, 6);
  L = res;
}
void opSET_6_deref_HL(State* self) {
  // 0xf6 - SET 6,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = setBit(dst, 6);
  write8(self->bus, HLp, res, true);
}
void opSET_6_A(State* self) {
  // 0xf7 - SET 6,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = setBit(dst, 6);
  A = res;
}
void opSET_7_B(State* self) {
  // 0xf8 - SET 7,B (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = B;
  uint8_t res = setBit(dst, 7);
  B = res;
}
void opSET_7_C(State* self) {
  // 0xf9 - SET 7,C (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = C;
  uint8_t res = setBit(dst, 7);
  C = res;
}
void opSET_7_D(State* self) {
  // 0xfa - SET 7,D (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = D;
  uint8_t res = setBit(dst, 7);
  D = res;
}
void opSET_7_E(State* self) {
  // 0xfb - SET 7,E (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = E;
  uint8_t res = setBit(dst, 7);
  E = res;
}
void opSET_7_H(State* self) {
  // 0xfc - SET 7,H (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = H;
  uint8_t res = setBit(dst, 7);
  H = res;
}
void opSET_7_L(State* self) {
  // 0xfd - SET 7,L (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = L;
  uint8_t res = setBit(dst, 7);
  L = res;
}
void opSET_7_deref_HL(State* self) {
  // 0xfe - SET 7,(HL) (x8/rsb) < T(16) | Tbr(16) >
  uint8_t dst = read8(self->bus, HLp, true, true);
  uint8_t res = setBit(dst, 7);
  write8(self->bus, HLp, res, true);
}
void opSET_7_A(State* self) {
  // 0xff - SET 7,A (x8/rsb) < T(8) | Tbr(8) >
  uint8_t dst = A;
  uint8_t res = setBit(dst, 7);
  A = res;
}

// --------------- Lookup Tables -----------------

void attachPrefixed(Backend* self) {
  fnptr* LUT = self->PREFIXED;
  LUT[0x0] = opRLC_B;
  LUT[0x1] = opRLC_C;
  LUT[0x2] = opRLC_D;
  LUT[0x3] = opRLC_E;
  LUT[0x4] = opRLC_H;
  LUT[0x5] = opRLC_L;
  LUT[0x6] = opRLC_deref_HL;
  LUT[0x7] = opRLC_A;
  LUT[0x8] = opRRC_B;
  LUT[0x9] = opRRC_C;
  LUT[0xa] = opRRC_D;
  LUT[0xb] = opRRC_E;
  LUT[0xc] = opRRC_H;
  LUT[0xd] = opRRC_L;
  LUT[0xe] = opRRC_deref_HL;
  LUT[0xf] = opRRC_A;
  LUT[0x10] = opRL_B;
  LUT[0x11] = opRL_C;
  LUT[0x12] = opRL_D;
  LUT[0x13] = opRL_E;
  LUT[0x14] = opRL_H;
  LUT[0x15] = opRL_L;
  LUT[0x16] = opRL_deref_HL;
  LUT[0x17] = opRL_A;
  LUT[0x18] = opRR_B;
  LUT[0x19] = opRR_C;
  LUT[0x1a] = opRR_D;
  LUT[0x1b] = opRR_E;
  LUT[0x1c] = opRR_H;
  LUT[0x1d] = opRR_L;
  LUT[0x1e] = opRR_deref_HL;
  LUT[0x1f] = opRR_A;
  LUT[0x20] = opSLA_B;
  LUT[0x21] = opSLA_C;
  LUT[0x22] = opSLA_D;
  LUT[0x23] = opSLA_E;
  LUT[0x24] = opSLA_H;
  LUT[0x25] = opSLA_L;
  LUT[0x26] = opSLA_deref_HL;
  LUT[0x27] = opSLA_A;
  LUT[0x28] = opSRA_B;
  LUT[0x29] = opSRA_C;
  LUT[0x2a] = opSRA_D;
  LUT[0x2b] = opSRA_E;
  LUT[0x2c] = opSRA_H;
  LUT[0x2d] = opSRA_L;
  LUT[0x2e] = opSRA_deref_HL;
  LUT[0x2f] = opSRA_A;
  LUT[0x30] = opSWAP_B;
  LUT[0x31] = opSWAP_C;
  LUT[0x32] = opSWAP_D;
  LUT[0x33] = opSWAP_E;
  LUT[0x34] = opSWAP_H;
  LUT[0x35] = opSWAP_L;
  LUT[0x36] = opSWAP_deref_HL;
  LUT[0x37] = opSWAP_A;
  LUT[0x38] = opSRL_B;
  LUT[0x39] = opSRL_C;
  LUT[0x3a] = opSRL_D;
  LUT[0x3b] = opSRL_E;
  LUT[0x3c] = opSRL_H;
  LUT[0x3d] = opSRL_L;
  LUT[0x3e] = opSRL_deref_HL;
  LUT[0x3f] = opSRL_A;
  LUT[0x40] = opBIT_0_B;
  LUT[0x41] = opBIT_0_C;
  LUT[0x42] = opBIT_0_D;
  LUT[0x43] = opBIT_0_E;
  LUT[0x44] = opBIT_0_H;
  LUT[0x45] = opBIT_0_L;
  LUT[0x46] = opBIT_0_deref_HL;
  LUT[0x47] = opBIT_0_A;
  LUT[0x48] = opBIT_1_B;
  LUT[0x49] = opBIT_1_C;
  LUT[0x4a] = opBIT_1_D;
  LUT[0x4b] = opBIT_1_E;
  LUT[0x4c] = opBIT_1_H;
  LUT[0x4d] = opBIT_1_L;
  LUT[0x4e] = opBIT_1_deref_HL;
  LUT[0x4f] = opBIT_1_A;
  LUT[0x50] = opBIT_2_B;
  LUT[0x51] = opBIT_2_C;
  LUT[0x52] = opBIT_2_D;
  LUT[0x53] = opBIT_2_E;
  LUT[0x54] = opBIT_2_H;
  LUT[0x55] = opBIT_2_L;
  LUT[0x56] = opBIT_2_deref_HL;
  LUT[0x57] = opBIT_2_A;
  LUT[0x58] = opBIT_3_B;
  LUT[0x59] = opBIT_3_C;
  LUT[0x5a] = opBIT_3_D;
  LUT[0x5b] = opBIT_3_E;
  LUT[0x5c] = opBIT_3_H;
  LUT[0x5d] = opBIT_3_L;
  LUT[0x5e] = opBIT_3_deref_HL;
  LUT[0x5f] = opBIT_3_A;
  LUT[0x60] = opBIT_4_B;
  LUT[0x61] = opBIT_4_C;
  LUT[0x62] = opBIT_4_D;
  LUT[0x63] = opBIT_4_E;
  LUT[0x64] = opBIT_4_H;
  LUT[0x65] = opBIT_4_L;
  LUT[0x66] = opBIT_4_deref_HL;
  LUT[0x67] = opBIT_4_A;
  LUT[0x68] = opBIT_5_B;
  LUT[0x69] = opBIT_5_C;
  LUT[0x6a] = opBIT_5_D;
  LUT[0x6b] = opBIT_5_E;
  LUT[0x6c] = opBIT_5_H;
  LUT[0x6d] = opBIT_5_L;
  LUT[0x6e] = opBIT_5_deref_HL;
  LUT[0x6f] = opBIT_5_A;
  LUT[0x70] = opBIT_6_B;
  LUT[0x71] = opBIT_6_C;
  LUT[0x72] = opBIT_6_D;
  LUT[0x73] = opBIT_6_E;
  LUT[0x74] = opBIT_6_H;
  LUT[0x75] = opBIT_6_L;
  LUT[0x76] = opBIT_6_deref_HL;
  LUT[0x77] = opBIT_6_A;
  LUT[0x78] = opBIT_7_B;
  LUT[0x79] = opBIT_7_C;
  LUT[0x7a] = opBIT_7_D;
  LUT[0x7b] = opBIT_7_E;
  LUT[0x7c] = opBIT_7_H;
  LUT[0x7d] = opBIT_7_L;
  LUT[0x7e] = opBIT_7_deref_HL;
  LUT[0x7f] = opBIT_7_A;
  LUT[0x80] = opRES_0_B;
  LUT[0x81] = opRES_0_C;
  LUT[0x82] = opRES_0_D;
  LUT[0x83] = opRES_0_E;
  LUT[0x84] = opRES_0_H;
  LUT[0x85] = opRES_0_L;
  LUT[0x86] = opRES_0_deref_HL;
  LUT[0x87] = opRES_0_A;
  LUT[0x88] = opRES_1_B;
  LUT[0x89] = opRES_1_C;
  LUT[0x8a] = opRES_1_D;
  LUT[0x8b] = opRES_1_E;
  LUT[0x8c] = opRES_1_H;
  LUT[0x8d] = opRES_1_L;
  LUT[0x8e] = opRES_1_deref_HL;
  LUT[0x8f] = opRES_1_A;
  LUT[0x90] = opRES_2_B;
  LUT[0x91] = opRES_2_C;
  LUT[0x92] = opRES_2_D;
  LUT[0x93] = opRES_2_E;
  LUT[0x94] = opRES_2_H;
  LUT[0x95] = opRES_2_L;
  LUT[0x96] = opRES_2_deref_HL;
  LUT[0x97] = opRES_2_A;
  LUT[0x98] = opRES_3_B;
  LUT[0x99] = opRES_3_C;
  LUT[0x9a] = opRES_3_D;
  LUT[0x9b] = opRES_3_E;
  LUT[0x9c] = opRES_3_H;
  LUT[0x9d] = opRES_3_L;
  LUT[0x9e] = opRES_3_deref_HL;
  LUT[0x9f] = opRES_3_A;
  LUT[0xa0] = opRES_4_B;
  LUT[0xa1] = opRES_4_C;
  LUT[0xa2] = opRES_4_D;
  LUT[0xa3] = opRES_4_E;
  LUT[0xa4] = opRES_4_H;
  LUT[0xa5] = opRES_4_L;
  LUT[0xa6] = opRES_4_deref_HL;
  LUT[0xa7] = opRES_4_A;
  LUT[0xa8] = opRES_5_B;
  LUT[0xa9] = opRES_5_C;
  LUT[0xaa] = opRES_5_D;
  LUT[0xab] = opRES_5_E;
  LUT[0xac] = opRES_5_H;
  LUT[0xad] = opRES_5_L;
  LUT[0xae] = opRES_5_deref_HL;
  LUT[0xaf] = opRES_5_A;
  LUT[0xb0] = opRES_6_B;
  LUT[0xb1] = opRES_6_C;
  LUT[0xb2] = opRES_6_D;
  LUT[0xb3] = opRES_6_E;
  LUT[0xb4] = opRES_6_H;
  LUT[0xb5] = opRES_6_L;
  LUT[0xb6] = opRES_6_deref_HL;
  LUT[0xb7] = opRES_6_A;
  LUT[0xb8] = opRES_7_B;
  LUT[0xb9] = opRES_7_C;
  LUT[0xba] = opRES_7_D;
  LUT[0xbb] = opRES_7_E;
  LUT[0xbc] = opRES_7_H;
  LUT[0xbd] = opRES_7_L;
  LUT[0xbe] = opRES_7_deref_HL;
  LUT[0xbf] = opRES_7_A;
  LUT[0xc0] = opSET_0_B;
  LUT[0xc1] = opSET_0_C;
  LUT[0xc2] = opSET_0_D;
  LUT[0xc3] = opSET_0_E;
  LUT[0xc4] = opSET_0_H;
  LUT[0xc5] = opSET_0_L;
  LUT[0xc6] = opSET_0_deref_HL;
  LUT[0xc7] = opSET_0_A;
  LUT[0xc8] = opSET_1_B;
  LUT[0xc9] = opSET_1_C;
  LUT[0xca] = opSET_1_D;
  LUT[0xcb] = opSET_1_E;
  LUT[0xcc] = opSET_1_H;
  LUT[0xcd] = opSET_1_L;
  LUT[0xce] = opSET_1_deref_HL;
  LUT[0xcf] = opSET_1_A;
  LUT[0xd0] = opSET_2_B;
  LUT[0xd1] = opSET_2_C;
  LUT[0xd2] = opSET_2_D;
  LUT[0xd3] = opSET_2_E;
  LUT[0xd4] = opSET_2_H;
  LUT[0xd5] = opSET_2_L;
  LUT[0xd6] = opSET_2_deref_HL;
  LUT[0xd7] = opSET_2_A;
  LUT[0xd8] = opSET_3_B;
  LUT[0xd9] = opSET_3_C;
  LUT[0xda] = opSET_3_D;
  LUT[0xdb] = opSET_3_E;
  LUT[0xdc] = opSET_3_H;
  LUT[0xdd] = opSET_3_L;
  LUT[0xde] = opSET_3_deref_HL;
  LUT[0xdf] = opSET_3_A;
  LUT[0xe0] = opSET_4_B;
  LUT[0xe1] = opSET_4_C;
  LUT[0xe2] = opSET_4_D;
  LUT[0xe3] = opSET_4_E;
  LUT[0xe4] = opSET_4_H;
  LUT[0xe5] = opSET_4_L;
  LUT[0xe6] = opSET_4_deref_HL;
  LUT[0xe7] = opSET_4_A;
  LUT[0xe8] = opSET_5_B;
  LUT[0xe9] = opSET_5_C;
  LUT[0xea] = opSET_5_D;
  LUT[0xeb] = opSET_5_E;
  LUT[0xec] = opSET_5_H;
  LUT[0xed] = opSET_5_L;
  LUT[0xee] = opSET_5_deref_HL;
  LUT[0xef] = opSET_5_A;
  LUT[0xf0] = opSET_6_B;
  LUT[0xf1] = opSET_6_C;
  LUT[0xf2] = opSET_6_D;
  LUT[0xf3] = opSET_6_E;
  LUT[0xf4] = opSET_6_H;
  LUT[0xf5] = opSET_6_L;
  LUT[0xf6] = opSET_6_deref_HL;
  LUT[0xf7] = opSET_6_A;
  LUT[0xf8] = opSET_7_B;
  LUT[0xf9] = opSET_7_C;
  LUT[0xfa] = opSET_7_D;
  LUT[0xfb] = opSET_7_E;
  LUT[0xfc] = opSET_7_H;
  LUT[0xfd] = opSET_7_L;
  LUT[0xfe] = opSET_7_deref_HL;
  LUT[0xff] = opSET_7_A;
}

void attachUnPrefixed(Backend* self) {
  fnptr* LUT = self->UNPREFIXED;
  LUT[0x0] = opNOP;
  LUT[0x1] = opLD_BC_U16;
  LUT[0x2] = opLD_deref_BC_A;
  LUT[0x3] = opINC_BC;
  LUT[0x4] = opINC_B;
  LUT[0x5] = opDEC_B;
  LUT[0x6] = opLD_B_U8;
  LUT[0x7] = opRLCA;
  LUT[0x8] = opLD_deref_U16_SP;
  LUT[0x9] = opADD_HL_BC;
  LUT[0xa] = opLD_A_deref_BC;
  LUT[0xb] = opDEC_BC;
  LUT[0xc] = opINC_C;
  LUT[0xd] = opDEC_C;
  LUT[0xe] = opLD_C_U8;
  LUT[0xf] = opRRCA;
  LUT[0x10] = opSTOP;
  LUT[0x11] = opLD_DE_U16;
  LUT[0x12] = opLD_deref_DE_A;
  LUT[0x13] = opINC_DE;
  LUT[0x14] = opINC_D;
  LUT[0x15] = opDEC_D;
  LUT[0x16] = opLD_D_U8;
  LUT[0x17] = opRLA;
  LUT[0x18] = opJR_I8;
  LUT[0x19] = opADD_HL_DE;
  LUT[0x1a] = opLD_A_deref_DE;
  LUT[0x1b] = opDEC_DE;
  LUT[0x1c] = opINC_E;
  LUT[0x1d] = opDEC_E;
  LUT[0x1e] = opLD_E_U8;
  LUT[0x1f] = opRRA;
  LUT[0x20] = opJR_NZ_I8;
  LUT[0x21] = opLD_HL_U16;
  LUT[0x22] = opLD_deref_HLi_A;
  LUT[0x23] = opINC_HL;
  LUT[0x24] = opINC_H;
  LUT[0x25] = opDEC_H;
  LUT[0x26] = opLD_H_U8;
  LUT[0x27] = opDAA;
  LUT[0x28] = opJR_Z_I8;
  LUT[0x29] = opADD_HL_HL;
  LUT[0x2a] = opLD_A_deref_HLi;
  LUT[0x2b] = opDEC_HL;
  LUT[0x2c] = opINC_L;
  LUT[0x2d] = opDEC_L;
  LUT[0x2e] = opLD_L_U8;
  LUT[0x2f] = opCPL;
  LUT[0x30] = opJR_NC_I8;
  LUT[0x31] = opLD_SP_U16;
  LUT[0x32] = opLD_deref_HLd_A;
  LUT[0x33] = opINC_SP;
  LUT[0x34] = opINC_deref_HL;
  LUT[0x35] = opDEC_deref_HL;
  LUT[0x36] = opLD_deref_HL_U8;
  LUT[0x37] = opSCF;
  LUT[0x38] = opJR_C_I8;
  LUT[0x39] = opADD_HL_SP;
  LUT[0x3a] = opLD_A_deref_HLd;
  LUT[0x3b] = opDEC_SP;
  LUT[0x3c] = opINC_A;
  LUT[0x3d] = opDEC_A;
  LUT[0x3e] = opLD_A_U8;
  LUT[0x3f] = opCCF;
  LUT[0x40] = opLD_B_B;
  LUT[0x41] = opLD_B_C;
  LUT[0x42] = opLD_B_D;
  LUT[0x43] = opLD_B_E;
  LUT[0x44] = opLD_B_H;
  LUT[0x45] = opLD_B_L;
  LUT[0x46] = opLD_B_deref_HL;
  LUT[0x47] = opLD_B_A;
  LUT[0x48] = opLD_C_B;
  LUT[0x49] = opLD_C_C;
  LUT[0x4a] = opLD_C_D;
  LUT[0x4b] = opLD_C_E;
  LUT[0x4c] = opLD_C_H;
  LUT[0x4d] = opLD_C_L;
  LUT[0x4e] = opLD_C_deref_HL;
  LUT[0x4f] = opLD_C_A;
  LUT[0x50] = opLD_D_B;
  LUT[0x51] = opLD_D_C;
  LUT[0x52] = opLD_D_D;
  LUT[0x53] = opLD_D_E;
  LUT[0x54] = opLD_D_H;
  LUT[0x55] = opLD_D_L;
  LUT[0x56] = opLD_D_deref_HL;
  LUT[0x57] = opLD_D_A;
  LUT[0x58] = opLD_E_B;
  LUT[0x59] = opLD_E_C;
  LUT[0x5a] = opLD_E_D;
  LUT[0x5b] = opLD_E_E;
  LUT[0x5c] = opLD_E_H;
  LUT[0x5d] = opLD_E_L;
  LUT[0x5e] = opLD_E_deref_HL;
  LUT[0x5f] = opLD_E_A;
  LUT[0x60] = opLD_H_B;
  LUT[0x61] = opLD_H_C;
  LUT[0x62] = opLD_H_D;
  LUT[0x63] = opLD_H_E;
  LUT[0x64] = opLD_H_H;
  LUT[0x65] = opLD_H_L;
  LUT[0x66] = opLD_H_deref_HL;
  LUT[0x67] = opLD_H_A;
  LUT[0x68] = opLD_L_B;
  LUT[0x69] = opLD_L_C;
  LUT[0x6a] = opLD_L_D;
  LUT[0x6b] = opLD_L_E;
  LUT[0x6c] = opLD_L_H;
  LUT[0x6d] = opLD_L_L;
  LUT[0x6e] = opLD_L_deref_HL;
  LUT[0x6f] = opLD_L_A;
  LUT[0x70] = opLD_deref_HL_B;
  LUT[0x71] = opLD_deref_HL_C;
  LUT[0x72] = opLD_deref_HL_D;
  LUT[0x73] = opLD_deref_HL_E;
  LUT[0x74] = opLD_deref_HL_H;
  LUT[0x75] = opLD_deref_HL_L;
  LUT[0x76] = opHALT;
  LUT[0x77] = opLD_deref_HL_A;
  LUT[0x78] = opLD_A_B;
  LUT[0x79] = opLD_A_C;
  LUT[0x7a] = opLD_A_D;
  LUT[0x7b] = opLD_A_E;
  LUT[0x7c] = opLD_A_H;
  LUT[0x7d] = opLD_A_L;
  LUT[0x7e] = opLD_A_deref_HL;
  LUT[0x7f] = opLD_A_A;
  LUT[0x80] = opADD_A_B;
  LUT[0x81] = opADD_A_C;
  LUT[0x82] = opADD_A_D;
  LUT[0x83] = opADD_A_E;
  LUT[0x84] = opADD_A_H;
  LUT[0x85] = opADD_A_L;
  LUT[0x86] = opADD_A_deref_HL;
  LUT[0x87] = opADD_A_A;
  LUT[0x88] = opADC_A_B;
  LUT[0x89] = opADC_A_C;
  LUT[0x8a] = opADC_A_D;
  LUT[0x8b] = opADC_A_E;
  LUT[0x8c] = opADC_A_H;
  LUT[0x8d] = opADC_A_L;
  LUT[0x8e] = opADC_A_deref_HL;
  LUT[0x8f] = opADC_A_A;
  LUT[0x90] = opSUB_A_B;
  LUT[0x91] = opSUB_A_C;
  LUT[0x92] = opSUB_A_D;
  LUT[0x93] = opSUB_A_E;
  LUT[0x94] = opSUB_A_H;
  LUT[0x95] = opSUB_A_L;
  LUT[0x96] = opSUB_A_deref_HL;
  LUT[0x97] = opSUB_A_A;
  LUT[0x98] = opSBC_A_B;
  LUT[0x99] = opSBC_A_C;
  LUT[0x9a] = opSBC_A_D;
  LUT[0x9b] = opSBC_A_E;
  LUT[0x9c] = opSBC_A_H;
  LUT[0x9d] = opSBC_A_L;
  LUT[0x9e] = opSBC_A_deref_HL;
  LUT[0x9f] = opSBC_A_A;
  LUT[0xa0] = opAND_A_B;
  LUT[0xa1] = opAND_A_C;
  LUT[0xa2] = opAND_A_D;
  LUT[0xa3] = opAND_A_E;
  LUT[0xa4] = opAND_A_H;
  LUT[0xa5] = opAND_A_L;
  LUT[0xa6] = opAND_A_deref_HL;
  LUT[0xa7] = opAND_A_A;
  LUT[0xa8] = opXOR_A_B;
  LUT[0xa9] = opXOR_A_C;
  LUT[0xaa] = opXOR_A_D;
  LUT[0xab] = opXOR_A_E;
  LUT[0xac] = opXOR_A_H;
  LUT[0xad] = opXOR_A_L;
  LUT[0xae] = opXOR_A_deref_HL;
  LUT[0xaf] = opXOR_A_A;
  LUT[0xb0] = opOR_A_B;
  LUT[0xb1] = opOR_A_C;
  LUT[0xb2] = opOR_A_D;
  LUT[0xb3] = opOR_A_E;
  LUT[0xb4] = opOR_A_H;
  LUT[0xb5] = opOR_A_L;
  LUT[0xb6] = opOR_A_deref_HL;
  LUT[0xb7] = opOR_A_A;
  LUT[0xb8] = opCP_A_B;
  LUT[0xb9] = opCP_A_C;
  LUT[0xba] = opCP_A_D;
  LUT[0xbb] = opCP_A_E;
  LUT[0xbc] = opCP_A_H;
  LUT[0xbd] = opCP_A_L;
  LUT[0xbe] = opCP_A_deref_HL;
  LUT[0xbf] = opCP_A_A;
  LUT[0xc0] = opRET_NZ;
  LUT[0xc1] = opPOP_BC;
  LUT[0xc2] = opJP_NZ_U16;
  LUT[0xc3] = opJP_U16;
  LUT[0xc4] = opCALL_NZ_U16;
  LUT[0xc5] = opPUSH_BC;
  LUT[0xc6] = opADD_A_U8;
  LUT[0xc7] = opRST_00H;
  LUT[0xc8] = opRET_Z;
  LUT[0xc9] = opRET;
  LUT[0xca] = opJP_Z_U16;
  LUT[0xcc] = opCALL_Z_U16;
  LUT[0xcd] = opCALL_U16;
  LUT[0xce] = opADC_A_U8;
  LUT[0xcf] = opRST_08H;
  LUT[0xd0] = opRET_NC;
  LUT[0xd1] = opPOP_DE;
  LUT[0xd2] = opJP_NC_U16;
  LUT[0xd4] = opCALL_NC_U16;
  LUT[0xd5] = opPUSH_DE;
  LUT[0xd6] = opSUB_A_U8;
  LUT[0xd7] = opRST_10H;
  LUT[0xd8] = opRET_C;
  LUT[0xd9] = opRETI;
  LUT[0xda] = opJP_C_U16;
  LUT[0xdc] = opCALL_C_U16;
  LUT[0xde] = opSBC_A_U8;
  LUT[0xdf] = opRST_18H;
  LUT[0xe0] = opLD_deref_FF00_U8_A;
  LUT[0xe1] = opPOP_HL;
  LUT[0xe2] = opLD_deref_FF00_C_A;
  LUT[0xe5] = opPUSH_HL;
  LUT[0xe6] = opAND_A_U8;
  LUT[0xe7] = opRST_20H;
  LUT[0xe8] = opADD_SP_I8;
  LUT[0xe9] = opJP_HL;
  LUT[0xea] = opLD_deref_U16_A;
  LUT[0xee] = opXOR_A_U8;
  LUT[0xef] = opRST_28H;
  LUT[0xf0] = opLD_A_deref_FF00_U8;
  LUT[0xf1] = opPOP_AF;
  LUT[0xf2] = opLD_A_deref_FF00_C;
  LUT[0xf3] = opDI;
  LUT[0xf5] = opPUSH_AF;
  LUT[0xf6] = opOR_A_U8;
  LUT[0xf7] = opRST_30H;
  LUT[0xf8] = opLD_HL_SP_I8;
  LUT[0xf9] = opLD_SP_HL;
  LUT[0xfa] = opLD_A_deref_U16;
  LUT[0xfb] = opEI;
  LUT[0xfe] = opCP_A_U8;
  LUT[0xff] = opRST_38H;
}

#undef AFp
#undef BCp
#undef DEp
#undef HLp
#undef SP
#undef PC
#undef A
#undef F
#undef B
#undef C
#undef D
#undef E
#undef H
#undef L
#undef COPY_FLAG
#undef Cf
#undef Hf
#undef Nf
#undef Zf
#undef getFlag

Backend createBackend(Bus* bus) {
  Registers regs;
  regs.AF.hi = 0x01;
  regs.AF.lo = 0xB0;
  regs.BC.hi = 0x00;
  regs.BC.lo = 0x13;
  regs.DE.hi = 0x00;
  regs.DE.lo = 0xD8;
  regs.HL.hi = 0x01;
  regs.HL.lo = 0x4D;
  regs.SP = 0xFFFE;
  State state = {.bus = bus,
                 .regs = regs,
                 .opcode = 0x00,
                 .halted = false,
                 .IME = false,
                 .IMERising = false};

  Backend backend = {.state = state};
  attachPrefixed(&backend);
  attachUnPrefixed(&backend);
  return backend;
}

void backendTick(Backend* self) {
  self->state.opcode = next8(&self->state);
  if (self->state.opcode == 0xCB) {
    uint8_t next = next8(&self->state);
    self->PREFIXED[next](&self->state);

  } else {
    self->UNPREFIXED[self->state.opcode](&self->state);
  }
}
