#ifndef _DBG_API_H_
#define _DBG_API_H_

#include <sys/types.h>
#include <stdint.h>



// API for GDB
int dbg_wb_read32(unsigned long adr, unsigned long *data);
int dbg_wb_write32(unsigned long adr, unsigned long data);
int dbg_wb_write16(unsigned long adr, uint16_t data);
int dbg_wb_write8(unsigned long adr, uint8_t data);
int dbg_wb_read_block32(unsigned long adr, unsigned long *data, int len);
int dbg_wb_read_block16(unsigned long adr, uint16_t *data, int len);
int dbg_wb_read_block8(unsigned long adr, uint8_t *data, int len);
int dbg_wb_write_block32(unsigned long adr, unsigned long *data, int len);
int dbg_wb_write_block16(unsigned long adr, uint16_t *data, int len);
int dbg_wb_write_block8(unsigned long adr, uint8_t *data, int len);
int dbg_cpu0_read(unsigned long adr, unsigned long *data);
int dbg_cpu0_read_block(unsigned long adr, unsigned long *data, int count);
int dbg_cpu0_write(unsigned long adr, unsigned long data);
int dbg_cpu0_write_block(unsigned long adr, unsigned long *data, int count);
int dbg_cpu0_write_ctrl(unsigned long adr, unsigned char data);
int dbg_cpu0_read_ctrl(unsigned long adr, unsigned char *data);
//int dbg_cpu1_read(unsigned long adr, unsigned long *data);
//int dbg_cpu1_write(unsigned long adr, unsigned long data);
//int dbg_cpu1_write_reg(unsigned long adr, unsigned char data);
//int dbg_cpu1_read_ctrl(unsigned long adr, unsigned char *data);

#endif
