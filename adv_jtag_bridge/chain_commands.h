#ifndef _CHAIN_COMMANDS_H_
#define _CHAIN_COMMANDS_H_

#include <sys/types.h>  // for uint32_t

// Discover devices on JTAG chain
int jtag_enumerate_chain(unsigned long **id_array, int *num_devices);

// Functions to set configuration for the JTAG chain
void config_set_IR_size(int size);
void config_set_IR_prefix_bits(int bits);
void config_set_IR_postfix_bits(int bits);
void config_set_DR_prefix_bits(int bits);
void config_set_DR_postfix_bits(int bits);
void config_set_debug_cmd(unsigned int cmd);
void config_set_alt_vjtag(unsigned char enable);
void config_set_vjtag_cmd_vir(unsigned int cmd);
void config_set_vjtag_cmd_vdr(unsigned int cmd);
void config_set_xilinx_bscan(unsigned char enable);

// Operations on the JTAG TAP
int tap_reset(void);
int tap_enable_debug_module(void);
int tap_set_ir(int ir);

// JTAG operations
int jtag_get_idcode(uint32_t cmd, uint32_t *idcode);

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
