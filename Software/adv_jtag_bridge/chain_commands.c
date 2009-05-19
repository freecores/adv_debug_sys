/* jtag_bridge.c -- JTAG protocol bridge between GDB and Advanced debug module.
   Copyright(C) Nathan Yawn, nyawn@opencores.net
   based on code from jp2 by Marko Mlinar, markom@opencores.org
   
   This file contains functions which perform high-level transactions
   on a JTAG chain and debug unit, such as setting a value in the TAP IR
   or doing a burst write through the wishbone module of the debug unit.
   It uses the protocol for the Advanced Debug Interface (adv_dbg_if).
   
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA. 
*/



#include <stdio.h>
#include <stdlib.h>  // for malloc()
#include <unistd.h>  // for usleep()
#include <pthread.h>  // for mutexes

#include "chain_commands.h"  // For the return error codes
#include "adv_debug_module.h"     // hardware-specific defines for the debug module
#include "altera_virtual_jtag.h"  // hardware-specifg defines for the Altera Virtual JTAG interface
#include "cable_common.h"         // low-level JTAG IO routines
#include "errcodes.h"

#define debug(...) //fprintf(stderr, __VA_ARGS__ )

// Polynomial for the CRC calculation
// Yes, it's backwards.  Yes, this is on purpose.
// The hardware is designed this way to save on logic and routing,
// and it's really all the same to us here.
#define DBG_CRC_POLY 0xedb88320

// How many tries before an abort
#define NUM_SOFT_RETRIES 5

// How many '0' status bits to get during a burst read
// before giving up
#define MAX_READ_STATUS_WAIT 100

// wait for 100ms
#define JTAG_RETRY_WAIT() usleep(100000);

// Mutex for access to the JTAG cable.  Makes API calls threadsafe.
pthread_mutex_t dbg_access_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Currently selected scan chain in the debug unit - just to prevent unnecessary
   transfers. */
int current_chain = -1;
int desired_chain = -1;

// Currently selected internal register in each module
// - cuts down on unnecessary transfers
int current_reg_idx[DBG_MAX_MODULES];

/* The chain that should be currently selected. */
//static int dbg_chain = -1;

// Retry data
int soft_retry_no = 0;
//static int hard_retry_no = 0;

// Configuration data
int global_IR_size = 0;
int global_IR_prefix_bits = 0;
int global_IR_postfix_bits = 0;
int global_DR_prefix_bits = 0;
int global_DR_postfix_bits = 0;
unsigned int global_jtag_cmd_debug = 0;        // Value to be shifted into the TAP IR to select the debug unit (unused for virtual jtag)
unsigned char global_altera_virtual_jtag = 0;  // Set true to use virtual jtag mode
unsigned int vjtag_cmd_vir = ALTERA_CYCLONE_CMD_VIR;  // virtual IR-shift command for altera devices, may be configured on command line
unsigned int vjtag_cmd_vdr = ALTERA_CYCLONE_CMD_VDR; // virtual DR-shift, ditto
unsigned char global_xilinx_bscan = 0;  // Set true if the hardware uses a Xilinx BSCAN_* device.

// Prototypes for local functions
uint32_t compute_crc(uint32_t crc_in, uint32_t data_in, int length_bits);
int retry_do(void);
void retry_ok(void);

int jtag_write_bit(uint8_t packet);
int jtag_read_write_bit(uint8_t packet, uint8_t *in_bit);
int jtag_write_stream(uint32_t *out_data, int length_bits, unsigned char set_TMS);
int jtag_read_write_stream(uint32_t *out_data, uint32_t *in_data, int length_bits, 
			   unsigned char adjust, unsigned char set_TMS);


int dbg_select_module(int chain);
int dbg_select_ctrl_reg(unsigned long regidx);
int dbg_ctrl_write(unsigned long regidx, uint32_t *cmd_data, int length_bits);
int dbg_ctrl_read(unsigned long regidx, uint32_t *data, int databits);
int dbg_burst_command(unsigned int opcode, unsigned long address, int length_words);
int dbg_wb_burst_read(int word_size_bytes, int word_count, unsigned long start_address, void *data);
int dbg_wb_burst_write(void *data, int word_size_bytes, int word_count, unsigned long start_address);



///////////////////////////////////////////////////////////////////////
// Configuration

void config_set_IR_size(int size) {
  global_IR_size = size;
}

void config_set_IR_prefix_bits(int bits) {
  global_IR_prefix_bits = bits;
}

void config_set_IR_postfix_bits(int bits) {
  global_IR_postfix_bits = bits;
}

void config_set_DR_prefix_bits(int bits) {
  global_DR_prefix_bits = bits;
}

void config_set_DR_postfix_bits(int bits) {
  global_DR_postfix_bits = bits;
}

void config_set_debug_cmd(unsigned int cmd) {
  global_jtag_cmd_debug = cmd;
}

void config_set_alt_vjtag(unsigned char enable) {
  global_altera_virtual_jtag = (enable) ? 1:0;
}

// At present, all devices which support virtual JTAG use the same VIR/VDR
// commands.  But, if they ever change, these can be changed on the command line.
void config_set_vjtag_cmd_vir(unsigned int cmd) {
  vjtag_cmd_vir = cmd;
}

void config_set_vjtag_cmd_vdr(unsigned int cmd) {
  vjtag_cmd_vdr = cmd;
}

void config_set_xilinx_bscan(unsigned char enable) {
  global_xilinx_bscan = (enable) ? 1:0;
}

//////////////////////////////////////////////////////////////////////
// Functions which operate on the JTAG TAP


/* Resets JTAG - Writes TRST=1, and TRST=0.  Sends 8 TMS to put the TAP
 * in test_logic_reset mode, for good measure.
 */
int tap_reset(void) {
  int i;
  int err = APP_ERR_NONE;

  debug("\nreset(");
  err |= jtag_write_bit(0);
  JTAG_RETRY_WAIT();
  /* In case we don't have TRST reset it manually */
  for(i = 0; i < 8; i++) err |= jtag_write_bit(TMS);
  err |= jtag_write_bit(TRST);  // if TRST not supported, this puts us in run test / idle
  JTAG_RETRY_WAIT();
  err |= jtag_write_bit(0);  // run test / idle
  debug(")\n");

  // Reset data on current module/register selections
  current_chain = -1;
  for(i = 0; i < DBG_MAX_MODULES; i++)
    current_reg_idx[i] = -1;

  return err;
}

  // Set the IR with the DEBUG command, one way or the other
int tap_enable_debug_module(void)
{
  uint32_t data;
 int err = APP_ERR_NONE;

  if(global_altera_virtual_jtag) {
    /* Set for virtual IR shift */
    err |= tap_set_ir(vjtag_cmd_vir);  // This is the altera virtual IR scan command
    err |= jtag_write_bit(TMS); /* SELECT_DR SCAN */
    err |= jtag_write_bit(0); /* CAPTURE_DR */
    err |= jtag_write_bit(0); /* SHIFT_DR */
    
    /* Select debug scan chain in  virtual IR */
    data = (0x1<<ALT_VJTAG_IR_SIZE)|ALT_VJTAG_CMD_DEBUG;
    err |= jtag_write_stream(&data, (ALT_VJTAG_IR_SIZE+1), 1);  // EXIT1_DR
    err |= jtag_write_bit(TMS); /* UPDATE_DR */
    err |= jtag_write_bit(0); /* IDLE */ 

    // This is a command to set an altera device to the "virtual DR shift" command
    err |= tap_set_ir(vjtag_cmd_vdr);
  }
  else {
    /* select debug scan chain and stay in it forever */
    err |= tap_set_ir(global_jtag_cmd_debug);
  }

  return err;
}

/* Moves a value into the TAP instruction register (IR)
 * Includes adjustment for scan chain IR length.
 */
uint32_t *ir_chain = NULL;

int tap_set_ir(int ir) {
  int chain_size;
  int chain_size_words;
  int i;
  int startoffset, startshift;
  int err = APP_ERR_NONE;
  
  // Adjust desired IR with prefix, postfix bits to set other devices in the chain to BYPASS
  chain_size = global_IR_size + global_IR_prefix_bits + global_IR_postfix_bits;
  chain_size_words = (chain_size/32)+1;

  if(ir_chain == NULL)  { // We have no way to know in advance how many bits there are in the combined IR register
    ir_chain = (uint32_t *) malloc(chain_size_words * sizeof(uint32_t));
    if(ir_chain == NULL)
      return APP_ERR_MALLOC;
  }

  for(i = 0; i < chain_size_words; i++)
    ir_chain[i] = 0xFFFFFFFF;  // Set all other devices to BYPASS

  // Copy the IR value into the output stream
  startoffset = global_IR_postfix_bits/32;
  startshift = (global_IR_postfix_bits - (startoffset*32));
  ir_chain[startoffset] &= (ir << startshift);
  ir_chain[startoffset] |= ~(0xFFFFFFFF << startshift);  // Put the 1's back in the LSB positions
  ir_chain[startoffset] |= (0xFFFFFFFF << (startshift + global_IR_size));  // Put 1's back in MSB positions, if any 
  if((startshift + global_IR_size) > 32) { // Deal with spill into the next word
    ir_chain[startoffset+1] &= ir >> (32-startshift);
    ir_chain[startoffset+1] |= (0xFFFFFFFF << (global_IR_size - (32-startshift)));  // Put the 1's back in the MSB positions
  }

  // Do the actual JTAG transaction
  debug("Set IR 0x%X\n", ir);
  err |= jtag_write_bit(TMS); /* SELECT_DR SCAN */
  err |= jtag_write_bit(TMS); /* SELECT_IR SCAN */

  err |= jtag_write_bit(0); /* CAPTURE_IR */
  err |= jtag_write_bit(0); /* SHIFT_IR */   

  /* write data, EXIT1_IR */
  debug("Setting IR, size %i, IR_size = %i, pre_size = %i, post_size = %i, data 0x%X\n", chain_size, global_IR_size, global_IR_prefix_bits, global_IR_postfix_bits, ir);
  err |= cable_write_stream(ir_chain, chain_size, 1);  // Use cable_ call directly (not jtag_), so we don't add DR prefix bits
  debug("Done setting IR\n");

  err |= jtag_write_bit(TMS); /* UPDATE_IR */
  err |= jtag_write_bit(0); /* IDLE */  
  current_chain = -1;
  return err;
}


// This assumes we are in the IDLE state, and we want to be in the SHIFT_DR state.
// We may need to do a little extra work, for Altera virtual JTAG...
int tap_set_shift_dr(void)
{
  int err = APP_ERR_NONE;

  // This always needs to be done
  err |= jtag_write_bit(TMS); /* SELECT_DR SCAN */
  err |= jtag_write_bit(0); /* CAPTURE_DR */
  err |= jtag_write_bit(0); /* SHIFT_DR */
  return err;
}


////////////////////////////////////////////////////////////////////
// Operations to read / write data over JTAG


/* Writes TCLK=0, TRST=1, TMS=bit1, TDI=bit0
   and    TCLK=1, TRST=1, TMS=bit1, TDI=bit0
*/
int jtag_write_bit(uint8_t packet) {
  debug("Wbit(%i)\n", packet);
  return cable_write_bit(packet);
}

int jtag_read_write_bit(uint8_t packet, uint8_t *in_bit) {
  int retval = cable_read_write_bit(packet, in_bit);
  debug("RWbit(%i,%i)", packet, *in_bit);
  return retval;
}

// This automatically adjusts for the DR length (other devices on scan chain)
// when the set_TMS flag is true.
int jtag_write_stream(uint32_t *out_data, int length_bits, unsigned char set_TMS)
{
  int i;
  int err = APP_ERR_NONE;

  if(!set_TMS)
    err |= cable_write_stream(out_data, length_bits, 0);
  else if(global_DR_prefix_bits == 0)
    err |= cable_write_stream(out_data, length_bits, 1);
  else {
    err |= cable_write_stream(out_data, length_bits, 0);
    // It could be faster to do a cable_write_stream for all the prefix bits (if >= 8 bits),
    // but we'd need a data array of unknown (and theoretically unlimited)
    // size to hold the 0 bits to write.
    for(i = 0; i < (global_DR_prefix_bits-1); i++)
      err |= jtag_write_bit(0);
    err |= jtag_write_bit(TMS);
  }
  return err;
}

// When set_TMS is true, this function insures the written data is in the desired position (past prefix bits)
// before sending TMS.  When 'adjust' is true, this function insures that the data read in accounts for postfix
// bits (they are shifted through before the read starts).
int jtag_read_write_stream(uint32_t *out_data, uint32_t *in_data, int length_bits, unsigned char adjust, unsigned char set_TMS)
{
  int i;
  int err = APP_ERR_NONE;

  if(adjust && (global_DR_postfix_bits > 0)) {
    // It would be faster to do a cable_write_stream for all the postfix bits,
    // but we'd need a data array of unknown (and theoretically unlimited)
    // size to hold the '0' bits to write.
    for(i = 0; i < global_DR_postfix_bits; i++)
      err |= cable_write_bit(0);
  }

  // If there are both prefix and postfix bits, we may shift more bits than strictly necessary.
  // If we shifted out the data while burning through the postfix bits, these shifts could be subtracted
  // from the number of prefix shifts.  However, that way leads to madness.
  if(!set_TMS)
    err |= cable_read_write_stream(out_data, in_data, length_bits, 0);  
  else if(global_DR_prefix_bits == 0)
    err |= cable_read_write_stream(out_data, in_data, length_bits, 1);  
  else {
    err |= cable_read_write_stream(out_data, in_data, length_bits, 0); 
    // It would be faster to do a cable_write_stream for all the prefix bits,
    // but we'd need a data array of unknown (and theoretically unlimited)
    // size to hold the '0' bits to write.
    for(i = 0; i < (global_DR_prefix_bits-1); i++)
      err |= jtag_write_bit(0);
    err |= jtag_write_bit(TMS);
  }
  return err;
}



// This function attempts to determine the structure of the JTAG chain
// It can determine how many devices are present.
// If the devices support the IDCODE command, it will be read and stored.
// There is no way to automatically determine the length of the IR registers - 
// this must be read from a BSDL file, if IDCODE is supported.
// When IDCODE is not supported, IR length of the target device must be entered on the command line.

#define ALLOC_SIZE 64
#define MAX_DEVICES 1024
int jtag_enumerate_chain(uint32_t **id_array, int *num_devices)
{
  uint32_t invalid_code = 0x7f;  // Shift this out, we know we're done when we get it back
  const unsigned int done_code = 0x3f;  // invalid_code is altered, we keep this for comparison (minus the start bit)
  int devindex = 0;  // which device we are currently trying to detect
  unsigned int tempID;
  uint32_t temp_manuf_code;
  uint32_t temp_rest_code;
  uint8_t start_bit = 0;
  unsigned long *idcodes;
  int reallocs = 0;
  int err = APP_ERR_NONE;

  // Malloc a reasonable number of entries, we'll expand if we must.  Linked lists are overrated.
  idcodes = (unsigned long *) malloc(ALLOC_SIZE*sizeof(unsigned long));
  if(idcodes == NULL) { 
    printf("Failed to allocate memory for device ID codes!\n"); 
    return APP_ERR_MALLOC;
  }

  // Put in SHIFT-DR mode
  err |= jtag_write_bit(TMS); /* SELECT_DR SCAN */
  err |= jtag_write_bit(0); /* CAPTURE_DR */
  err |= jtag_write_bit(0); /* SHIFT_DR */

  printf("Enumerating JTAG chain...\n");

  // Putting a limit on the # of devices supported has the useful side effect
  // of insuring we still exit in error cases (we never get the 0x7f manuf. id)
  while(devindex < MAX_DEVICES) {
    // get 1 bit. 0 = BYPASS, 1 = start of IDCODE
    err |= jtag_read_write_bit(invalid_code&0x01, &start_bit);
    invalid_code >>= 1;

    if(start_bit == 0) {
      if(devindex >= (ALLOC_SIZE << reallocs)) {  // Enlarge the memory array if necessary, double the size each time
	idcodes = (unsigned long *) realloc(idcodes, (ALLOC_SIZE << ++reallocs)*sizeof(unsigned long));
	if(idcodes == NULL) { 
	  printf("Failed to allocate memory for device ID codes during enumeration!\n"); 
	  return APP_ERR_MALLOC;
	}
      }
      idcodes[devindex] = -1;
      devindex++;
    }
    else {
      // get 11 bit manufacturer code
      err |= jtag_read_write_stream(&invalid_code, &temp_manuf_code, 11, 0, 0);
      invalid_code >>= 11;
      
      if(temp_manuf_code != done_code) {
	// get 20 more bits, rest of ID
	err |= jtag_read_write_stream(&invalid_code, &temp_rest_code, 20, 0, 0);
	invalid_code >>= 20;
	tempID = (temp_rest_code << 12) | (temp_manuf_code << 1) | 0x01;
	if(devindex >= (ALLOC_SIZE << reallocs)) {  // Enlarge the memory array if necessary, double the size each time
	  idcodes = (unsigned long *) realloc(idcodes, (ALLOC_SIZE << ++reallocs)*sizeof(unsigned long));
	  if(idcodes == NULL) { 
	    printf("Failed to allocate memory for device ID codes during enumeration!\n"); 
	    return APP_ERR_MALLOC;
	  }
	}
	idcodes[devindex] = tempID;
	devindex++;
      } else {
	break;
      }
    }

    if(err)  // Don't try to keep probing if we get a comm. error
      return err;
  }

  if(devindex >= MAX_DEVICES)
    printf("WARNING: maximum supported devices on JTAG chain (%i) exceeded.\n", MAX_DEVICES);

  // Put in IDLE mode
  err |= jtag_write_bit(TMS); /* EXIT1_DR */
  err |= jtag_write_bit(TMS); /* UPDATE_DR */
  err |= jtag_write_bit(0); /* IDLE */ 

  *id_array = idcodes;
  *num_devices = devindex;
  return err;
}



int jtag_get_idcode(uint32_t cmd, uint32_t *idcode)
{
  uint32_t data_out = 0;
  int err = APP_ERR_NONE;
  unsigned char saveconfig = global_altera_virtual_jtag;
  global_altera_virtual_jtag = 0; // We want the actual IDCODE, not the virtual device IDCODE

  err |= tap_set_ir(cmd);
  err |= tap_set_shift_dr();
  err |= jtag_read_write_stream(&data_out, idcode, 32, 1, 1);       /* EXIT1_DR */

  if(err)
    printf("Error getting ID code!\n");

  // Put in IDLE mode
  err |= jtag_write_bit(TMS); /* UPDATE_DR */
  err |= jtag_write_bit(0); /* IDLE */ 

  global_altera_virtual_jtag = saveconfig;
  return err;
}


/////////////////////////////////////////////////////////////////
// Helper functions

/* counts retries and returns zero if we should abort */
/* TODO: dynamically adjust timings for jp2 */
int retry_do() {
  int err = APP_ERR_NONE;

  if (soft_retry_no >= NUM_SOFT_RETRIES) {
      return 0;

      // *** TODO:  Add a 'hard retry', which re-initializes the cable, re-enumerates the bus, etc.

  } else { /* quick reset */
    if(err |= tap_reset()) {
      printf("Error %s while resetting for retry.\n", get_err_string(err)); 
      return 0;
    }

    // Put us back into DEBUG mode
    if(err |= tap_enable_debug_module()) {
      printf("Error %s enabling debug module during retry.\n", get_err_string(err)); 
      return 0;
    }

    soft_retry_no++;
    printf("Retry...\n");
  }

  return 1;
}

/* resets retry counter */
void retry_ok() {
  soft_retry_no = 0;
}

uint32_t compute_crc(uint32_t crc_in, uint32_t data_in, int length_bits)
{
  int i;
  unsigned int d, c;
  uint32_t crc_out = crc_in;
  
  for(i = 0; i < length_bits; i = i+1) 
    {
      d = ((data_in >> i) & 0x1) ? 0xffffffff : 0;
      c = (crc_out & 0x1) ? 0xffffffff : 0;
      crc_out = crc_out >> 1;
      crc_out = crc_out ^ ((d ^ c) & DBG_CRC_POLY);
    }
  return crc_out;
}

//////////////////////////////////////////////////////////////////
// Functions which operate on the debug unit

/* Selects one of the modules in the debug unit (e.g. wishbone unit, CPU0, etc.)  
 */
int dbg_select_module(int chain) 
{
  uint32_t data;
  int err = APP_ERR_NONE;

  if (current_chain == chain)
    return err;

  current_chain = -1;
  desired_chain = chain;

  // MSB of the data out must be set to 1, indicating a module select command
  data = chain | (1<<DBG_MODULE_SELECT_REG_SIZE);

  debug("select module %i\n", chain);
  err |= tap_set_shift_dr();    /* SHIFT_DR */ 
  
  /* write data, EXIT1_DR */
  err |= jtag_write_stream(&data, 3, 1);  // When TMS is set (last parameter), DR length is also adjusted; EXIT1_DR

  // *** If 'valid module selected' feedback is ever added, test it here

  err |= jtag_write_bit(TMS); /* UPDATE_DR */
  err |= jtag_write_bit(0); /* IDLE */  
  current_chain = chain;

  if(err)
    printf("Error %s selecting active debug module\n", get_err_string(err));

  return err;
}

// Set the index of the desired register in the currently selected module
// 1 bit module select command
// 4 bits opcode
// n bits index
// Make sure the corrent module/chain is selected before calling this
int dbg_select_ctrl_reg(unsigned long regidx)
{
  uint32_t data;
  int index_len = 0;
  uint32_t opcode;
  int err = APP_ERR_NONE;

  if(err |= dbg_select_module(desired_chain))
    return err;

  debug("selreg %ld\n", regidx);

  // If this reg is already selected, don't do a JTAG transaction
  if(current_reg_idx[current_chain] == regidx)
    return APP_ERR_NONE;

  switch(current_chain) {
  case DC_WISHBONE:
    index_len = DBG_WB_REG_SEL_LEN;
    opcode = DBG_WB_CMD_IREG_SEL;
    break;
  case DC_CPU0:
    index_len = DBG_CPU0_REG_SEL_LEN;
    opcode = DBG_CPU0_CMD_IREG_SEL;
    break;
  case DC_CPU1:
    index_len = DBG_CPU1_REG_SEL_LEN;
    opcode = DBG_CPU1_CMD_IREG_SEL;
    break;
  default:
    printf("ERROR! Illegal debug chain selected while selecting control register!\n");
    return 1;
  }
 

  // Set up the data.
  data = (opcode & ~(1<<DBG_WB_OPCODE_LEN)) << index_len;  // MSB must be 0 to access modules
  data |= regidx;

  debug("Selreg: data is 0x%lX (opcode = 0x%lX)\n", data,opcode);

  err |= tap_set_shift_dr();  /* SHIFT_DR */ 
  
  /* write data, EXIT1_DR */
  err |= jtag_write_stream(&data, 5+index_len, 1);

  err |= jtag_write_bit(TMS); /* UPDATE_DR */
  err |= jtag_write_bit(0); /* IDLE */  

  /* reset retry counter */
  retry_ok();
  current_reg_idx[current_chain] = regidx;

  if(err)
    printf("Error %s selecting control register %ld in module %i\n", get_err_string(err), regidx, current_chain);

  return err;
}


/* Sends out a generic command to the selected debug unit module, LSB first.  Fields are:
 * MSB: 1-bit module command
 * 4-bit opcode
 * m-bit register index
 * n-bit data (LSB)
 * Note that in the data array, the LSB of data[0] will be sent first,
 * (and become the LSB of the command)
 * up through the MSB of data[0], then the LSB of data[1], etc.
 */
int dbg_ctrl_write(unsigned long regidx, uint32_t *cmd_data, int length_bits) {
  uint32_t data;
  int index_len = 0;
  uint32_t opcode;
  int err = APP_ERR_NONE;

  if(err |= dbg_select_module(desired_chain))
    return err;

  debug("ctrl wr idx %ld dat 0x%lX\n", regidx, cmd_data[0]);

  switch(current_chain) {
  case DC_WISHBONE:
    index_len = DBG_WB_REG_SEL_LEN;
    opcode = DBG_WB_CMD_IREG_WR;
    break;
  case DC_CPU0:
    index_len = DBG_CPU0_REG_SEL_LEN;
    opcode = DBG_CPU0_CMD_IREG_WR;
    break;
  case DC_CPU1:
    index_len = DBG_CPU1_REG_SEL_LEN;
    opcode = DBG_CPU1_CMD_IREG_WR;
    break;
  default:
    printf("ERROR! Illegal debug chain selected (%i) while doing control write!\n", current_chain);
    return 1;
  }
 

  // Set up the data.  We cheat a bit here, by using 2 stream writes.
  data = (opcode & ~(1<<DBG_WB_OPCODE_LEN)) << index_len;  // MSB must be 0 to access modules
  data |= regidx;

  err |= tap_set_shift_dr();  /* SHIFT_DR */ 
  
  /* write data, EXIT1_DR */
  err |= jtag_write_stream(cmd_data, length_bits, 0);
  err |= jtag_write_stream(&data, 5+index_len, 1);

  err |= jtag_write_bit(TMS); /* UPDATE_DR */
  err |= jtag_write_bit(0); /* IDLE */  

  /* reset retry counter */
  retry_ok();
  current_reg_idx[current_chain] = regidx;

 if(err)
    printf("Error %s writing control register %ld in module %i\n", get_err_string(err), regidx, current_chain);

  return err;
}


/* reads control register (internal to the debug unit)
 * Currently only 1 register in the CPU module, so no register select
 */
int dbg_ctrl_read(unsigned long regidx, uint32_t *data, int databits) {
  uint32_t outdata[4] = {0,0,0,0};  // *** We assume no more than 128 databits
  int opcode;
  int opcode_len;
  int err = APP_ERR_NONE;

  if(err |= dbg_select_module(desired_chain))
    return err;

  if(err |= dbg_select_ctrl_reg(regidx))
    return err;

  debug("ctrl rd idx %ld\n", regidx);

  // There is no 'read' command, We write a NOP to read
  switch(current_chain) {
  case DC_WISHBONE:
    opcode = DBG_WB_CMD_NOP;
    opcode_len = DBG_WB_OPCODE_LEN;
    break;
  case DC_CPU0:
    opcode = DBG_CPU0_CMD_NOP;
    opcode_len = DBG_CPU0_OPCODE_LEN;
    break;
  case DC_CPU1:
    opcode = DBG_CPU1_CMD_NOP;    
    opcode_len = DBG_CPU1_OPCODE_LEN;
    break;
  default:
    printf("ERROR! Illegal debug chain selected while doing control read!\n");
    return 1;
  }

  outdata[0] = opcode & ~(0x1 << opcode_len);  // Zero MSB = op for module, not top-level debug unit

  err |= tap_set_shift_dr();  /* SHIFT_DR */ 
  
  // We cheat a bit here by using two stream operations.
  // First we burn the postfix bits and read the desired data, then we push a NOP
  // into position through the prefix bits.  We may be able to combine the two and save
  // some cycles, but that way leads to madness.
  err |= jtag_read_write_stream(outdata, data, databits, 1, 0);  // adjust for prefix bits
  err |= jtag_write_stream(outdata, opcode_len+1, 1);  // adjust for postfix bits, Set TMS: EXIT1_DR

  err |= jtag_write_bit(TMS); /* UPDATE_DR */
  err |= jtag_write_bit(0); /* IDLE */  


  /* reset retry counter */
  retry_ok();
  
  if(err)
    printf("Error %s reading control register %ld in module %i\n", get_err_string(err), regidx, current_chain);

  return err;
}


/* sends out a burst command to the selected module in the debug unit (MSB to LSB): 
 * 1-bit module command
 * 4-bit opcode
 * 32-bit address
 * 16-bit length (of the burst, in words)
 */
int dbg_burst_command(unsigned int opcode, unsigned long address, int length_words) {
  uint32_t data[2];
  int err = APP_ERR_NONE;

  if(err |= dbg_select_module(desired_chain))
    return err;

  debug("burst op %i adr 0x%lX len %i\n", opcode, address, length_words);

  // Set up the data
  data[0] = length_words | (address << 16);
  data[1] = ((address >> 16) | ((opcode & 0xf) << 16)) & ~(0x1<<20); // MSB must be 0 to access modules

  err |= tap_set_shift_dr();  /* SHIFT_DR */ 
  
  /* write data, EXIT1_DR */
  err |= jtag_write_stream(data, 53, 1);  // When TMS is set (last parameter), DR length is also adjusted; EXIT1_DR

  err |= jtag_write_bit(TMS); /* UPDATE_DR */
  err |= jtag_write_bit(0); /* IDLE */  

  /* reset retry counter */
  retry_ok();

  if(err)
    printf("Error %s sending burst command to module %i\n", get_err_string(err), desired_chain);

  return err;
}

// Set up and execute a burst read from a contiguous block of addresses.
// Note that there is a minor weakness in the CRC algorithm in case of retries:
// the CRC is only checked for the final burst read.  Thus, if errors/partial retries
// break up a transfer into multiple bursts, only the last burst will be CRC protected.
#define MAX_BUS_ERRORS 10
int dbg_wb_burst_read(int word_size_bytes, int word_count, unsigned long start_address, void *data)
{
  unsigned char opcode;
  uint8_t status;
  unsigned long instream;
  int i, j;
  unsigned long crc_calc;
  uint32_t crc_read;
  unsigned char word_size_bits;
  uint32_t out_data = 0;
  uint32_t in_data;
  unsigned long addr;
  uint32_t err_data[2];
  int bus_error_retries = 0;
  int err = APP_ERR_NONE;

    debug("Doing burst read, word size %d, word count %d, start address 0x%lX", word_size_bytes, word_count, start_address);

    if(word_count <= 0) {
      debug("Ignoring illegal read burst length (%d)\n", word_count);
      return 0;
    }

    instream = 0;
    word_size_bits = word_size_bytes << 3;

    // Select the appropriate opcode
    switch(current_chain) {
    case DC_WISHBONE:
      if (word_size_bytes == 1) opcode = DBG_WB_CMD_BREAD8;
      else if(word_size_bytes == 2) opcode = DBG_WB_CMD_BREAD16;
      else if(word_size_bytes == 4) opcode = DBG_WB_CMD_BREAD32;
      else {
	printf("Tried burst read with invalid word size (%0x), defaulting to 4-byte words", word_size_bytes);
	opcode = DBG_WB_CMD_BREAD32;
      }
      break;
    case DC_CPU0:
      if(word_size_bytes == 4) opcode = DBG_CPU0_CMD_BREAD32;
      else {
	printf("Tried burst read with invalid word size (%0x), defaulting to 4-byte words", word_size_bytes);
	opcode = DBG_CPU0_CMD_BREAD32;
      }
      break;
    case DC_CPU1:
      if(word_size_bytes == 4) opcode = DBG_CPU1_CMD_BREAD32;
      else {
	printf("Tried burst read with invalid word size (%0x), defaulting to 4-byte words", word_size_bytes);
	opcode = DBG_CPU0_CMD_BREAD32;
      }
      break;
    default:
      printf("ERROR! Illegal debug chain selected while doing burst read!\n");
      return 1;
    }

 wb_burst_read_retry_full:
    i = 0;
    addr = start_address;
 wb_burst_read_retry_partial:
    crc_calc = 0xffffffff;
    

    // Send the BURST READ command, returns TAP to idle state
    if(err |= dbg_burst_command(opcode, addr, (word_count-i)))  // word_count-i in case of partial retry 
      return err;

    // This is a kludge to word around oddities in the Xilinx BSCAN_* devices, and the
    // adv_dbg_if state machine.  The debug FSM needs 1 TCK between UPDATE_DR above, and
    // the CAPTURE_DR below, and the BSCAN_* won't provide it.  So, we force it, by putting the TAP
    // in BYPASS, which makes the debug_select line inactive, which is AND'ed with the TCK line (in the xilinx_internal_jtag module),
    // which forces it low.  Then we re-enable USER1/debug_select to make TCK high.  One TCK
    // event, the hard way. 
    if(global_xilinx_bscan) {
      err |= tap_set_ir(0xFFFFFFFF);
      err |= tap_enable_debug_module();
    }

    // Get us back to shift_dr mode to read a burst
    err |=  tap_set_shift_dr();
    
    // We do not adjust for the DR length here.  BYPASS regs are loaded with 0,
    // and the debug unit waits for a '1' status bit before beginning to read data.
   
   // Repeat for each word: wait until ready = 1, then read word_size_bits bits.
   for(; i < word_count; i++) 
     {
       // Get 1 status bit, then word_size_bytes*8 bits
       status = 0;
       j = 0;
       while(!status) {  // Status indicates whether there is a word available to read.  Wait until it returns true.
         err |= jtag_read_write_bit(0, &status);
         j++;
	 // If max count exceeded, retry starting with the failure address
	 if(j > MAX_READ_STATUS_WAIT) {
	   printf("Burst read timed out.\n");
	   if(!retry_do()) { 
	     printf("Retry count exceeded in burst read!\n"); 
	     return err|APP_ERR_MAX_RETRY;
	   }
	   err = APP_ERR_NONE;  // on retry, errors cleared
	   addr = start_address + (i*word_size_bytes);
	   goto wb_burst_read_retry_partial;
	 }
       }
      
       if(j > 1) {  // It's actually normal for the first read of a burst to take 2 tries, even with a fast WB clock - 3 with a Xilinx BSCAN
         debug("Took %0d tries before good status bit during burst read", j);
       }

       // Get one word of data
       err |= jtag_read_write_stream(&out_data, &in_data, word_size_bits, 0, 0);
       debug("Read 0x%0lx", in_data);

       if(err) {  // Break and retry as soon as possible on error
	 printf("Error %s during burst read.\n", get_err_string(err));
	   if(!retry_do()) { 
	     printf("Retry count exceeded in burst read!\n"); 
	     return err|APP_ERR_MAX_RETRY;
	   }
	   err = APP_ERR_NONE;  // on retry, errors cleared
	   addr = start_address + (i*word_size_bytes);
	   goto wb_burst_read_retry_partial;
       }

       crc_calc = compute_crc(crc_calc, in_data, word_size_bits);
     
       if(word_size_bytes == 1) ((unsigned char *)data)[i] = in_data & 0xFF;
       else if(word_size_bytes == 2) ((unsigned short *)data)[i] = in_data & 0xFFFF;
       else ((unsigned long *)data)[i] = in_data;
     }
    
   // All bus data was read.  Read the data CRC from the debug module.
   err |= jtag_read_write_stream(&out_data, &crc_read, 32, 0, 1);
   err |= jtag_write_bit(TMS);  // update_ir
   err |= jtag_write_bit(0);    // idle

   if(crc_calc != crc_read) {
     printf("CRC ERROR! Computed 0x%lx, read CRC 0x%lx\n", crc_calc, crc_read);
     if(!retry_do()) { 
       printf("Retry count exceeded!  Abort!\n\n");
       return err|APP_ERR_CRC;
     }
     goto  wb_burst_read_retry_full;
   }
   else debug("CRC OK!");
    


   // Now, read the error register, and retry/recompute as necessary.
   if(current_chain == DC_WISHBONE)
     {
       err |= dbg_ctrl_read(DBG_WB_REG_ERROR, err_data, 1);  // First, just get 1 bit...read address only if necessary,
       if(err_data[0] & 0x1) {  // Then we have a problem.
	 err |= dbg_ctrl_read(DBG_WB_REG_ERROR, err_data, 33);
	 addr = (err_data[0] >> 1) | (err_data[1] << 31);
	 i = (addr - start_address) / word_size_bytes;
	 printf("ERROR!  WB bus error during burst read, address 0x%lX (index 0x%X), retrying!\n", addr, i);
	 bus_error_retries++;
	 if(bus_error_retries > MAX_BUS_ERRORS) {
	   printf("Max WB bus errors reached during burst read\n");
	   return err|APP_ERR_MAX_BUS_ERR;
	 }
	 // Don't call retry_do(), a JTAG reset won't help a WB bus error
	 err_data[0] = 1;
	 err |= dbg_ctrl_write(DBG_WB_REG_ERROR, err_data, 1);  // Write 1 bit, to reset the error register,
	 goto wb_burst_read_retry_partial;
       }
     }

   retry_ok();
   return err;
}

// Set up and execute a burst write to a contiguous set of addresses
int dbg_wb_burst_write(void *data, int word_size_bytes, int word_count, unsigned long start_address)
{
  unsigned char opcode;
  uint8_t status;
  uint32_t datawords[2] = {0,0};
  uint32_t statuswords[2] = {0,0};
  int i;
  unsigned long crc_calc;
  uint32_t crc_match;
  unsigned int word_size_bits;
  unsigned long addr;
  int bus_error_retries = 0;
  uint32_t err_data[2];
  int loopct, successes;
  int first_status_loop;
  int err = APP_ERR_NONE;

    debug("Doing burst write, word size %d, word count %d, start address 0x%lx", word_size_bytes, word_count, start_address);
    word_size_bits = word_size_bytes << 3;

    if(word_count <= 0) {
      printf("Ignoring illegal burst write size (%d)\n", word_count);
      return 0;
    }

    // Select the appropriate opcode
    switch(current_chain) {
    case DC_WISHBONE:
      if (word_size_bytes == 1) opcode = DBG_WB_CMD_BWRITE8;
      else if(word_size_bytes == 2) opcode = DBG_WB_CMD_BWRITE16;
      else if(word_size_bytes == 4) opcode = DBG_WB_CMD_BWRITE32;
      else {
	printf("Tried WB burst write with invalid word size (%0x), defaulting to 4-byte words", word_size_bytes);
	opcode = DBG_WB_CMD_BWRITE32;
      }
      break;
    case DC_CPU0:
      if(word_size_bytes == 4) opcode = DBG_CPU0_CMD_BWRITE32;
      else {
	printf("Tried CPU0 burst write with invalid word size (%0x), defaulting to 4-byte words", word_size_bytes);
	opcode = DBG_CPU0_CMD_BWRITE32;
      }
      break;
    case DC_CPU1:
      if(word_size_bytes == 4) opcode = DBG_CPU1_CMD_BWRITE32;
      else {
	printf("Tried CPU1 burst write with invalid word size (%0X), defaulting to 4-byte words", word_size_bytes);
	opcode = DBG_CPU0_CMD_BWRITE32;
      }
      break;
    default:
      printf("ERROR! Illegal debug chain selected while doing burst WRITE!\n");
      return 1;
    }

    // Compute which loop iteration in which to expect the first status bit
    first_status_loop = 1 + ((global_DR_prefix_bits + global_DR_postfix_bits)/(word_size_bits+1));

 wb_burst_write_retry_full:
    i = 0;
    addr = start_address;
 wb_burst_write_retry_partial:
    crc_calc = 0xffffffff;
    successes = 0;
    

    // Send burst command, return to idle state
    if(err |= dbg_burst_command(opcode, addr, (word_count-i)))  // word_count-i in case of partial retry
      return err;
   
   // Get us back to shift_dr mode to write a burst
   err |= jtag_write_bit(TMS);  // select_dr_scan
   err |= jtag_write_bit(0);           // capture_ir
   err |= jtag_write_bit(0);           // shift_ir

   // Write a start bit (a 1) so it knows when to start counting
   err |= jtag_write_bit(TDO);

   // Now, repeat...
   for(loopct = 0; i < word_count; i++,loopct++)  // loopct only used to check status... 
     {
       // Write word_size_bytes*8 bits, then get 1 status bit
       if(word_size_bytes == 4)       datawords[0] = ((unsigned long *)data)[i];
       else if(word_size_bytes == 2) datawords[0] = ((unsigned short *)data)[i];
       else                          datawords[0] = ((unsigned char *)data)[i];
      
       crc_calc = compute_crc(crc_calc, datawords[0], word_size_bits);

       // This is an optimization
       if((global_DR_prefix_bits + global_DR_postfix_bits) == 0) {
	 err |= jtag_write_stream(datawords, word_size_bits, 0);  // Write data
	 err |= jtag_read_write_bit(0, &status);  // Read status bit
	 if(!status) {
	   addr = start_address + (i*word_size_bytes);
	   printf("Write before bus ready, retrying (idx %i, addr 0x%08lX).\n", i, addr);
	   if(!retry_do()) { printf("Retry count exceeded!  Abort!\n\n"); exit(1);}
	   // Don't bother going to TAP idle state, we're about to reset the TAP
	   goto wb_burst_write_retry_partial;
	 }
       }
       else {  // This is slower (for a USB cable anyway), because a read takes 1 more USB transaction than a write.
	 err |= jtag_read_write_stream(datawords, statuswords, word_size_bits+1, 0, 0);
	 debug("St. 0x%08lX 0x%08lX\n", statuswords[0], statuswords[1]);
	 status = (statuswords[0] || statuswords[1]);
	 if(loopct > first_status_loop) {
	   if(status) successes++;
	   else {
	     i = successes;
	     addr = start_address + (i*word_size_bytes);
	     printf("Write before bus ready, retrying (idx %i, addr 0x%08lX).\n", i, addr);
	     if(!retry_do()) { printf("Retry count exceeded!  Abort!\n\n"); exit(1);}
	     // Don't bother going to TAP idle state, we're about to reset the TAP
	     goto wb_burst_write_retry_partial;
	   }
	 }
       }

       if(err) {
	 printf("Error %s getting status bit, retrying.\n", get_err_string(err));  
	 if(!retry_do()) { 
	   printf("Retry count exceeded!\n"); 
	   return err|APP_ERR_MAX_RETRY;
	 }
	 err = APP_ERR_NONE;
	 addr = start_address + (i*word_size_bytes);
	 // Don't bother going to TAP idle state, we're about to reset the TAP
	 goto wb_burst_write_retry_partial;
	 }

      debug("Wrote 0x%0lx", datawords[0]);
     }
    
   // *** If this is a multi-device chain, at least one status bit will be lost.
   // *** If we want to check for it, we'd have to look while sending the CRC, and
   // *** maybe while burning bits to get the match bit.  So, for now, there is a
   // *** hole here.

   // Done sending data, Send the CRC we computed
   err |= jtag_write_stream(&crc_calc, 32, 0);
   for(i = 0; i < global_DR_prefix_bits; i++)  // Push the CRC data all the way to the debug unit
     err |= jtag_write_bit(0);                 // Can't do this with a stream command without setting TMS on the last bit

   // Read the 'CRC match' bit, and go to exit1_dr
   // May need to adjust for other devices in chain!
   datawords[0] = 0;
   err |= jtag_read_write_stream(datawords, &crc_match, 1, 1, 0);  // set 'adjust' to pull match bit all the way in
   // But don't set TMS above, that would shift prefix bits (again), wasting time.
   err |= jtag_write_bit(TMS);  // exit1_dr
   err |= jtag_write_bit(TMS);  // update_dr
   err |= jtag_write_bit(0);           // idle

   if(!crc_match) {
     printf("CRC ERROR! match bit after write is %ld (computed CRC 0x%lx)", crc_match, crc_calc);
     if(!retry_do()) { printf("Retry count exceeded!  Abort!\n\n"); exit(1);}
     goto  wb_burst_write_retry_full;
   }
   else debug("CRC OK!");


   // Now, read the error register and retry/recompute as needed
   if (current_chain == DC_WISHBONE)
     {
       err |= dbg_ctrl_read(DBG_WB_REG_ERROR, err_data, 1);  // First, just get 1 bit...read address only if necessary
       if(err_data[0] & 0x1) {  // Then we have a problem.
	 err |= dbg_ctrl_read(DBG_WB_REG_ERROR, err_data, 33);
	 addr = (err_data[0] >> 1) | (err_data[1] << 31);
	 i = (addr - start_address) / word_size_bytes;
	 printf("ERROR!  WB bus error during burst write, address 0x%lX (index 0x%X), retrying!\n", addr, i);
	 bus_error_retries++;
	 if(bus_error_retries > MAX_BUS_ERRORS) {
	   printf("Max WB bus errors reached!\n");
	   return err|APP_ERR_MAX_BUS_ERR;
	 }
	 // Don't call retry_do(), a JTAG reset won't help a WB bus error
	 err |= dbg_ctrl_write(DBG_WB_REG_ERROR, err_data, 1);  // Write 1 bit, to reset the error register.
	 goto wb_burst_write_retry_partial;
       }
     }

   retry_ok();
   return err;
}


//////////////////////////////////////////////////////////////////////
// API used by the GDB server

/* read a word from wishbone */
int dbg_wb_read32(unsigned long adr, unsigned long *data) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_WISHBONE)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_read(4, 1, adr, (void *)data); // All WB reads / writes are bursts
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}

/* write a word to wishbone */
int dbg_wb_write32(unsigned long adr, unsigned long data) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_WISHBONE)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_write((void *)&data, 4, 1, adr);
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}

// write a word to wishbone
// Never actually called from the GDB interface
int dbg_wb_write16(unsigned long adr, uint16_t data) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_WISHBONE)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }  
  err = dbg_wb_burst_write((void *)&data, 2, 1, adr);
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}

// write a word to wishbone
// Never actually called from the GDB interface
int dbg_wb_write8(unsigned long adr, uint8_t data) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_WISHBONE)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_write((void *)&data, 1, 1, adr);
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}


int dbg_wb_read_block32(unsigned long adr, unsigned long *data, int len) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_WISHBONE)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_read(4, len, adr, (void *)data);  // 'len' is words.
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}


// Never actually called from the GDB interface
int dbg_wb_read_block16(unsigned long adr, uint16_t *data, int len) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_WISHBONE)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_read(2, len, adr, (void *)data);  // *** is 'len' bits or words?? Call wants words...
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}

// Never actually called from the GDB interface
int dbg_wb_read_block8(unsigned long adr, uint8_t *data, int len) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_WISHBONE)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_read(1, len, adr, (void *)data);  // *** is 'len' bits or words?? Call wants words...
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}



// write a block to wishbone 
int dbg_wb_write_block32(unsigned long adr, unsigned long *data, int len) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_WISHBONE)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_write((void *)data, 4, len, adr);  // 'len' is words.
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}


// write a block to wishbone
// Never actually called from the GDB interface
int dbg_wb_write_block16(unsigned long adr, uint16_t *data, int len) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_WISHBONE)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_write((void *)data, 2, len, adr);  // *** is 'len' bits or words?? Call wants words...
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}

// write a block to wishbone
int dbg_wb_write_block8(unsigned long adr, uint8_t *data, int len) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_WISHBONE)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_write((void *)data, 1, len, adr);  // 'len' is in words...
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}


/* read a register from cpu0.  This is assumed to be an OR32 CPU, with 32-bit regs. */
int dbg_cpu0_read(unsigned long adr, unsigned long *data) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_CPU0)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_read(4, 1, adr, (void *) data); // All CPU register reads / writes are bursts
  pthread_mutex_unlock(&dbg_access_mutex);
  debug("dbg_cpu_read(), addr 0x%X, data[0] = 0x%X\n", adr, data[0]);
  return err;
}

/* read multiple registers from cpu0.  This is assumed to be an OR32 CPU, with 32-bit regs. */
int dbg_cpu0_read_block(unsigned long adr, unsigned long *data, int count) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_CPU0)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_read(4, count, adr, (void *) data); // All CPU register reads / writes are bursts
  pthread_mutex_unlock(&dbg_access_mutex);
  debug("dbg_cpu_read_block(), addr 0x%X, count %i, data[0] = 0x%X\n", adr, count, data[0]);
  return err;
}

/* write a cpu register to cpu0.  This is assumed to be an OR32 CPU, with 32-bit regs. */
int dbg_cpu0_write(unsigned long adr, unsigned long data) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_CPU0))) 
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_write((void *)&data, 4, 1, adr);
  debug("cpu0_write, adr 0x%X, data 0x%X, ret %i\n", adr, data, err);
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}

/* write multiple cpu registers to cpu0.  This is assumed to be an OR32 CPU, with 32-bit regs. */
int dbg_cpu0_write_block(unsigned long adr, unsigned long *data, int count) {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_CPU0))) 
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_write((void *)data, 4, count, adr);
  debug("cpu0_write_block, adr 0x%X, data[0] 0x%X, count %i, ret %i\n", adr, data[0], count, err);
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}

/* write a debug unit cpu module register 
 * Since OR32 debug module has only 1 register,
 * adr is ignored (for now) */
int dbg_cpu0_write_ctrl(unsigned long adr, unsigned char data) {
  int err;
  uint32_t dataword = data;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_CPU0))) {
    printf("Failed to set chain to 0x%X\n", DC_CPU0);
    pthread_mutex_unlock(&dbg_access_mutex);
    return err;
  }
  if((err = dbg_ctrl_write(DBG_CPU0_REG_STATUS, &dataword, 2))) {
    printf("Failed to write chain to 0x%X control reg 0x%X\n", DC_CPU0,DBG_CPU0_REG_STATUS );  // Only 2 bits: Reset, Stall
    pthread_mutex_unlock(&dbg_access_mutex);
    return err;
  }
  debug("cpu0_write_ctrl(): set reg to 0x%X\n", data);
  pthread_mutex_unlock(&dbg_access_mutex);
  return APP_ERR_NONE;
}


/* read a register from cpu module of the debug unit. 
 * Currently, there is only 1 register, so we do not need to select it, adr is ignored
 */
int dbg_cpu0_read_ctrl(unsigned long adr, unsigned char *data) {
  int err;
  uint32_t dataword;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_CPU0))) {
    printf("Failed to set chain to 0x%X\n", DC_CPU0);
    pthread_mutex_unlock(&dbg_access_mutex);
    return err;
  }
  if ((err = dbg_ctrl_read(DBG_CPU0_REG_STATUS, &dataword, 2))) {
    printf("Failed to read chain 0x%X control reg 0x%X\n", DC_CPU0, DBG_CPU0_REG_STATUS);
    pthread_mutex_unlock(&dbg_access_mutex);
    return err;
  }
  // reset is bit 1, stall is bit 0 in *data
  pthread_mutex_unlock(&dbg_access_mutex);
  *data = dataword;
  return APP_ERR_NONE;
}

// CPU1 Functions.  Note that 2 CPUs are not currently supported by GDB, so these are never actually
// called from the GDB interface.  They are included for completeness and future use.
// read a register from cpu1
int dbg_cpu1_read(unsigned long adr, unsigned long *data)
 {
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_CPU1)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_read(4, 1, adr, (void *) data); // All CPU register reads / writes are bursts
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}


// write a cpu register
int dbg_cpu1_write(unsigned long adr, unsigned long data) 
{
  int err;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_CPU0)))
    {
      pthread_mutex_unlock(&dbg_access_mutex);
      return err;
    }
  err = dbg_wb_burst_write((void *)&data, 4, 1, adr);
  pthread_mutex_unlock(&dbg_access_mutex);
  return err;
}


// write a debug unit cpu module register
int dbg_cpu1_write_ctrl(unsigned long adr, unsigned char data) {
   int err;
  uint32_t dataword = data;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_CPU1))) {
    printf("Failed to set chain to 0x%X\n", DC_CPU1);
    pthread_mutex_unlock(&dbg_access_mutex);
    return err;
  }
  if((err = dbg_ctrl_write(DBG_CPU1_REG_STATUS, &dataword, 2))) {
    printf("Failed to write chain to 0x%X control reg 0x%X\n", DC_CPU1,DBG_CPU0_REG_STATUS );  // Only 2 bits: Reset, Stall
    pthread_mutex_unlock(&dbg_access_mutex);
    return err;
  }
  pthread_mutex_unlock(&dbg_access_mutex);
  return APP_ERR_NONE;
}


// read a debug unit cpu module register
int dbg_cpu1_read_ctrl(unsigned long adr, unsigned char *data) {
  int err;
  uint32_t dataword;
  pthread_mutex_lock(&dbg_access_mutex);
  if ((err = dbg_select_module(DC_CPU1))) {
    printf("Failed to set chain to 0x%X\n", DC_CPU1);
    pthread_mutex_unlock(&dbg_access_mutex);
    return err;
  }
  if ((err = dbg_ctrl_read(DBG_CPU1_REG_STATUS, &dataword, 2))) {
    printf("Failed to read chain 0x%X control reg 0x%X\n", DC_CPU0, DBG_CPU1_REG_STATUS);
    pthread_mutex_unlock(&dbg_access_mutex);
    return err;
  }
  // reset is bit 1, stall is bit 0 in *data
  pthread_mutex_unlock(&dbg_access_mutex);
  *data = dataword;
  return APP_ERR_NONE;
}



