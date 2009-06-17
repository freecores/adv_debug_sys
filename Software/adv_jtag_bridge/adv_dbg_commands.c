/* adv_dbg_commands.c -- JTAG protocol bridge between GDB and Advanced debug module.
   Copyright(C) Nathan Yawn, nyawn@opencores.net
   
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
#include <unistd.h>  // for exit()
//#include <pthread.h>  // for mutexes

#include "chain_commands.h"
#include "adv_dbg_commands.h"     // hardware-specific defines for the debug module
//#include "altera_virtual_jtag.h"  // hardware-specifg defines for the Altera Virtual JTAG interface
#include "cable_common.h"         // low-level JTAG IO routines
#include "errcodes.h"

#define debug(...) //fprintf(stderr, __VA_ARGS__ )

// How many '0' status bits to get during a burst read
// before giving up
#define MAX_READ_STATUS_WAIT 100

// Currently selected internal register in each module
// - cuts down on unnecessary transfers
int current_reg_idx[DBG_MAX_MODULES];

// Prototypes for local functions
uint32_t adbg_compute_crc(uint32_t crc_in, uint32_t data_in, int length_bits);



////////////////////////////////////////////////////////////////////////
// Helper functions

uint32_t adbg_compute_crc(uint32_t crc_in, uint32_t data_in, int length_bits)
{
  int i;
  unsigned int d, c;
  uint32_t crc_out = crc_in;
  
  for(i = 0; i < length_bits; i = i+1) 
    {
      d = ((data_in >> i) & 0x1) ? 0xffffffff : 0;
      c = (crc_out & 0x1) ? 0xffffffff : 0;
      crc_out = crc_out >> 1;
      crc_out = crc_out ^ ((d ^ c) & ADBG_CRC_POLY);
    }
  return crc_out;
}

//////////////////////////////////////////////////////////////////
// Functions which operate on the advanced debug unit

/* Selects one of the modules in the debug unit (e.g. wishbone unit, CPU0, etc.)  
 */
int adbg_select_module(int chain) 
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

  err |= tap_exit_to_idle();  // Go from EXIT1 to IDLE

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
int adbg_select_ctrl_reg(unsigned long regidx)
{
  uint32_t data;
  int index_len = 0;
  uint32_t opcode;
  int err = APP_ERR_NONE;

  if(err |= adbg_select_module(desired_chain))
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
 
  err |= tap_exit_to_idle();  // Go from EXIT1 to IDLE

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
int adbg_ctrl_write(unsigned long regidx, uint32_t *cmd_data, int length_bits) {
  uint32_t data;
  int index_len = 0;
  uint32_t opcode;
  int err = APP_ERR_NONE;

  if(err |= adbg_select_module(desired_chain))
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

  err |= tap_exit_to_idle();  // Go from EXIT1 to IDLE

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
int adbg_ctrl_read(unsigned long regidx, uint32_t *data, int databits) {
  uint32_t outdata[4] = {0,0,0,0};  // *** We assume no more than 128 databits
  int opcode;
  int opcode_len;
  int err = APP_ERR_NONE;

  if(err |= adbg_select_module(desired_chain))
    return err;

  if(err |= adbg_select_ctrl_reg(regidx))
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

  err |= tap_exit_to_idle();  // Go from EXIT1 to IDLE

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
int adbg_burst_command(unsigned int opcode, unsigned long address, int length_words) {
  uint32_t data[2];
  int err = APP_ERR_NONE;

  if(err |= adbg_select_module(desired_chain))
    return err;

  debug("burst op %i adr 0x%lX len %i\n", opcode, address, length_words);

  // Set up the data
  data[0] = length_words | (address << 16);
  data[1] = ((address >> 16) | ((opcode & 0xf) << 16)) & ~(0x1<<20); // MSB must be 0 to access modules

  err |= tap_set_shift_dr();  /* SHIFT_DR */ 
  
  /* write data, EXIT1_DR */
  err |= jtag_write_stream(data, 53, 1);  // When TMS is set (last parameter), DR length is also adjusted; EXIT1_DR

  err |= tap_exit_to_idle();  // Go from EXIT1 to IDLE

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
int adbg_wb_burst_read(int word_size_bytes, int word_count, unsigned long start_address, void *data)
{
  unsigned char opcode;
  uint8_t status;
  unsigned long instream;
  int i, j;
  uint32_t crc_calc;
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
    if(err |= adbg_burst_command(opcode, addr, (word_count-i)))  // word_count-i in case of partial retry 
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

       crc_calc = adbg_compute_crc(crc_calc, in_data, word_size_bits);
     
       if(word_size_bytes == 1) ((unsigned char *)data)[i] = in_data & 0xFF;
       else if(word_size_bytes == 2) ((unsigned short *)data)[i] = in_data & 0xFFFF;
       else ((unsigned long *)data)[i] = in_data;
     }
    
   // All bus data was read.  Read the data CRC from the debug module.
   err |= jtag_read_write_stream(&out_data, &crc_read, 32, 0, 1);

   err |= tap_exit_to_idle();  // Go from EXIT1 to IDLE

   if(crc_calc != crc_read) {
     printf("CRC ERROR! Computed 0x%x, read CRC 0x%x\n", crc_calc, crc_read);
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
       err |= adbg_ctrl_read(DBG_WB_REG_ERROR, err_data, 1);  // First, just get 1 bit...read address only if necessary,
       if(err_data[0] & 0x1) {  // Then we have a problem.
	 err |= adbg_ctrl_read(DBG_WB_REG_ERROR, err_data, 33);
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
	 err |= adbg_ctrl_write(DBG_WB_REG_ERROR, err_data, 1);  // Write 1 bit, to reset the error register,
	 goto wb_burst_read_retry_partial;
       }
     }

   retry_ok();
   return err;
}

// Set up and execute a burst write to a contiguous set of addresses
int adbg_wb_burst_write(void *data, int word_size_bytes, int word_count, unsigned long start_address)
{
  unsigned char opcode;
  uint8_t status;
  uint32_t datawords[2] = {0,0};
  uint32_t statuswords[2] = {0,0};
  int i;
  uint32_t crc_calc;
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
    if(err |= adbg_burst_command(opcode, addr, (word_count-i)))  // word_count-i in case of partial retry
      return err;
   
   // Get us back to shift_dr mode to write a burst
   //err |= jtag_write_bit(TMS);  // select_dr_scan
   //err |= jtag_write_bit(0);           // capture_ir
   //err |= jtag_write_bit(0);           // shift_ir
   err |= tap_set_shift_dr();

   // Write a start bit (a 1) so it knows when to start counting
   err |= jtag_write_bit(TDO);

   // Now, repeat...
   for(loopct = 0; i < word_count; i++,loopct++)  // loopct only used to check status... 
     {
       // Write word_size_bytes*8 bits, then get 1 status bit
       if(word_size_bytes == 4)       datawords[0] = ((unsigned long *)data)[i];
       else if(word_size_bytes == 2) datawords[0] = ((unsigned short *)data)[i];
       else                          datawords[0] = ((unsigned char *)data)[i];
      
       crc_calc = adbg_compute_crc(crc_calc, datawords[0], word_size_bits);

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
   err |= tap_exit_to_idle();  // Go from EXIT1 to IDLE

   if(!crc_match) {
     printf("CRC ERROR! match bit after write is %i (computed CRC 0x%x)", crc_match, crc_calc);
     if(!retry_do()) { printf("Retry count exceeded!  Abort!\n\n"); exit(1);}
     goto  wb_burst_write_retry_full;
   }
   else debug("CRC OK!");


   // Now, read the error register and retry/recompute as needed
   if (current_chain == DC_WISHBONE)
     {
       err |= adbg_ctrl_read(DBG_WB_REG_ERROR, err_data, 1);  // First, just get 1 bit...read address only if necessary
       if(err_data[0] & 0x1) {  // Then we have a problem.
	 err |= adbg_ctrl_read(DBG_WB_REG_ERROR, err_data, 33);
	 addr = (err_data[0] >> 1) | (err_data[1] << 31);
	 i = (addr - start_address) / word_size_bytes;
	 printf("ERROR!  WB bus error during burst write, address 0x%lX (index 0x%X), retrying!\n", addr, i);
	 bus_error_retries++;
	 if(bus_error_retries > MAX_BUS_ERRORS) {
	   printf("Max WB bus errors reached!\n");
	   return err|APP_ERR_MAX_BUS_ERR;
	 }
	 // Don't call retry_do(), a JTAG reset won't help a WB bus error
	 err |= adbg_ctrl_write(DBG_WB_REG_ERROR, err_data, 1);  // Write 1 bit, to reset the error register.
	 goto wb_burst_write_retry_partial;
       }
     }

   retry_ok();
   return err;
}
