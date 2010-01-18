//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_defines.v                                              ////
////                                                              ////
////                                                              ////
////  This file is part of the Advanced Debug Interface.          ////
////                                                              ////
////  Author(s):                                                  ////
////       Nathan Yawn (nathan.yawn@opencores.org)                ////
////                                                              ////
////                                                              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2008 - 2010 Authors                            ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
//
// CVS Revision History
//
// $Log: adbg_defines.v,v $
// Revision 1.4  2010-01-14 02:03:40  Nathan
// Make hi-speed mode the default
//
// Revision 1.3  2010-01-10 22:53:48  Nathan
// Added define for hi-speed mode
//
// Revision 1.2  2009/05/17 20:54:56  Nathan
// Changed email address to opencores.org
//
// Revision 1.1  2008/07/22 20:28:30  Nathan
// Changed names of all files and modules (prefixed an a, for advanced).  Cleanup, indenting.  No functional changes.
//
// Revision 1.5  2008/07/06 20:02:53  Nathan
// Fixes for synthesis with Xilinx ISE (also synthesizable with 
// Quartus II 7.0).  Ran through dos2unix.
//
// Revision 1.4  2008/06/30 20:09:20  Nathan
// Removed code to select top-level module as active (it served no 
// purpose).  Re-numbered modules, requiring changes to testbench and software driver.
//

// Length of the MODULE ID register
`define	DBG_TOP_MODULE_ID_LENGTH	2

// How many modules can be supported by the module id length
`define     DBG_TOP_MAX_MODULES           4

// Chains
`define DBG_TOP_WISHBONE_DEBUG_MODULE  2'h0
`define DBG_TOP_CPU0_DEBUG_MODULE      2'h1
`define DBG_TOP_CPU1_DEBUG_MODULE      2'h2

// Length of data
`define DBG_TOP_MODULE_DATA_LEN  53


// If WISHBONE sub-module is supported uncomment the folowing line
`define DBG_WISHBONE_SUPPORTED

// If CPU_0 sub-module is supported uncomment the folowing line
`define DBG_CPU0_SUPPORTED

// If CPU_1 sub-module is supported uncomment the folowing line
//`define DBG_CPU1_SUPPORTED

// If this is defined, status bits will be skipped on burst
// writes to improve download speeds.
`define ADBG_USE_HISPEED
