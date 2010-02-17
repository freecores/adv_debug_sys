README_testbench.txt
Advanced Debug Module (adv_dbg_if)
Nathan Yawn, nathan.yawn@opencores.org

Two testbenches are supplied with the advanced debug interface. The first
uses behavioral simulation of a wishbone bus with a memory attached, and
another behavioral simulation of an OR1200 CPU.  This testbench performs
and tests bus / memory operations, and performs a few CPU operations, The
top-level module is in adv_dbg_tb.v.  Other than the beavioral models, it
instantiates an adv_dbg_if (found in ../rtl/verilog/), and a JTAG TAP
("jtag" module, not included with this module).  Note that the TAP
distributed by OpenCores will not work correctly; use the version modified
by Nathan Yawn.

The second testbench includes an actuall wishbone/OR1200 system. Its
top-level entity is xsv_fpga_top.  It instantiates a wb_conbus, an OR1200,
an onchipram, a jtag TAP, and a UART16550, along with an adv_dbg_if.  The
testbench is also instantiated here, and is used to drive the inputs to
the JTAG TAP.  This testbench is less polished, but includes a functional
test of the single-step capability of the CPU.

Both testbenches were written for use in  ModelSim (version 6.3).  A 
wave.do file is also included for each testbench, which will display a
useful collection of signals in the ModelSim wave view.

