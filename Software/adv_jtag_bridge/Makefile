prefix = /usr/local

# These are for native Linux.  You may need to put the path to libusb.a into the LIBS variable
# with the -L<dir> command.  You may also need to add the path to usb.h in INCLUDEDIRS -
# the default install location is already listed. 
#CFLAGS = -g -O2 -Wall
#CC = gcc
#LIBS = -lusb -lpthread 
#INCLUDEDIRS = -I/usr/local/include/libusb-1.0/ -I/usr/local/include/

# These are for cygwin.  It assumes libusb.a  and usb.h are in the current directory.
CFLAGS = -g -O2 -Wall
CC = gcc
LIBS = -L. -lioperm -lusb -lpthread
INCLUDEDIRS = -I/usr/local/include/




PROGRAMS = adv_jtag_bridge

HEADERS = adv_jtag_bridge.h adv_debug_module.h chain_commands.h opencores_tap.h \
	altera_virtual_jtag.h rsp-server.h bsdl.h or32_selftest.c cable_common.h \
	cable_parallel.h cable_sim.h cable_usbblaster.h cable_xpc_dlc9.h \
	bsdl_parse.h errcodes.h spr-defs.h except.h

SOURCES = adv_jtag_bridge.c rsp-server.c chain_commands.c cable_common.c bsdl.c \
	or32_selftest.c cable_parallel.c cable_sim.c cable_usbblaster.c cable_xpc_dlc9.c \
	bsdl_parse.c errcodes.c

OBJECTS = adv_jtag_bridge.o rsp-server.o chain_commands.o cable_common.o bsdl.o \
	or32_selftest.o cable_parallel.o cable_sim.o cable_usbblaster.o cable_xpc_dlc9.o \
	bsdl_parse.o errcodes.o

all: $(PROGRAMS)

default: $(PROGRAMS)

.c.o:
	$(CC) $(CFLAGS) -c $<

adv_jtag_bridge: Makefile $(OBJECTS) $(HEADERS)
	rm -f $@
	$(CC) -o $@ $(CFLAGS) $(OBJECTS) $(LIBS) $(INCLUDEDIRS)


install: all
	[ -d $(prefix)/bin ] || mkdir -p $(prefix)/bin
	for p in $(PROGRAMS) ; do \
	    /bin/rm -f $(prefix)/bin/$$p; \
	    /bin/cp -p $$p $(prefix)/bin/$$p; \
	done

clean: Makefile
	rm -f $(PROGRAMS) *.o *~
