prefix = /usr/local

# Your build environment. selects libs, lib dirs, and include dirs
# Supported: linux, cygwin
BUILD_ENVIRONMENT=cygwin

# Set this to 'true' if you want to build for the legacy debug unit ('debug_if' core),
# leave 'false' if you are building for the Advanced Debug Unit.
SUPPORT_LEGACY=false

# Set this to 'true' to include support for cables which require libusb
# currently includes Altera USB-Blaster and Xilinx XPC DLC8
SUPPORT_USB_CABLES=true

# Set this to 'true' to support cables which require libFTDI
# SUPPORT_USB_CABLES must also be set to 'true' to support FTDI-based cables.
SUPPORT_FTDI_CABLES=true


# ----------------------------------------------------------------------------
# Most people shouldn't have to change anything below this line

ifeq ($(BUILD_ENVIRONMENT),linux)
# These are for native Linux.  You may need to put the path to libusb into the LIBS variable
# with the -L<dir> command.
CFLAGS = -g -O2 -Wall
CC = gcc
LIBS = -lpthread
INCLUDEDIRS = -I/usr/local/include/libusb-1.0/ -I/usr/local/include/

else

ifeq ($(BUILD_ENVIRONMENT),cygwin)
# These are for cygwin.  It assumes libusb.a is in the current directory.
CFLAGS = -g -O2 -Wall
CC = gcc
LIBS = -L. -lioperm -lpthread
INCLUDEDIRS = -I/usr/local/include/
endif

endif

ifeq ($(SUPPORT_LEGACY),true)
CFLAGS +=  -D__LEGACY__
endif


PROGRAMS = adv_jtag_bridge

HEADERS = adv_jtag_bridge.h chain_commands.h opencores_tap.h \
	altera_virtual_jtag.h rsp-server.h bsdl.h or32_selftest.c cable_common.h \
	cable_parallel.h cable_sim.h \
	bsdl_parse.h errcodes.h spr-defs.h except.h adv_dbg_commands.h dbg_api.h \
	legacy_dbg_commands.h

SOURCES = adv_jtag_bridge.c rsp-server.c chain_commands.c cable_common.c bsdl.c \
	or32_selftest.c cable_parallel.c cable_sim.c \
	bsdl_parse.c errcodes.c adv_dbg_commands.c dbg_api.c legacy_dbg_commands.c

OBJECTS = adv_jtag_bridge.o rsp-server.o chain_commands.o cable_common.o bsdl.o \
	or32_selftest.o cable_parallel.o cable_sim.o \
	bsdl_parse.o errcodes.o adv_dbg_commands.o dbg_api.o legacy_dbg_commands.o


ifeq ($(SUPPORT_USB_CABLES),true)
CFLAGS += -D__SUPPORT_USB_CABLES__
HEADERS += cable_usbblaster.h cable_xpc_dlc9.h
SOURCES += cable_usbblaster.c cable_xpc_dlc9.c
OBJECTS += cable_usbblaster.o cable_xpc_dlc9.o

ifeq ($(SUPPORT_FTDI_CABLES),true)
CFLAGS += -D__SUPPORT_FTDI_CABLES__
LIBS += -lftdi
HEADERS += cable_ft2232.h
SOURCES += cable_ft2232.c
OBJECTS += cable_ft2232.o
endif

# libusb must follow libftdi in the list of libraries
LIBS += -lusb
endif


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
