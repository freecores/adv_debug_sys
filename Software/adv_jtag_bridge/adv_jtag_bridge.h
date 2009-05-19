#ifndef _ADV_JTAG_BRIDGE_H_
#define _ADV_JTAG_BRIDGE_H_

//#ifndef Boolean
//#define Boolean int
//#endid

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* Selects crc trailer size in bits. Currently supported: 8 */
//#define CRC_SIZE (8)

/* Scan chain size in bits.  */
//#define SC_SIZE (4)

//#ifndef ULONGEST
//#define ULONGEST unsigned long
//#endif


//#define DC_SIZE           4
//#define DC_STATUS_SIZE    4

//#define DC_WISHBONE       0
//#define DC_CPU0           1
//#define DC_CPU1           2

//#define DI_GO          0
//#define DI_READ_CMD    1
//#define DI_WRITE_CMD   2
//#define DI_READ_CTRL   3
//#define DI_WRITE_CTRL  4

//#define DBG_CRC_SIZE      32


//#define DBG_ERR_OK        0
//#define DBG_ERR_CRC       8


//#define CHECK(x) check(__FILE__, __LINE__, (x))
//void check(char *fn, int l, int i);


#endif /* _ADV_JTAG_BRIDGE_H_ */

