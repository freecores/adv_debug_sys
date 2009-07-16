
#ifndef _CABLE_FT2232_H_
#define _CABLE_FT2232_H_

#define debug(...) //fprintf(stderr, __VA_ARGS__ )
#ifndef min
#define min(X,Y) ((X) < (Y) ? (X) : (Y))
#endif

#define FTDX_MAXSEND 4096
#define FTDX_MAXSEND_MPSSE (64 * 1024)
#define FTDI_MAXRECV   ( 4 * 64)

#define BIT_CABLEID2_TCK	0 /* ADBUS0 */
#define BIT_CABLEID2_TDIDO	1 /* ADBUS1 */
#define BIT_CABLEID2_TDODI	2 /* ADBUS2 */
#define BIT_CABLEID2_TMS	3 /* ADBUS3 */
#define BITMASK_CABLEID2_TCK	(1 << BIT_CABLEID2_TCK) 
#define BITMASK_CABLEID2_TDIDO	(1 << BIT_CABLEID2_TDIDO)
#define BITMASK_CABLEID2_TDODI	(1 << BIT_CABLEID2_TDODI)
#define BITMASK_CABLEID2_TMS	(1 << BIT_CABLEID2_TMS)

#define BIT_CABLEID2_OE			1 /* ACBUS1 */
#define BIT_CABLEID2_RXLED		2 /* ACBUS2 */
#define BIT_CABLEID2_TXLED		3 /* ACBUS3 */
#define BITMASK_CABLEID2_OE	(1 << BIT_CABLEID2_OE) 
#define BITMASK_CABLEID2_RXLED	(1 << BIT_CABLEID2_RXLED)
#define BITMASK_CABLEID2_TXLED	(1 << BIT_CABLEID2_TXLED)

typedef struct usbconn_t usbconn_t;

typedef struct {
	char *name;
	char *desc;
	char *driver;
	int32_t vid;
	int32_t pid;
} usbconn_cable_t;

typedef struct {
	const char *type;
	usbconn_t *(*connect)( const char **, int, usbconn_cable_t *); 
	void (*free)( usbconn_t * );
	int (*open)( usbconn_t * );
	int (*close)( usbconn_t * );
	int (*read)( usbconn_t *, uint8_t *, int );
	int (*write)( usbconn_t *, uint8_t *, int, int );
} usbconn_driver_t;

struct  usbconn_t {
	usbconn_driver_t *driver;
	void *params;
	usbconn_cable_t *cable;
};

typedef struct {
  /* USB device information */
  unsigned int vid;
  unsigned int pid;
  struct ftdi_context *ftdic;
  char *serial;
  /* send and receive buffer handling */
  uint32_t  send_buf_len;
  uint32_t  send_buffered;
  uint8_t  *send_buf;
  uint32_t  recv_buf_len;
  uint32_t  to_recv;
  uint32_t  recv_write_idx;
  uint32_t  recv_read_idx;
  uint8_t  *recv_buf;
} ftdi_param_t;


usbconn_t * usbconn_ftdi_connect();
int my_ftdi_write_data(struct ftdi_context *ftdi, unsigned char *buf, int size);
char *my_ftdi_get_error_string (struct ftdi_context *ftdi);
int my_ftdi_read_data(struct ftdi_context *ftdi, unsigned char *buf, int size);
int my_ftdi_usb_open_desc(struct ftdi_context *ftdi, int vendor, int product, const char* description, const char* serial);
void my_ftdi_deinit(struct ftdi_context *ftdi);
int my_ftdi_usb_purge_buffers(struct ftdi_context *ftdi);
int my_ftdi_usb_purge_rx_buffer(struct ftdi_context *ftdi);
int my_ftdi_usb_purge_tx_buffer(struct ftdi_context *ftdi);
int my_ftdi_usb_reset(struct ftdi_context *ftdi);
int my_ftdi_set_latency_timer(struct ftdi_context *ftdi, unsigned char latency);
int my_ftdi_set_baudrate(struct ftdi_context *ftdi, int baudrate);
int my_ftdi_read_data_set_chunksize(struct ftdi_context *ftdi, unsigned int chunksize);
int my_ftdi_write_data_set_chunksize(struct ftdi_context *ftdi, unsigned int chunksize);
int my_ftdi_set_event_char(struct ftdi_context *ftdi, unsigned char eventch, unsigned char enable);
int my_ftdi_set_error_char(struct ftdi_context *ftdi, unsigned char errorch, unsigned char enable);
int my_ftdi_set_bitmode(struct ftdi_context *ftdi, unsigned char bitmask, unsigned char mode);
int my_ftdi_usb_close(struct ftdi_context *ftdi);
int cable_common_write_bit(uint8_t packet);
int cable_common_read_write_bit(uint8_t packet_out, uint8_t *bit_in);
int cable_common_write_stream(uint32_t *stream, int len_bits, int set_last_bit);
int cable_common_read_stream(uint32_t *outstream, uint32_t *instream, int len_bits, int set_last_bit);
int cable_ftdi_init();
int cable_ftdi_write_bit(uint8_t packet);
int cable_ftdi_read_write_bit(uint8_t packet_out, uint8_t *bit_in);
int cable_ftdi_write_stream(uint32_t *stream, int len_bits, int set_last_bit);
int cable_ftdi_read_stream(uint32_t *outstream, uint32_t *instream, int len_bits, int set_last_bit);
int cable_ftdi_opt(int c, char *str);
int cable_ftdi_flush();
int cable_ftdi_close();

#endif


