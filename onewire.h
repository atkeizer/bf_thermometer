#include <inttypes.h>

#define OW_PIN  PE6
#define OW_IN   PINE
#define OW_OUT  PORTE
#define OW_DDR  DDRE

#define OW_CONF_CYCLESPERACCESS 13
#define OW_CONF_DELAYOFFSET ( (uint16_t)( ((OW_CONF_CYCLESPERACCESS)*1000000L) / F_CPU  ) )
#define OW_MATCH_ROM	0x55
#define OW_SKIP_ROM     0xCC
#define	OW_SEARCH_ROM	0xF0
#define	OW_SEARCH_FIRST	0xFF		// start new search
#define	OW_PRESENCE_ERR	0xFF
#define	OW_DATA_ERR     0xFE
#define OW_LAST_DEVICE	0x00		// last device found
//			0x01 ... 0x40: continue searching

// rom-code size including CRC
#define OW_ROMCODE_SIZE 8

extern uint8_t ow_reset(void);
extern uint8_t ow_bit_io( uint8_t b );
extern uint8_t ow_byte_wr( uint8_t b );
extern uint8_t ow_byte_rd( void );
//extern uint8_t ow_rom_search( uint8_t diff, uint8_t *id );
extern void ow_command( uint8_t command, uint8_t *id );
extern void ow_parasite_enable(void);
extern void ow_parasite_disable(void);
extern uint8_t ow_input_pin_state(void);


