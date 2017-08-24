#include <stdio.h>
#include "iso-tp.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>

int main() {
  IsoTp isotp;
  struct can_frame frame;

/* raw can send test */
  frame.can_id  = 0x123;
  frame.can_dlc = 2;
  frame.data[0] = 0x11;
  frame.data[1] = 0x22;

  //isotp.can_send ( frame.can_id, frame.can_dlc, frame.data );
  
/* raw can recv test */
  uint32_t rcv_id;
  uint8_t rcv_len;
  uint8_t rxBuffer [ 100 ];

  /*isotp.can_receive( &rcv_id, &rcv_len, rxBuffer );
  printf ( "%x %d", rcv_id, rcv_len );

  for ( int i = 0; i < rcv_len; i++ )
     printf ( " %x ", rxBuffer [ i ] );
  printf ( "\n" );*/

/* higher can protocol level send test
single frame tx test */
/*struct Message_t
{
  uint8_t len=0;
  isotp_states_t tp_state=ISOTP_IDLE;
  uint16_t seq_id=1;
  uint8_t fc_status=ISOTP_FC_CTS;
  uint8_t blocksize=0;
  uint8_t min_sep_time=0;
  uint32_t tx_id=0;
  uint32_t rx_id=0;
  uint8_t *Buffer;
};*/
  struct Message_t msg;
  uint8_t retval=0;

  /*msg.tx_id = 0x321;
  msg.rx_id = 0x123;
  msg.Buffer = rxBuffer;
  strcpy ( ( char * ) rxBuffer, "01234567890123456789012345678901234567890123" );
  msg.len = strlen ( ( char * ) rxBuffer );
  retval = isotp.send ( &msg );
  if ( retval == -1 )
    perror ( " isotp single frame test fail " );*/

  msg.tx_id = 0x321;
  msg.rx_id = 0x123;
  msg.Buffer = rxBuffer;
  msg.blocksize = 1;
  retval = isotp.receive ( &msg );
  if ( retval == -1 )
    perror ( "isotp recv frame test fail\n" );
  else {
     printf ( "received packet length: %d\n", msg.len );
     for ( int i = 0; i < msg.len; i++ ) {
        printf ( "%x ", rxBuffer [ i ] );
     }
     printf ( "\n" );
  }
}
