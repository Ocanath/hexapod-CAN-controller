#ifndef PPP_H
#define PPP_H

#include <stdint.h>

#define FRAME_CHAR 0x7E
#define ESC_CHAR 0x7D
#define ESC_MASK 0x20

#define STUFF_BUFFER_SIZE 256
extern uint8_t gl_ppp_stuff_buffer[STUFF_BUFFER_SIZE];

int PPP_stuff(uint8_t * payload, int payload_size, uint8_t * stuffed_buffer, int stuffed_buffer_size);
int PPP_unstuff( uint8_t * payload, int payload_buffer_size, uint8_t * stuffed_buffer, int stuffed_buffer_length);
int parse_PPP_stream(uint8_t new_byte, uint8_t* payload_buffer, int payload_buffer_size, uint8_t* input_buffer, int input_buffer_size, int* bidx);


#endif
