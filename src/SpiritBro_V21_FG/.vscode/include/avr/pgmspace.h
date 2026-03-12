/* Minimal compatibility header for avr/pgmspace.h on non-AVR platforms */
#ifndef _AVR_PGMSPACE_H_
#define _AVR_PGMSPACE_H_

#ifndef PROGMEM
#define PROGMEM
#endif

#include <string.h>

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define memcpy_P(dest, src, len) memcpy((dest), (src), (len))

#endif /* _AVR_PGMSPACE_H_ */
