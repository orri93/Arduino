#include "fds-celsius-logo.h"

#ifndef NO_DISPLAY
const unsigned char fds_celsius_logo_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x7f, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0xf8, 0x00, 0x40, 0x01, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xf0, 0x00, 0xf0, 0xc7, 0xff, 0x00,
   0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xe0, 0x00,
   0xf8, 0xcf, 0xff, 0x03, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x38, 0xe0, 0x00, 0xf8, 0xdf, 0xff, 0x07, 0xfe, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xc0, 0x01, 0xfc, 0xc7, 0xff, 0x0f,
   0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x80, 0x01,
   0xfc, 0xc3, 0xff, 0x1f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x0e, 0x00, 0x01, 0x3e, 0xc0, 0xc7, 0x1f, 0x3f, 0x00, 0x05, 0x00,
   0x01, 0x08, 0x00, 0x00, 0x00, 0x07, 0x00, 0x03, 0x3c, 0xc0, 0x07, 0x1f,
   0x3f, 0x80, 0x1a, 0x00, 0x01, 0x08, 0x00, 0x00, 0x00, 0x83, 0x40, 0x02,
   0x3e, 0xc0, 0x07, 0x3f, 0x3f, 0x40, 0x00, 0x00, 0x41, 0x00, 0x00, 0x04,
   0x80, 0x83, 0xc0, 0x02, 0x7e, 0xc0, 0x07, 0x3e, 0x3e, 0x60, 0x00, 0x1f,
   0x51, 0x89, 0x10, 0x15, 0xc0, 0xc3, 0xc0, 0x07, 0xfc, 0xc0, 0x03, 0x1e,
   0x7e, 0x20, 0x80, 0x10, 0x19, 0x88, 0x90, 0x01, 0xc0, 0xef, 0xd8, 0x07,
   0xfe, 0xc0, 0x07, 0x3f, 0x7e, 0x60, 0x80, 0x32, 0x31, 0x88, 0x10, 0x03,
   0xe0, 0xff, 0xfe, 0x07, 0xfc, 0xc0, 0x07, 0x1f, 0x7c, 0x20, 0x80, 0x15,
   0xc1, 0x88, 0x10, 0x0c, 0xe0, 0xff, 0xff, 0x0f, 0xfe, 0xc0, 0x87, 0x1f,
   0x7c, 0x40, 0x80, 0x00, 0x81, 0x89, 0x10, 0x18, 0xf0, 0xff, 0xff, 0x0f,
   0x7c, 0xc0, 0xfb, 0x9f, 0x7e, 0xc0, 0x18, 0x11, 0x89, 0x89, 0x99, 0x08,
   0xf0, 0xff, 0xf7, 0x0f, 0x3e, 0xc0, 0xf7, 0x8f, 0x7f, 0x00, 0x0b, 0x16,
   0x71, 0x08, 0x17, 0x0f, 0xf8, 0xff, 0x8f, 0x1f, 0x3c, 0xc0, 0xf7, 0x8f,
   0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x3f, 0x1c,
   0x3e, 0xc0, 0xf7, 0x83, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0xff, 0x1e, 0x3c, 0xc0, 0xf7, 0x81, 0x1f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x07, 0x28, 0x80, 0x24, 0x00,
   0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x03,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xf8, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#endif
