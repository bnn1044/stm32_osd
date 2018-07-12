/*
Copyright (c) 2010 Myles Metzer

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/
/*
This library provides a simple method for outputting data to a tv
from a frame buffer stored in sram.  This implementation is done
completly by interupt and will return give as much cpu time to the
application as possible.
*/

/*
//
// TTVout (TVout compatible library) for Arduino Stm 32 v0.1
// Modified date 2017/01/12 by Tamayoshi, partial diversion from original TVout.h
// Correction date 2017/03/03 TNTSC v 2.2 compatible
// Correction date 2017/04/05 TNTSC v 2.3 correspondence (system clock 48 MHz correspondence)
// Modification date 2017/04/13 draw_rect, change type of argument of draw_circle
// Modified date 2017/04/26 Changed coordinate argument type of print system from uint8_t to uint16_t
// Fixed the selection date of 2017/04/30, SPI 1, SPI 2 update possible
// updated date 2017/06/25, NTSC object is modified to dynamic generation, NTSC external memory area specification supported
// Update date 2017/11/18, change the return value of hres (), hres () to int16_t
//
*/

#ifndef TTVOUT_H
#define TTVOUT_H

#include <Arduino.h>
#include "TNTSC.h"

#define WHITE         1
#define BLACK         0
#define INVERT        2

#define UP            0
#define DOWN          1
#define LEFT          2
#define RIGHT         3

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0

#define BITBAND 1  // Use bit band function

// Macros for clearer usage
#define clear_screen()      fill(0)
#define invert(color)       fill(2)

class TTVout {
  private:
    void init(uint8_t* vram, uint16_t width, uint16_t height) ;

  public:
	  TNTSC_class* TNTSC;

  TTVout() {TNTSC= &::TNTSC;} ;      // constructor
    ~TTVout() {};                    // destructor 
    void begin(uint8_t mode=SC_DEFAULT,uint8_t spino = 1,uint8_t* extram=NULL); // Start using
    void end() {TNTSC->end();};  // End usage
    void adjust(int16_t cnt) {TNTSC->adjust(cnt);} 
    uint16_t hres() {return _width;} ;  // Acquire number of horizontal dots on screen
    uint16_t vres() {return _height;} ; // Acquire vertical dot number of screen
    uint8_t* VRAM() {  return _screen;};// Obtain VRM start address
    char char_line();
    void delay(uint32_t x) {::delay(x);};  // delay (milliseconds)
    void delay_frame(uint16_t x);
    unsigned long millis() {return ::millis();} ;
    void setBktmStartHook(void (*func)()); // Blanking period start hook setting
    void setBktmEndHook(void (*func)());   // Blanking period end hook setting
    unsigned char get_pixel(int16_t x, int16_t y);
    void set_pixel(int16_t x, int16_t y, uint8_t d) ;
    void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t dt);
    void draw_row(int16_t line, int16_t x0, int16_t x1, uint8_t c);
    void draw_column(int16_t row, int16_t y0, int16_t y1, uint8_t c);
    void fill(uint8_t color);
    void shift(uint8_t distance, uint8_t direction);
    void draw_rect(int16_t x0, int16_t y0, int16_t w, int16_t h, uint8_t c, int8_t fc = -1); 
    void draw_circle(int16_t x0, int16_t y0, int16_t radius, uint8_t c, int8_t fc = -1);
    void bitmap(uint16_t x, uint16_t y, const unsigned char * bmp, uint16_t i = 0, uint16_t width = 0, uint16_t lines = 0);
	void bitmap8(uint8_t x, uint8_t y, const unsigned char * bmp, uint16_t i = 0, uint8_t width = 0, uint8_t lines = 0) 
		 { bitmap((uint8_t)x,(uint8_t)y,bmp,i,width,lines); };
    void tone(uint16_t frequency, uint16_t duration_ms=0);
    void noTone();
    void print_char(uint16_t x, uint16_t y, uint8_t c); // 
    void set_cursor(uint16_t, uint16_t);
    void select_font(const unsigned char * f);
    void write(uint8_t);
    void write(const char *str);
    void write(const uint8_t *buffer, uint8_t size);
    void print(const char[]);
    void print(char, int = BYTE);
    void print(unsigned char, int = BYTE);
    void print(int, int = DEC);
    void print(unsigned int, int = DEC);
    void print(long, int = DEC);
    void print(unsigned long, int = DEC);
    void print(double, int = 2);

    void print(uint16_t x, uint16_t y, const char str[]);
    void print(uint16_t x, uint16_t y, char c, int base) ;
    void print(uint16_t x, uint16_t y, unsigned char b, int base);
    void print(uint16_t x, uint16_t y, int n, int base) ;
    void print(uint16_t x, uint16_t y, unsigned int n, int base) ;
    void print(uint16_t x, uint16_t y, long n, int base);
    void print(uint16_t x, uint16_t y, unsigned long n, int base);
    void print(uint16_t x, uint16_t y, double n, int digits);

    void println(uint16_t, uint16_t, const char[]);
    void println(uint16_t, uint16_t, char, int = BYTE);
    void println(uint16_t, uint16_t, unsigned char, int = BYTE);
    void println(uint16_t, uint16_t, int, int = DEC);
    void println(uint16_t, uint16_t, unsigned int, int = DEC);
    void println(uint16_t, uint16_t, long, int = DEC);
    void println(uint16_t, uint16_t, unsigned long, int = DEC);
    void println(uint16_t, uint16_t, double, int = 2);
    void println(uint16_t, uint16_t);

    void println(const char[]);
    void println(char, int = BYTE);
    void println(unsigned char, int = BYTE);
    void println(int, int = DEC);
    void println(unsigned int, int = DEC);
    void println(long, int = DEC);
    void println(unsigned long, int = DEC);
    void println(double, int = 2);
    void println(void);
  
    void printPGM(const char[]);
    void printPGM(uint16_t, uint16_t, const char[]);
      
    void cls() ;

    void inc_txtline();
    void printNumber(unsigned long, uint8_t);
    void printFloat(double, uint8_t);

  private:   
    void sp(uint16_t x, uint16_t y, uint8_t c) {
    #if BITBAND==1
      if (c==1)
        _adr[_width*y+ (x&0xf8) +7 -(x&7)] = 1;
      else if (c==0)
        _adr[_width*y+ (x&0xf8) +7 -(x&7)] = 0;
      else 
        _adr[_width*y+ (x&0xf8) +7 -(x&7)] ^= 1;
    #else
      if (c==1)
        _screen[(x/8) + (y*_hres)] |= 0x80 >> (x&7);
      else if (c==0)
        _screen[(x/8) + (y*_hres)] &= ~0x80 >> (x&7);
      else
        _screen[(x/8) + (y*_hres)] ^= 0x80 >> (x&7);
    #endif
    }
  
  private:    
    uint8_t   _mode;
    uint16_t  _cursor_x;
    uint16_t  _cursor_y;
    const unsigned char * _font;
    uint8_t* _screen;        // frame buffer address
    uint16_t _width;         // Number of horizontal dots on screen
    uint16_t _height;        // screen vertical dot number
    uint16_t _hres;          // number of horizontal bytes
    uint16_t _vres;          // Number of vertical dots
    volatile uint32_t*_adr;  // frame buffer bit band address
};

#endif
