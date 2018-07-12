// FILE: TNTSC.h
// NTSC video output library for Arduino STM32 by Tamayoshi
// Created date 2017/02/20, operation check with Blue Pill board (STM32F103C8)
// updated date 2017/02/27, addition of delay_frame ()
// updated date 2017/02/27, addition of hook registration function
// Update date 2017/03/03, resolution mode addition
// updated date 2017/04/05, clock 48 MHz supported
// updated date 2017/04/27, NTSC scanning line number correction function adjust () added
// Fixed the selection date of 2017/04/30, SPI 1, SPI 2 update possible
// Updated date 2017/06/25, fixed external VRAM can be specified
//

#ifndef __TNTSC_H__
#define __TNTSC_H__

#include <Arduino.h>

#if F_CPU == 72000000L
#define  SC_112x108   0  // 112 x 108
#define  SC_224x108   1  // 224 x 108
#define  SC_224x216   2  // 224 x 216
#define  SC_448x108   3  // 448x108
#define  SC_448x216   4  // 448 x 216
#define  SC_DEFAULT   SC_224x216
#elif F_CPU == 48000000L
#define  SC_128x96    0  // 128 x 96
#define  SC_256x96    1  // 256 x 96
#define  SC_256x192   2  // 256 x 192
#define  SC_512x96    6  // 512 x 96
#define  SC_512x192   4  // 512 x 192
#define  SC_128x108   5  // 128 x 108
#define  SC_256x108   8  // 256 x 108
#define  SC_256x216   7  // 256 x 216
#define  SC_512x108   8  // 512 x 108
#define  SC_512x216   9  // 512 x 216
#define  SC_DEFAULT   SC_256x192
#endif

// ntsc Video display class definition
class  TNTSC_class {
private:
	uint8_t flgExtVram; // use of external secured memory (0: used 1: available)
public:
	void  begin(uint8_t mode = SC_DEFAULT, uint8_t spino = 1, uint8_t * extram = NULL);   // Start NTSC video display
	void  end();                               // End NTSC video display
	uint8_t *   VRAM();                       // Get the VRAM address
	void  cls();                              // clear screen
	void  delay_frame(uint16_t x);           // Wait for frame conversion time
	void  setBktmStartHook(void(*func) ());  // Blanking period start hook setting
	void  setBktmEndHook(void(*func) ());    // Blanking period end hook setting
	void  adjust(int16_t cnt);

	uint16_t  width();
	uint16_t  height();
	uint16_t  vram_size();
	uint16_t  screen();

private:
	static  void  handle_vout();
  static  void  vSync_reset();
 	static  void  SPI_dmaSend(uint8_t * transmitBuf, uint16_t length);
	static  void  DMA1_CH3_handle();
};

extern TNTSC_class TNTSC; // global object usage declaration

# endif
