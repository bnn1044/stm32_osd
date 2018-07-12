// FILE: TNTSC.cpp
// NTSC video output library for Arduino STM32 by Tamayoshi
// Created date 2017/02/20, operation check with Blue Pill board (STM32F103C8)
// updated date 2017/02/27, addition of delay_frame ()
// updated date 2017/02/27, addition of hook registration function
// Update date 2017/03/03, resolution mode addition
// updated date 2017/04/05, clock 48 MHz supported
// Update date 2017/04/18, abolition of SPI interrupt (operation checking in progress)
// updated date 2017/04/27, NTSC scan line number correction function addition
// Fixed the selection date of 2017/04/30, SPI 1, SPI 2 update possible
// Updated date 2017/06/25, fixed external VRAM can be specified

#include"TNTSC.h"
#include<SPI.h>
#define  gpio_write ( pin, val ) gpio_write_bit (PIN_MAP [pin] .gpio_device, PIN_MAP [pin] .gpio_bit, val)
#define  PWM_CLK PA1            // Sync signal output pin (PWM)
#define  DAT PA7                // Video signal output pin
#define  NTSC_S_TOP  3          // vertical synchronization start line
#define  NTSC_S_END  5          // vertical synchronization end line
#define  NTSC_VTOP   30          // video display start line
#define  IRQ_PRIORITY   2       // timer interrupt priority
#define  MYSPI1_DMA_CH DMA_CH3  // DMA channel for SPI 1
#define  MYSPI2_DMA_CH DMA_CH5  // DMA channel for SPI 2
#define  MYSPI_DMA DMA1         // DMA for SPI
#define  Vsync_Pin       PA3       // interrupt from V sync
#define  Hsync_Pin       PA2       // interrupt from H sync
   
// Parameter setting by screen resolution
typedef  struct   {
	uint16_t width;    // number of horizontal dots on screen
	uint16_t height;   // screen vertical dot number
	uint16_t ntscH;    // NTSC screen vertical dot number
	uint16_t hsize;    // Number of horizontal bytes
	uint8_t  flgHalf;  // vertical scanning line number (0: Normal 1: half)
	uint32_t spiDiv;   // SPI clock division
} SCREEN_SETUP;

# if F_CPU == 72000000L
# define  NTSC_TIMER_DIV  3  // System clock division 1/3
const SCREEN_SETUP screen_type[] __FLASH__{
 //width height ntscH  hsize flgHalf spiDiv
	{ 112, 108, 216, 14, 1, SPI_CLOCK_DIV32 }, // 112X108
	{ 224, 108, 216, 28, 1, SPI_CLOCK_DIV16 }, // 224X108
	{ 224, 216, 216, 28, 0, SPI_CLOCK_DIV16 }, // 224X216
	{ 448, 108, 216, 56, 1, SPI_CLOCK_DIV8 },  // 448X108
	{ 448, 216, 216, 56, 0, SPI_CLOCK_DIV8 },  // 448X216
};
#elif   F_CPU == 48000000L
#define  NTSC_TIMER_DIV  2  // System clock division 1/2
const SCREEN_SETUP screen_type[] __FLASH__{
	{ 128, 96, 192, 16,  1,  SPI_CLOCK_DIV16 }, // 128x96
	{ 256, 96, 192, 32,  1,  SPI_CLOCK_DIV8 },  // 256X96
	{ 256, 192, 192, 32, 0,  SPI_CLOCK_DIV8 },  // 256X192
	{ 512, 96, 192, 64,  1,  SPI_CLOCK_DIV4 },  // 512X96
	{ 512, 192, 192, 64, 0,  SPI_CLOCK_DIV4 },  // 512X192
	{ 128, 108, 216, 16, 1,  SPI_CLOCK_DIV16 }, // 128X108
	{ 256, 108, 216, 32, 1,  SPI_CLOCK_DIV8 },  // 256X108
	{ 256, 216, 216, 32, 0,  SPI_CLOCK_DIV8 },  // 256X216
	{ 512, 108, 216, 64, 1,  SPI_CLOCK_DIV4 },  // 512X108
	{ 512, 216, 216, 64, 0,  SPI_CLOCK_DIV4 },  // 512X216
};
#endif
#define  NTSC_LINE (262+0)                       // Screen configuration Number of scanning lines (added to 2 for some monitors)
#define  SYNC(V)  gpio_write(PWM_CLK, V)         // synchronous signal output (PWM)
static  uint8_t* vram;                           // video display frame buffer
static  volatile uint8_t* ptr;                   // pointer to refer to the video display frame buffer
static  volatile int count = 1;                  // variable to count the scan line

static  void(*_bktmStartHook)() = NULL;          // blanking period start hook
static  void(*_bktmEndHook)() = NULL;            // blanking period end hook

static uint8_t  _screen;
static uint16_t _width;
static uint16_t _height;
static uint16_t _ntscHeight;
static uint16_t _vram_size;
static uint16_t _ntsc_line = NTSC_LINE;
static uint16_t _ntsc_adjust =0;
static uint8_t  _spino = 1;
static dma_channel  _spi_dma_ch = MYSPI1_DMA_CH;
static dma_dev* _spi_dma  = MYSPI_DMA;
static SPIClass* pSPI;
uint16_t TNTSC_class::width()  {return _width;;} ;
uint16_t TNTSC_class::height() {return _height;} ;
uint16_t TNTSC_class::vram_size() { return _vram_size;};
uint16_t TNTSC_class::screen() { return _screen;};
// Blanking period start hook setting
void TNTSC_class::setBktmStartHook(void (*func)()) {
  _bktmStartHook = func;
}
// Blanking period end hook setting
void TNTSC_class::setBktmEndHook(void (*func)()) {
  _bktmEndHook = func;
}
// Interrupt handler for DMA (clear data output)
void TNTSC_class::DMA1_CH3_handle() {
  while(pSPI->dev()->regs->SR & SPI_SR_BSY);
    pSPI->dev()->regs->DR = 0;
}

// Data output using DMA
void TNTSC_class::SPI_dmaSend(uint8_t *transmitBuf, uint16_t length) {
  dma_setup_transfer( 
    _spi_dma, _spi_dma_ch,  // DMA channel specification for SPI 1  
    &pSPI->dev()->regs->DR, // destination address: specify the SPI data register
	DMA_SIZE_8BITS,			// Destination data size : 1 byte
    transmitBuf,            // source address: SRAM address
    DMA_SIZE_8BITS,         // Destination data size: 1 byte
    DMA_MINC_MODE|          // flag: cyclic
    DMA_FROM_MEM |          // Peripheral from memory, transfer complete interrupted
    DMA_TRNS_CMPLT          // Transfer complete Interrupted calling 
  );
  dma_set_num_transfers(_spi_dma, _spi_dma_ch, length);  // transfer size specification
  dma_enable(_spi_dma, _spi_dma_ch);  // DMA???
}

// Data display for video (raster output)
void TNTSC_class::handle_vout() {
  delayMicroseconds(8);                                                 //delay 8us after the H sync
  if (count >= NTSC_VTOP && count <= _ntscHeight+NTSC_VTOP-1) {  	     // >=30  <= 216+50-1
    SPI_dmaSend((uint8_t *)ptr, screen_type[_screen].hsize);
  	if (screen_type[_screen].flgHalf) {
      if ((count-NTSC_VTOP) & 1) 
      ptr+= screen_type[_screen].hsize;
    } else {
      ptr+=screen_type[_screen].hsize;
    }
  }
	// Sync pulse width setting for the next scanning line
  /*if(count >= NTSC_S_TOP-1 && count <= NTSC_S_END-1){
		// Vertical sync pulse (PWM pulse width change)
	 TIMER2->regs.adv->CCR2 = 1412;
  } else {
		// Horizontal sync pulse (PWM pulse width change)
    TIMER2->regs.adv->CCR2 = 112;
  }*/
   count++; 
  Serial.println(count);
}
void TNTSC_class::vSync_reset() {
  if( count > _ntsc_line ){
    count=1;
    ptr = vram;    
  } 
  
}
void  TNTSC_class::adjust(int16_t cnt) {
	_ntsc_adjust = cnt;
	_ntsc_line = NTSC_LINE + cnt;
}
// Start NTSC video display
// void TNTSC_class :: begin (uint8_t mode) {
void  TNTSC_class::begin(uint8_t mode, uint8_t spino, uint8_t * extram) {
	// Screen setting
  pinMode(Vsync_Pin, INPUT);
  pinMode(Hsync_Pin, INPUT);
	_screen = mode <= 4 ? mode : SC_DEFAULT;
	_width = screen_type[_screen].width;
	_height = screen_type[_screen].height;
	_vram_size = screen_type[_screen].hsize * _height;
	_ntscHeight = screen_type[_screen].ntscH;
	_spino = spino;
	flgExtVram = false;
	if (extram) {
		vram = extram;
		flgExtVram = true;
	}
	else {
		vram = (uint8_t *)malloc(_vram_size);   // video display frame buffer
	}
	cls();
	ptr = vram;   // Frame buffer reference pointer for video display
	count = 1;
	// SPI initialization / setting
	if (spino == 2) {
		pSPI = new  SPIClass(2);
		_spi_dma = MYSPI_DMA;
		_spi_dma_ch = MYSPI2_DMA_CH;
	}
	else {
		pSPI = &SPI;
		_spi_dma = MYSPI_DMA;
		_spi_dma_ch = MYSPI1_DMA_CH;
	};
	pSPI->begin();
	pSPI->setBitOrder(MSBFIRST);   // The data order is the beginning
	pSPI->setDataMode(SPI_MODE3); // MODE 3 (MODE 1 is also acceptable)
	if (_spino == 2) {
		pSPI->setClockDivider(screen_type[_screen].spiDiv - 1); // Set the clock to 1/8 of the system clock 36 MHz
	}
	else {
		pSPI->setClockDivider(screen_type[_screen].spiDiv);     // Set the clock to 1/16 of the system clock 72 MHz
	}
	pSPI->dev()->regs->CR1 |= SPI_CR1_BIDIMODE_1_LINE | SPI_CR1_BIDIOE; // Setting for sending only use

	// DMA setting for SPI data transfer
	dma_init(_spi_dma);
	dma_attach_interrupt(_spi_dma, _spi_dma_ch, &DMA1_CH3_handle);
	spi_tx_dma_enable(pSPI->dev());

	// / Initial setting of timer 2
	//nvic_irq_set_priority(NVIC_TIMER2, IRQ_PRIORITY); // Set interrupt priority level
	Timer2.pause();                                   // timer stop
	Timer2.setPrescaleFactor(NTSC_TIMER_DIV);         // divide the    system clock 72 MHz to 24 MHz   0.04166666us/Count  =1/24,000,000
	//Timer2.setOverflow(1524);                       // Counter value 1524 overflow occurred 63.5 us

	/*// + 4.7 us Horizontal sync signal output setting
	pinMode(PWM_CLK, PWM);                           // Sync signal output pin (PWM)
	timer_cc_set_pol(TIMER2, 2, 1);                  // Set the output to active LOW set the timer polarity
	pwmWrite(PWM_CLK, 112);                          // Set the pulse width to 4.7 μs (provisional setting)

	// + 9.4us Register interrupt handler for video output
	Timer2.setCompare(1, 225 - 60);   // subtraction of overhead etc.
	Timer2.setMode(1, TIMER_OUTPUTCOMPARE);
	Timer2.attachInterrupt(1, handle_vout);
	Timer2.setCount(0);
	Timer2.refresh();        // timer update
	Timer2.resume();         // timer start  */
  attachInterrupt(Vsync_Pin, vSync_reset,FALLING);
  attachInterrupt(Hsync_Pin, handle_vout,FALLING);
}

// End of NTSC video display
void  TNTSC_class::end() {
	Timer2.pause();
	//Timer2.detachInterrupt(1);
   detachInterrupt(Hsync_Pin);
   detachInterrupt(Vsync_Pin);
	spi_tx_dma_disable(pSPI->dev());
	dma_detach_interrupt(_spi_dma, _spi_dma_ch);
	pSPI->end();
	if (!flgExtVram)
		free(vram);
	if (_spino == 2) {
		delete pSPI;
		// pSPI-> ~ SPIClass ();
	}
}
// Acquire the VRAM address
uint8_t * TNTSC_class::VRAM() {
	return vram;
}
// Clear screen
void  TNTSC_class::cls() {
	memset(vram, 0, _vram_size);
}
// Wait between frames
void  TNTSC_class::delay_frame(uint16_t x) {
	while (x) {
		while (count!= _ntscHeight + NTSC_VTOP);
		while (count == _ntscHeight + NTSC_VTOP);
		x--;
	}
}
TNTSC_class TNTSC;
