/* Tiny Clock Face

   David Johnson-Davies - www.technoblogy.com - 18th February 2016
   ATtiny85 @ 8 MHz (internal oscillator; BOD disabled)
   
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
*/
#include <avr/sleep.h>
#include <avr/power.h>

// Constants

const int SleepTime = 30000; // Time display stays on

// Pins

const int clk = 0;
const int data = 1;
const int dc = 2;
const int cs = 3;

// One Wire Protocol **********************************************

// Buffer to read data or ROM code
static union {
  uint8_t DataBytes[5];
  struct {
    uint8_t control;
    long seconds;
  } rtc;
};

const int OneWirePin = 4;

const int SkipROM = 0xCC;
const int WriteClock = 0x99;
const int ReadClock = 0x66;

inline void PinLow () {
  DDRB = DDRB | 1<<OneWirePin;
}

inline void PinRelease () {
  DDRB = DDRB & ~(1<<OneWirePin);
}

// Returns 0 or 1
inline uint8_t PinRead () {
  return PINB>>OneWirePin & 1;
}

void DelayMicros (int micro) {
  TCNT1 = 0; TIFR = 1<<OCF1A;
  OCR1A = (micro>>1) - 1;
  while ((TIFR & 1<<OCF1A) == 0);
}

void LowRelease (int low, int high) {
  PinLow();
  DelayMicros(low);
  PinRelease();
  DelayMicros(high);
}

uint8_t OneWireSetup () {
  TCCR1 = 0<<CTC1 | 0<<PWM1A | 5<<CS10;  // CTC mode, 500kHz clock
  GTCCR = 0<<PWM1B;
 }

uint8_t OneWireReset () {
  uint8_t data = 1;
  LowRelease(480, 70);
  data = PinRead();
  DelayMicros(410);
  return data;                         // 0 = device present
}

void OneWireWrite (uint8_t data) {
  int del;
  for (int i = 0; i<8; i++) {
    if ((data & 1) == 1) del = 6; else del = 60;
    LowRelease(del, 70 - del);
    data = data >> 1;
  }
}

uint8_t OneWireRead () {
  uint8_t data = 0;
  for (int i = 0; i<8; i++) {
    LowRelease(6, 9);
    data = data | PinRead()<<i;
    DelayMicros(55);
  }
  return data;
}

// Read bytes into array, least significant byte first
void OneWireReadBytes (int bytes) {
  for (int i=0; i<bytes; i++) {
    DataBytes[i] = OneWireRead();
  }
}

// Write bytes from array, least significant byte first
void OneWireWriteBytes (int bytes) {
  for (int i=0; i<bytes; i++) {
     OneWireWrite(DataBytes[i]);
  }
}

// OLED 64 x 48 monochrome display **********************************************

// Screen buffer
const int Buffersize = 64*6;
unsigned char Buffer[Buffersize];

// Initialisation sequence for OLED module
int const InitLen = 23;
const unsigned char Init[InitLen] PROGMEM = {
  0xAE, // Display off
  0xD5, // Set display clock
  0x80, // Recommended value
  0xA8, // Set multiplex
  0x3F,
  0xD3, // Set display offset
  0x00,
  0x40, // Zero start line
  0x8D, // Charge pump
  0x14,
  0x20, // Memory mode
  0x00, // Horizontal addressing
  0xA1, // 0xA0/0xA1 flip horizontally
  0xC8, // 0xC0/0xC8 flip vertically
  0xDA, // Set comp ins
  0x12,
  0x81, // Set contrast
  0x7F, // 0x00 to 0xFF
  0xD9, // Set pre charge
  0xF1,
  0xDB, // Set vcom detect
  0x40,
  0xA6  // Normal (0xA7=Inverse)
};

// Write a data byte to the display
void Data(uint8_t d) {  
  uint8_t changes = d ^ (d>>1);
  PORTB = PORTB & ~(1<<data);
  for (uint8_t bit = 0x80; bit; bit >>= 1) {
    PINB = 1<<clk; // clk low
    if (changes & bit) PINB = 1<<data;
    PINB = 1<<clk; // clk high
  }
}

// Write a command byte to the display
void Command(uint8_t c) { 
  PINB = 1<<dc; // dc low
  Data(c);
  PINB = 1<<dc; // dc high
}

void InitDisplay () {
  PINB = 1<<cs; // cs low
  for (uint8_t c=0; c<InitLen; c++) Command(pgm_read_byte(&Init[c]));
  PINB = 1<<cs; // cs high
}

void ClearBuffer () {
  for (int i = 0 ; i < Buffersize; i++) Buffer[i] = 0;
}

void DisplayOn () {
  PINB = 1<<cs; // cs low
  Command(0xAF);
  PINB = 1<<cs; // cs low
}

void DisplayOff () {
  PINB = 1<<cs; // cs low
  Command(0xAE);
  PINB = 1<<cs; // cs low
}

void DisplayBuffer() {
  PINB = 1<<cs; // cs low
  // Set column address range
  Command(0x21); Command(32); Command(95);
  // Set page address range
  Command(0x22); Command(2); Command(7); 
  for (int i = 0 ; i < Buffersize; i++) Data(Buffer[i]);
  PINB = 1<<cs; // cs high
}

// Clock **********************************************

// Constants
const int Delta = 9; // Approximation to 1 degree in radians * 2^9
const int Ss = 2;
const int Clk = 1;
const int Mosi = 0;
const int Reset = 9;

//Create global variables
unsigned long Time;
int Factor;

// Current plot position
int x0;
int y0;

// Plot point x,y into buffer if in current slice
void PlotPoint(int x, int y) {
  int row = 23 - y;
  int col = x + 32;
  int page = row>>3;
  int bit = row & 0x07;
  // Set correct bit in slice buffer
  Buffer[page*64 + col] |= 1<<bit;
}

// Move current plot position to x1,y1
void MoveTo(int x1, int y1) {
  x0 = x1;
  y0 = y1;
}

// Draw a line to x1,y1
void DrawTo(int x1, int y1) {
  int sx, sy, e2, err;
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  if (x0 < x1) sx = 1; else sx = -1;
  if (y0 < y1) sy = 1; else sy = -1;
  err = dx - dy;
  for (;;) {
    PlotPoint(x0, y0);
    if (x0==x1 && y0==y1) return;
    e2 = err<<1;
    if (e2 > -dy) {
      err = err - dy;
      x0 = x0 + sx;
    }
    if (e2 < dx) {
      err = err + dx;
      y0 = y0 + sy;
    }
  }
}
  
// Draw a hand from 0,0 to x,y
void DrawHand(int x, int y) {
   int v = x/2; int u = y/2;
   int w = v/5; int t = u/5;
   MoveTo(0, 0);
   DrawTo(v-t, u+w);
   DrawTo(x, y);
   DrawTo(v+t, u-w);
   DrawTo(0, 0);
}
       
// Draw clock
void DrawClock(int hour, int minute, int second) {
  int x = 0; int y = 23<<9;
  for (int i=0; i<360; i++) {
    int x9 = x>>9; int y9 = y>>9;
    DrawTo(x9, y9);
    // Hour marks
    if (i%30 == 0) {
      MoveTo(x9 - (x9>>3), y9 - (y9>>3));
      DrawTo(x9, y9);
    }
    // Hour hand
    if (i == hour * 30 + (minute>>1))
      DrawHand(x9 - (x9>>2), y9 - (y9>>2));
    // Minute hand
    if (i == minute * 6 + second/10) DrawHand(x9, y9);
    // Second hand
    if (i == second * 6) {
      MoveTo(0, 0);
      DrawTo(x9, y9);
    }
    // Border of clock
    MoveTo(x9, y9);
    // if (x9 > 0) DrawTo(23, y9); else DrawTo (-23, y9);
    x = x + (y9 * Delta);
    y = y - ((x>>9) * Delta);
  }
}

// Setup **********************************************

void SetTime () {
  unsigned long Offset = millis();
  unsigned long secs = 0;
  for (;;) {
    int Mins = (unsigned long)(secs / 60) % 60;
    int Hours = (unsigned long)(secs / 3600) % 12;
    // Write time to RTC
    rtc.control = 0x0C;
    rtc.seconds = secs + ((millis()-Offset)/1000);
    OneWireReset();
    OneWireWrite(SkipROM);
    OneWireWrite(WriteClock);
    OneWireWriteBytes(5);
    ClearBuffer();
    DrawClock(Hours, Mins, -1);
    DisplayBuffer();
    unsigned long Start = millis();
    while (millis()-Start < 500);
    secs = secs + 60;
  }
}

void setup() {
  // Define pins
  pinMode(dc, OUTPUT); digitalWrite(dc,HIGH);
  pinMode(clk, OUTPUT); digitalWrite(clk,HIGH);
  pinMode(data, OUTPUT);
  pinMode(cs, OUTPUT); digitalWrite(cs,HIGH);
  InitDisplay();
  DisplayOn();
  OneWireSetup();
  // Disable what we don't need to save power
  ADCSRA &= ~(1<<ADEN);         // Disable ADC
  PRR = 1<<PRUSI | 1<<PRADC;    // Turn off clocks
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // Set time on a power-on reset
  if (MCUSR & 1<<PORF) { MCUSR = 0; SetTime(); }
}

void loop() {
  unsigned long secs;
  // First read the time
  OneWireReset();
  OneWireWrite(SkipROM);
  OneWireWrite(ReadClock);
  OneWireReadBytes(5);
  OneWireReset();
  secs = rtc.seconds;
  //
  // Then display it  
  int Mins = (unsigned long)(secs / 60) % 60;
  int Hours = (unsigned long)(secs / 3600) % 12;
  int Secs = secs % 60;
  unsigned long Start = millis();
  unsigned long Now = Start;
  while (Now-Start < SleepTime) {
    ClearBuffer();
    DrawClock(Hours, Mins, (Secs + (Now-Start)/1000) % 60);
    DisplayBuffer();
    Now = millis();
  }
  DisplayOff();
  digitalWrite(dc,HIGH);
  digitalWrite(clk,HIGH);
  digitalWrite(data,HIGH);
  digitalWrite(cs,HIGH);
  pinMode(OneWirePin, OUTPUT); digitalWrite(OneWirePin,HIGH);
  sleep_enable();
  sleep_cpu();
}

