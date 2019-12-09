// Display Library for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: these e-papers require 3.3V supply AND data lines!
//
// based on Demo Example from Good Display: http://www.e-paper-display.com/download_list/downloadcategoryid=34&isMode=false.html
// Controller : IL3829 : http://www.e-paper-display.com/download_detail/downloadsId=534.html
//
// Author: Jean-Marc Zingg
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2

#include "GxEPD2_154_D67.h"
const uint8_t GxEPD2_154_D67::GDOControl[] = {0x01, (HEIGHT - 1) % 256, (HEIGHT - 1) / 256, 0x00}; //for 1.54inch
const uint8_t GxEPD2_154_D67::softstart[] = {0x0c, 0xd7, 0xd6, 0x9d};
const uint8_t GxEPD2_154_D67::VCOMVol[] = {0x2c, 0x9b}; // VCOM 7c
const uint8_t GxEPD2_154_D67::DummyLine[] = {0x3a, 0x1a}; // 4 dummy line per gate
const uint8_t GxEPD2_154_D67::Gatetime[] = {0x3b, 0x08}; // 2us per line

const uint8_t GxEPD2_154_D67::LUTDefault_full[] PROGMEM =
{
  0x32,  // command
  0x50, 0xAA, 0x55, 0xAA, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t GxEPD2_154_D67::LUTDefault_part[] PROGMEM =
{
  0x32,  // command
  0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x14, 0x44, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

GxEPD2_154_D67::GxEPD2_154_D67(int8_t cs, int8_t dc, int8_t rst, int8_t busy) :
  GxEPD2_EPD(cs, dc, rst, busy, HIGH, 10000000, WIDTH, HEIGHT, panel, hasColor, hasPartialUpdate, hasFastPartialUpdate)
{
}

void GxEPD2_154_D67::clearScreen(uint8_t value)
{
  _initial_write = false; // initial full screen buffer clean done
  if (_initial_refresh)
  {
    _Init_Full();
    _setPartialRamArea(0, 0, WIDTH, HEIGHT);
    // update erase buffer
    _writeCommand(0x24);
    for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
    {
      _writeData(value);
    }
    _Update_Full();
    _initial_refresh = false; // initial full update done
  }
  else
  {
    if (!_using_partial_mode) _Init_Part();
    _setPartialRamArea(0, 0, WIDTH, HEIGHT);
    _writeCommand(0x24);
    for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
    {
      _writeData(value);
    }
    _Update_Part();
  }
  if (!_using_partial_mode) _Init_Part();
  _setPartialRamArea(0, 0, WIDTH, HEIGHT);
  _writeCommand(0x24);
  for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
  {
    _writeData(value);
  }
  _Update_Part();
}

void GxEPD2_154_D67::writeScreenBuffer(uint8_t value)
{
  _initial_write = false; // initial full screen buffer clean done
  // this controller has no command to write "old data"
  if (_initial_refresh) clearScreen(value);
  else _writeScreenBuffer(value);
}

void GxEPD2_154_D67::_writeScreenBuffer(uint8_t value)
{
  if (!_using_partial_mode) _Init_Part();
  _setPartialRamArea(0, 0, WIDTH, HEIGHT);
  // update erase buffer
  _writeCommand(0x24);
  for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
  {
    _writeData(value);
  }
  _writeCommand(0x26);
  for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
  {
    _writeData(value);
  }
}

void GxEPD2_154_D67::writeImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (_initial_write) writeScreenBuffer(); // initial full screen buffer clean
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
  int16_t wb = (w + 7) / 8; // width bytes, bitmaps are padded
  x -= x % 8; // byte boundary
  w = wb * 8; // byte boundary
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x; // limit
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;
  if (!_using_partial_mode) _Init_Part();
  _setPartialRamArea(x1, y1, w1, h1);
  _writeCommand(0x24);
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint8_t data;
      // use wb, h of bitmap for index!
      int16_t idx = mirror_y ? j + dx / 8 + ((h - 1 - (i + dy))) * wb : j + dx / 8 + (i + dy) * wb;
      if (pgm)
      {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
        data = pgm_read_byte(&bitmap[idx]);
#else
        data = bitmap[idx];
#endif
      }
      else
      {
        data = bitmap[idx];
      }
      if (invert) data = ~data;
      _writeData(data);
    }
  }
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
}

void GxEPD2_154_D67::writeImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (_initial_write) writeScreenBuffer(); // initial full screen buffer clean
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
  if ((w_bitmap < 0) || (h_bitmap < 0) || (w < 0) || (h < 0)) return;
  if ((x_part < 0) || (x_part >= w_bitmap)) return;
  if ((y_part < 0) || (y_part >= h_bitmap)) return;
  int16_t wb_bitmap = (w_bitmap + 7) / 8; // width bytes, bitmaps are padded
  x_part -= x_part % 8; // byte boundary
  w = w_bitmap - x_part < w ? w_bitmap - x_part : w; // limit
  h = h_bitmap - y_part < h ? h_bitmap - y_part : h; // limit
  x -= x % 8; // byte boundary
  w = 8 * ((w + 7) / 8); // byte boundary, bitmaps are padded
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x; // limit
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;
  if (!_using_partial_mode) _Init_Part();
  _setPartialRamArea(x1, y1, w1, h1);
  _writeCommand(0x24);
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint8_t data;
      // use wb_bitmap, h_bitmap of bitmap for index!
      int16_t idx = mirror_y ? x_part / 8 + j + dx / 8 + ((h_bitmap - 1 - (y_part + i + dy))) * wb_bitmap : x_part / 8 + j + dx / 8 + (y_part + i + dy) * wb_bitmap;
      if (pgm)
      {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
        data = pgm_read_byte(&bitmap[idx]);
#else
        data = bitmap[idx];
#endif
      }
      else
      {
        data = bitmap[idx];
      }
      if (invert) data = ~data;
      _writeData(data);
    }
  }
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
}

void GxEPD2_154_D67::writeImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black)
  {
    writeImage(black, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_154_D67::writeImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black)
  {
    writeImagePart(black, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_154_D67::writeNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (data1)
  {
    writeImage(data1, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_154_D67::drawImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImage(bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
  writeImage(bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_154_D67::drawImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                               int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImagePart(bitmap, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
  writeImagePart(bitmap, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_154_D67::drawImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImage(black, color, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
  writeImage(black, color, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_154_D67::drawImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                               int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImagePart(black, color, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
  writeImagePart(black, color, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_154_D67::drawNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeNative(data1, data2, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
  writeNative(data1, data2, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_154_D67::refresh(bool partial_update_mode)
{
  if (partial_update_mode) refresh(0, 0, WIDTH, HEIGHT);
  else
  {
    if (_using_partial_mode) _Init_Full();
    _Update_Full();
    _initial_refresh = false; // initial full update done
  }
}

void GxEPD2_154_D67::refresh(int16_t x, int16_t y, int16_t w, int16_t h)
{
  if (_initial_refresh) return refresh(false); // initial update needs be full update
  x -= x % 8; // byte boundary
  w -= x % 8; // byte boundary
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x; // limit
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
  w1 -= x1 - x;
  h1 -= y1 - y;
  if (!_using_partial_mode) _Init_Part();
  _setPartialRamArea(x1, y1, w1, h1);
  _Update_Part();
}

void GxEPD2_154_D67::powerOff()
{
  _PowerOff();
}

void GxEPD2_154_D67::hibernate()
{
  _PowerOff();
  if (_rst >= 0)
  {
    _writeCommand(0x10); // deep sleep mode
    _writeData(0x1);     // enter deep sleep
    _hibernating = true;
  }
}

void GxEPD2_154_D67::_setPartialRamArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  _writeCommand(0x11); // set data entry mode
  _writeData(0x03);    // y+, x+ 
  _writeCommand(0x44); //set ram x address, start end position
  _writeData(x / 8); //A[5:0] xstart
  _writeData((x + w ) / 8 - 1); //B[5:0] xend
  _writeCommand(0x45); // set ram y address, start end position
  _writeData(y % 256); //A[7:0] //ystart
  _writeData(y / 256); //A[8] 
  _writeData((y + h - 1) % 256); // B[7:0]  //yend
  _writeData((y + h - 1) / 256); // B[8]
  _writeCommand(0x4e); // set ram x address counter
  _writeData(x / 8); //A[5:0], 
  _writeCommand(0x4f); // set ram y address counter
  _writeData(y % 256); // A[7:0]
  _writeData(y / 256); // A[8]
}
void GxEPD2_154_D67::_setRamDataEntryMode(uint8_t em)
{
  const uint16_t xPixelsPar = WIDTH - 1;
  const uint16_t yPixelsPar = HEIGHT - 1;

  em = gx_uint16_min(em, 0x03);
  // Define data entry sequence
  _writeCommand(0x11);
  _writeData(em);

  switch (em)
  {
    case 0x00: // x decrease, y decrease
      _SetRamArea(xPixelsPar / 8, 0x00, yPixelsPar % 256, yPixelsPar / 256, 0x00, 0x00);  // X-source area,Y-gate area
      _SetRamPointer(xPixelsPar / 8, yPixelsPar % 256, yPixelsPar / 256); // set ram
      break;
    case 0x01: // x increase, y decrease : as in demo code
      _SetRamArea(0x00, xPixelsPar / 8, yPixelsPar % 256, yPixelsPar / 256, 0x00, 0x00);  // X-source area,Y-gate area
      _SetRamPointer(0x00, yPixelsPar % 256, yPixelsPar / 256); // set ram
      break;
    case 0x02: // x decrease, y increase
      _SetRamArea(xPixelsPar / 8, 0x00, 0x00, 0x00, yPixelsPar % 256, yPixelsPar / 256);  // X-source area,Y-gate area
      _SetRamPointer(xPixelsPar / 8, 0x00, 0x00); // set ram
      break;
    case 0x03: // x increase, y increase : normal mode
      _SetRamArea(0x00, xPixelsPar / 8, 0x00, 0x00, yPixelsPar % 256, yPixelsPar / 256);  // X-source area,Y-gate area
      _SetRamPointer(0x00, 0x00, 0x00); // set ram
      break;
  }
}

void GxEPD2_154_D67::_SetRamArea(uint8_t Xstart, uint8_t Xend, uint8_t Ystart, uint8_t Ystart1, uint8_t Yend, uint8_t Yend1)
{
  // Specify the start and end positions of the window address in the X direction by an address unit
  _writeCommand(0x44);
  _writeData(Xstart);
  _writeData(Xend);

  // Specify the start and end positions of the window address in the Y direction by an address unit
  _writeCommand(0x45);
  _writeData(Ystart);
  _writeData(Ystart1);
  _writeData(Yend);
  _writeData(Yend1);
}
void GxEPD2_154_D67::_SetRamPointer(uint8_t addrX, uint8_t addrY, uint8_t addrY1)
{
  // Make initial setting for the RAM X address in the address counter (AC)
  _writeCommand(0x4e);
  _writeData(addrX);
  
  // Make initial setting for the RAM Y address in the address counter (AC)
  _writeCommand(0x4f);
  _writeData(addrY);
  _writeData(addrY1);
}
void GxEPD2_154_D67::_PowerOn()
{
  if (!_power_is_on)
  {
    _writeCommand(0x22);
    _writeData(0xc0);
    _writeCommand(0x20);
    _waitWhileBusy("_PowerOn", power_on_time);
  }
  _power_is_on = true;
}

void GxEPD2_154_D67::_PowerOff()
{
  _writeCommand(0x22);
  _writeData(0x03);
  _writeCommand(0x20);
  _waitWhileBusy("_PowerOff", power_off_time);
  _power_is_on = false;
  _using_partial_mode = false;
}

void GxEPD2_154_D67::_InitDisplay()
{
  if (_hibernating) _reset();
  
  _writeCommand(0x12);  //soft reset
  _waitWhileBusy("_SoftReset", power_on_time);
  _writeCommandData(GDOControl, sizeof(GDOControl));  // Panel configuration, Gate selection
//   _writeCommand(0x01); // Panel configuration, Gate selection
//   _writeData((HEIGHT - 1) % 256); //A[7:0]
//   _writeData((HEIGHT - 1) / 256); //A[8]
//   _writeData(0x00); // B[2:0], GD =0, SM = 0, TB=0
    _setRamDataEntryMode(0x03);
//   _setPartialRamArea(0, 0, WIDTH, HEIGHT);
 
  _waitWhileBusy("_RamArea", power_on_time);
  _writeCommand(0x3c); // border waveform
  _writeData(0x05);

  _writeCommand(0x18); // border waveform
  _writeData(0x80);
  _writeCommandData(softstart, sizeof(softstart));  // X decrease, Y decrease
//   _writeCommand(0x2c); // VCOM setting,Write VCOM register from MCU interface
//   _writeData(0x9b);
//   _writeCommand(0x3a); // DummyLine
//   _writeData(0x1a);    // 4 dummy line per gate
//   _writeCommand(0x3b); // Gatetime
//   _writeData(0x08);    // 2us per line
}

void GxEPD2_154_D67::_Init_Full()
{
  _InitDisplay();
  _writeCommandDataPGM(LUTDefault_full, sizeof(LUTDefault_full));

  _waitWhileBusy("_Lut_full", power_off_time);
  _PowerOn();
  _using_partial_mode = false;
}

void GxEPD2_154_D67::_Init_Part()
{
  _InitDisplay();
  _writeCommandDataPGM(LUTDefault_part, sizeof(LUTDefault_part));

  _waitWhileBusy("_Lut_part", power_off_time);
  _PowerOn();
  _using_partial_mode = true;
}

void GxEPD2_154_D67::_Update_Full()
{
  _writeCommand(0x22);
  _writeData(0xf7);
// Enable Clock Signal, then Enable CP = c0
  // c4 makes the screen go black then white again
//   _writeData(0xc4);
  _writeCommand(0x20);
  _waitWhileBusy("_Update_Full", full_refresh_time);
  _writeCommand(0xff);
}

void GxEPD2_154_D67::_Update_Part()
{
  _writeCommand(0x22);
  _writeData(0xff);
// To Display Pattern
//   _writeData(0x04);
  _writeCommand(0x20);
  _waitWhileBusy("_Update_Part", partial_refresh_time);
  _writeCommand(0xff);
}
