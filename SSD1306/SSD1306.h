#ifndef _SSD1306_H_
#define _SSD1306_H_

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1306_I2C I2C2
	
#include "stm32f10x_i2c.h"
#include <stdbool.h>
	
#define BLACK 0
#define WHITE 1

#define GOFi2cOLED_WIDTH 		      128	//128 Pixels
#define GOFi2cOLED_HEIGHT 		      32	//64  Pixels

#define GOFi2cOLED_Command_Mode		      0x80
#define GOFi2cOLED_Data_Mode		      0x40

//Fundamental Command (more than one bytes command pleaserefer to SSD1306 datasheet for details)
#define Set_Contrast_Cmd                      0x81     //Double byte command to select 1 out of 256 contrast steps.Default(RESET = 0x7F)
#define Entire_Display_On_Resume_Cmd          0xA4     //Resume to RAM content display(RESET), Output follows RAM content
#define Entire_Display_On_Cmd                 0xA5     //Entire display ON, Output ignores RAM content
#define GOFi2cOLED_Normal_Display_Cmd	      0xA6     //Normal display (RESET)
#define GOFi2cOLED_Inverse_Display_Cmd	      0xA7     //Inverse display
#define GOFi2cOLED_Display_Off_Cmd	      0xAE     //sleep mode(RESET)
#define GOFi2cOLED_Display_On_Cmd	      0xAF     //normal mode

//Scrolling Command (more than one bytes command pleaserefer to SSD1306 datasheet for details)
#define Right_Horizontal_Scroll_Cmd           0x26
#define Left_Horizontal_Scroll_Cmd            0x27
#define Vertical_Right_Horizontal_Scroll_Cmd  0x29
#define Vertical_Left_Horizontal_Scroll_Cmd   0x2A
#define Activate_Scroll_Cmd                   0x2F
#define Deactivate_Scroll_Cmd                 0x2E
#define Set_Vertical_Scroll_Area_Cmd          0xA3

//Addressing Setting Command (more than one bytes command pleaserefer to SSD1306 datasheet for details)
#define Set_Memory_Addressing_Mode_Cmd        0x20
#define HORIZONTAL_MODE			      0x00
#define VERTICAL_MODE			      0x01
#define PAGE_MODE			      0x02       //Default(reset)
#define Set_Column_Address_Cmd                0x21       //Setup column start and end address. This command is only for horizontal or vertical addressing mode.
#define Set_Page_Address_Cmd                  0x22       //Setup page start and end address. This command is only for horizontal or vertical addressing mode.

//Hardware Configuration (Panel resolution & layout related) Command (more than one bytes command please refer to SSD1306 datasheet for details)
#define Segment_Remap_Cmd                     0xA1       //column address 127 is mapped to SEG0
#define Segment_Normal_map_Cmd                0xA0       //Default. column address 0 is mapped to SEG0(RESET)
#define Set_Multiplex_Ratio_Cmd               0xA8       //Set MUX ratio to N+1 MUX
#define COM_Output_Normal_Scan_Cmd            0xC0       //Normal mode (RESET). Scan from COM0 to COM[N ¨C1]
#define COM_Output_Remap_Scan_Cmd             0xC8       //Remapped mode. Scan from COM[N-1] to COM0
#define Set_Display_Offset_Cmd                0xD3       //Set vertical shift by COM from 0d~63d. The value is reset to 00h after RESET.
#define Set_COM_Pins_Hardware_Config_Cmd      0xDA   

//Timing & Driving Scheme Setting Command (more than one bytes command pleaserefer to SSD1some more than one bytes command please 306 datasheet for details)
#define Set_Display_Clock_Divide_Ratio_Cmd    0xD5
#define Set_Precharge_Period_Cmd              0xD9
#define Set_VCOMH_Deselect_Level_Cmd          0xDB
#define No_Operation_Cmd                      0xE3

#define Charge_Pump_Setting_Cmd      	      0x8D
#define Charge_Pump_Enable_Cmd	              0x14
#define Charge_Pump_Disable_Cmd               0x10     //default

#define Scroll_Left			      0x00
#define Scroll_Right			      0x01

#define Scroll_2Frames			      0x7
#define Scroll_3Frames			      0x4
#define Scroll_4Frames			      0x5
#define Scroll_5Frames			      0x0
#define Scroll_25Frames			      0x6
#define Scroll_64Frames			      0x1
#define Scroll_128Frames		      0x2
#define Scroll_256Frames		      0x3

#define Dummy_Byte_0x00                       0x00
#define Dummy_Byte_0xFF                       0xFF


#define swap(a, b) { uint8_t t = a; a = b; b = t };
	
void Write_Byte(u8 Device_address, u8 Register_address, u8 Value);
void constructor(uint8_t w, uint8_t h) ;

void init(uint8_t address);

void sendCommand(unsigned char command);
void sendData(unsigned char Data);

void display();
void clearDisplay();

void drawPixel(uint8_t x, uint8_t y, uint8_t color);
void drawBitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color);

void setBrightness(unsigned char Brightness);
void setTextSize(uint8_t s);
void setCursor(uint8_t x, uint8_t y);
void setTextColor(uint8_t c, uint8_t bg);
//void setTextWrap(boollean w);



#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H */
