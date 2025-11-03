
/*  last update 21/02/2020 */
/*  Avi Hayun */

#ifndef _initavi_h_
#define _initavi_h_

#include "compiler_defs.h"
#include "C8051F380_defs.h"
#include <stdio.h>

void AVI(void);
void Init_Device(void);

/********************************* oscillator************************/
// F_OSC = 48    /* CLOCK frequency in MHz */
/********************************************************************/
void delay_ms(U16 ms);
void delay_us(U16 us);  //65536usec > t > 5usec

//=============================================================
//Define lcd io
sbit CS_LCD = P3 ^ 4;		//chip select when low
sbit DC_LCD = P3 ^ 3;		//high :data  low:command

sbit T_CLK	  = P3^0;//  SCK SPI   
sbit T_DIN    = P3^1;// MOSI SPI
sbit T_DO     = P3^2;// MISO	SPI	


// LCD_SCK -  P0^0;		SERIAL CLOCK INPUT    
// LCD_MOSI - P0^2;	  SERIAL DATA INPUT   
// LCD_MISO - P0^1;	  SERIAL DATA OUTPUT  

#define up2down        1
#define down2up		     2
#define right2left     3
#define left2right	   4

#define U8   unsigned char
#define U16  unsigned int
#define U32  unsigned long
#define S8   char
#define S16  int
#define S32  long	


#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE             0x001F  
#define BRED             0XF81F
#define GRED             0XFFE0
#define GBLUE            0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN            0XBC40 
#define BRRED            0XFC07 
#define GRAY             0X8430
#define DARKBLUE      	 0X01CF	
#define LIGHTBLUE      	 0X7D7C	
#define GRAYBLUE       	 0X5458 
#define LIGHTGREEN     	 0X841F 
#define LGRAY            0XC618 
#define LGRAYBLUE        0XA651 
#define LBBLUE           0X2B12 
#define BLUE2            0x051F
#define PINK             0xF81F
#define NAVY             0x000F      /*   0,   0, 128 */
#define DARKGREEN        0x03E0      /*   0, 128,   0 */
#define DARKCYAN         0x03EF      /*   0, 128, 128 */
#define MAROOM           0x7800      /* 128,   0,   0 */
#define PURPLE           0x780F      /* 128,   0, 128 */
#define OLIVE            0x7BE0      /* 128, 128,   0 */
#define LIGHTGREY        0xC618      /* 192, 192, 192 */
#define DARKGREY         0x7BEF      /* 128, 128, 128 */
#define ORANGE           0xFD20      /* 255, 165,   0 */
#define GREENYELLOW      0xAFE5      /* 173, 255,  47 */

typedef struct
{
	int width;
	int height;
	U16 fontColor;
	U16	fontBackground;
	U8 fontWithbackgrount;
	U8 fontSize;
	U8  rotation;
	int x;
	int y;

}lcd_dev;

extern lcd_dev LCD;

void initSysSpi(void);
void setPrint(U8 target);// 0-LCD , 1- uart

/**********************************************************/
void LCD_begin(void);
void LCD_fillScreen(U16 Color);
void LCD_drawPixel(int x, int y, U16 Color);
void LCD_setRotation(U8 Direction);
void LCD_setCursor(int x, int y);
void LCD_setText1Color(U16 Color);
void LCD_setText2Color(U16 fColor, U16 bColor);
void LCD_setTextSize(U8 fontSize);
/**********************************************************/
void LCD_print(char *s);
void LCD_println(char *s);
void LCD_printC(int x, int y, char *s, U8 fontSize, U16 Color);
void LCD_print2C(int x, int y, char *s, U8 fontSize, U16 fColor, U16 bColor);
void LCD_printCenter(int x, int y, char *str, U8 fontSize, U16 fColor, U16 bColor);

void LCD_printNumF(float num);
void LCD_printNumI(int num);
void LCD_printNumC(char num);
void LCD_printNumH(U16 num);
void LCD_printNumB(U8 num);

void Int2Bin(U16 num,char str[]);
void Byte2Bin(U8 num,char str[]);

/******************* HEB ***********************************/
void LCD_printHeb(char *str);
void LCD_printHebln(char *str);
void LCD_printHebC(int x, int y, char *str, U8 fontSize, U16 fColor);
void LCD_printHeb2C(int x, int y, char *str, U8 fontSize, U16 fColor, U16 bColor);
void LCD_printHebCenter(int x, int y, char *str, U8 fontSize, U16 fColor, U16 bColor);

/*************************************************************/
void LCD_DrawBitmap16(int x, int y,  U8 *pic);

void LCD_drawFastVLine(int x, int y, int h, U16 Color);
void LCD_drawFastHLine(int x, int y, int w, U16 Color);
void LCD_drawLine(int x1, int y1, int x2, int y2, U16 Color);
void LCD_drawRect(int x, int y, int w, int h, U16 Color);
void LCD_fillRect(int x, int y, int w, int h, U16 Color);
void LCD_drawCircle(int CenterCircleX, int CenterCircleY, int Radius, U16 Color);
void LCD_fillCircle(int CenterCircleX, int CenterCircleY, int Radius, U16 Color);
void LCD_drawTriangle(int x1, int y1, int x2, int y2, int x3, int y3, U16 Color);
void LCD_fillTriangle(int x1, int y1, int x2, int y2, int x3, int y3, U16 Color);
void LCD_fillCircleHelper(int x0, int y0, int r, U8 cornername, int delta, U16 Color);
void LCD_fillRoundRect(int x, int y, int w, int h, int r, U16 Color);

/*************************************************/
sbit T_CS     =    P3^7;
sbit T_IRQ    =    P3^6;  

// T_CLK	  - P0^0   SCK SPI   
// T_DIN    - P0^2   MOSI SPI
// T_DO     - P0^1   MISO	SPI																				   

int ReadTouchX(void);
int ReadTouchY(void);

void TouchCal();
void TouchSet(int xs1, int xs2, int ys1, int ys2);
void LCD_clearButton(void);

void LCD_drawButton(U8 NumButton,int x, int y, int w,int h, int r, U16 Color,U16 textcolor,char *label,U8 textsize);
int ButtonTouch(int x,int y);

#endif