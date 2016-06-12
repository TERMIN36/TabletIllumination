
/// LCD ports configuration ///
/// Output pins definitions ///
#include "stm32f1xx_hal.h"
#include "..\MDK-ARM\lcd.h"

unsigned char wh_data, wh_sig, wh_led = 1;

/// Input pins definitions///
/// (bits locations must be the same as outputs) ///

/// Data directions ///



///////////////////////

/// LCD parameters ///
#define lcd_max_x 16//number of chars in one row
#define lcd_max_y 2 //number of rows
//////////////////////

void lcd_init(void);
void lcd_init_write(char cmd);
void lcd_write_command(char cmd);
void lcd_write_data(char data);
void lcd_clear(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_send_data(char dta);
//char lcd_read_byte(void);
void sendByte_shift(void);

void define_char( unsigned char *pc,unsigned char char_code);


unsigned char lcd_x_cnt=0;
unsigned char lcd_y_cnt=0;
static unsigned char _base_y[4]={0x80,0xc0};

void LCD_LedOn(void)
{
    wh_led = 1;
    sendByte_shift();
}

void LCD_LedOff(void)
{
    wh_led = 0;
    sendByte_shift();
}
void sendBit(uint8_t b) {
	/*
	HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, b == 1 ? GPIO_PIN_SET: GPIO_PIN_RESET);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);*/
}
void sendByte(char b) {
	sendBit((b >> 7) & 0x01);
	sendBit((b >> 6) & 0x01);
	sendBit((b >> 5) & 0x01);
	sendBit((b >> 4) & 0x01);
	sendBit((b >> 3) & 0x01);
	sendBit((b >> 2) & 0x01);
	sendBit((b >> 1) & 0x01);
	sendBit((b >> 0) & 0x01);
	//HAL_Delay(1);
//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);
	//HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}
void sendByte_shift(void)
{
    unsigned char temp1 = 0;
    // RS EN D4 D5 D6 D7 LED NA
    temp1 = ((wh_data >> 2) & 0x3C) | (wh_sig & 0x03) | (wh_led ? 64 : 0);
    sendByte(temp1);
}



void lcd_init(void)
{
wh_data = 0;
ClearEN
ClearRS

HAL_Delay(50);//wait for LCD internal initialization

_base_y[2]=lcd_max_x+0x80;
_base_y[3]=lcd_max_y+0xc0;
lcd_init_write(0x30);
lcd_init_write(0x30);
lcd_init_write(0x30);
lcd_init_write(0x20);//set 4-bit mode
lcd_write_command(0x28);//4 bit mode, 2 strings
lcd_write_command(0x04);//AC=decrement, S=no shift screen
lcd_write_command(0x85);

lcd_clear();
}

/// Clear LCD ///
void lcd_clear(void)
{
        lcd_write_command(0x02);
        lcd_write_command(0x0C);
        lcd_write_command(0x01);
}

/// Print single char ///
void lcd_putchar(char chr)
{
lcd_write_data(chr);
if(++lcd_x_cnt==lcd_max_x)
        {
        lcd_x_cnt=0;
        if(++lcd_y_cnt==lcd_max_y) lcd_y_cnt=0;
        lcd_gotoxy(0,lcd_y_cnt);
        }
}

/// Set cursor position x y ///
void lcd_gotoxy(unsigned char x, unsigned char y)
{
        lcd_write_command(_base_y[y]+x);
        lcd_x_cnt=x;
        lcd_y_cnt=y;
}

// write the string str located in SRAM to the LCD
void lcd_puts(char *str)
{
char k;
while (k = *str++) {lcd_putchar(k);};
}

// write the string str located in FLASH to the LCD
void lcd_putsf(char *str)
{
char k;
while (k = *str++) lcd_putchar(k);
}

/// Write higher nibble in 8-bit mode on init stage ///
void lcd_init_write(char cmd)
{
ClearEN
//write mode
ClearRS//command mode

//HAL_Delay(1);

wh_data = cmd;
sendByte_shift();
SetEN
//HAL_Delay(1);
ClearEN
//HAL_Delay(10);
}

/// Send to command register ///
void lcd_write_command(char cmd)
{
ClearRS//command mode
lcd_send_data(cmd);
}

/// Send to data register ///
void lcd_write_data(char data)
{
SetRS//data mode
lcd_send_data(data);
}

/// Send prepared data ///
void lcd_send_data(char dta)
{
ClearEN
//write mode

//HAL_Delay(1);

wh_data = dta;
sendByte_shift();
SetEN//latch-up
//HAL_Delay(1);
ClearEN


wh_data = dta << 4;
sendByte_shift();
SetEN
//HAL_Delay(1);
ClearEN



//HAL_Delay(10);
//while(lcd_read_BF());//wait when LCD will be ready
}
 //������� ������ ������ �� ���� ������
void LCD_SendStringFlash2(char *str)
{
  unsigned char data;
  while (*str)
  {
    data = *str++;
    lcd_write_data(data);
  }
}

//������� ����� ������ �� ���
void LCD_SendString(char *str)
{
  unsigned char data;
  while (*str)
  {
    data = *str++;
    lcd_write_data(data);
  }
}
/*
void define_char(unsigned char *pc, unsigned char char_code)
{
unsigned char i,a;
a=(char_code<<3) | 0x40;
for (i=0; i<8; i++)
        {
        //SetRS
        lcd_write_command(a++);
        lcd_write_data(*pc++);
        };
}

void LCD_WriteCharForMemory(unsigned char address, unsigned char *char_code)
{
    unsigned char i;
    lcd_write_command(64 | address*7);
    for (i=0; i<8; i++)
        {
        lcd_write_data(*char_code++);
        };
    lcd_write_command(128);
}
*/


void lcd_putnum(int num, char count_char)
{
unsigned int i = 0;
int a;
    a = num;
    if (count_char > 5)
    {
        i = 0;
        while (a > 999999) {i++; a = a - 1000000;};
        lcd_putchar(i+48);
    }
    if (count_char > 4)
    {
        i = 0;
        while (a > 9999) {i++; a = a - 10000;};
        lcd_putchar(i+48);
    }
    if (count_char > 3)
    {
        i = 0;
        while (a > 999) {i++; a = a - 1000;};
        lcd_putchar(i+48);
    }
    if (count_char > 2)
    {
        i = 0;
        while (a > 99) {i++; a = a - 100;};
        lcd_putchar(i+48);
    }

    i = 0;
    while (a > 9) {i++; a = a - 10;};
    lcd_putchar(i+48);
    lcd_putchar(a+48);
}
