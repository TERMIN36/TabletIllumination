#ifndef _LCD_
#define _LCD_

//макросы для работы с битами
#define ClearBit(reg, bit)       reg &= (~(1<<(bit)))
#define SetBit(reg, bit)         reg |= (1<<(bit))

#define SetEN           SetBit(wh_sig,1);sendByte_shift();
#define ClearEN        ClearBit(wh_sig,1);sendByte_shift();
#define SetRS           SetBit(wh_sig, 0);sendByte_shift();
#define ClearRS        ClearBit(wh_sig, 0);sendByte_shift();
#define LCD_Led = bit        wh_led = bit; sendByte_shift();


#define    LCD_CHAR_HP                   &lcd_cg[0]      // знак для отрисовки процентов жизнец
#define    LCD_CHAR_P    &lcd_cg[7]                       // буква "П"
#define    LCD_CHAR_bi    &lcd_cg[14]                     // буква "Ы"
#define    LCD_CHAR_b    &lcd_cg[21]                     // буква "Б"
#define    LCD_CHAR_y    &lcd_cg[28]                      // буква "й"
#define    LCD_CHAR_5    &lcd_cg[35]
#define    LCD_CHAR_6    &lcd_cg[42]
#define    LCD_CHAR_7    &lcd_cg[49]


void lcd_init(void);
void lcd_init_write(char cmd);
void lcd_write_command(char cmd);
void lcd_write_data(char data);
void lcd_clear(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_send_data(char dta);
//char lcd_read_byte(void);
//char lcd_read_BF(void);
//void define_char(unsigned char *pc, unsigned char char_code);
void LCD_SendString(char *str);
void LCD_SendStringFlash2(char *str);
void lcd_puts(char *str);
void lcd_putchar(char chr);
void lcd_putnum(int num, char count_char);

void LCD_LedOn(void);
void LCD_LedOff(void);

//void LCD_WriteCharForMemory(unsigned char address, unsigned char *char_code);
#endif
