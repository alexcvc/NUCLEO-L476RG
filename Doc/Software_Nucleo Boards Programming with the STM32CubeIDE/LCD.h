void LCD_STROBE(void);
void lcd_write_cmd(unsigned char);
void lcd_write_data(unsigned char);
void lcd_clear(void);
void lcd_puts(const char * s);
void lcd_putch(char c);
void lcd_goto(int,int);
void lcd_init(void);