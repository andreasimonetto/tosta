#ifndef INC_LCD_CHAR_H_
#define INC_LCD_CHAR_H_

#include <stdint.h>

#define LCD_CHAR_ROWS  (LCD_HEIGHT >> 3)
#define LCD_CHAR_COLS  (LCD_WIDTH / 6)

#define CHAR_PARAMS_MAX      8
#define CHAR_ESCAPEBUF_SIZE  8

void LCD_char_inject(uint8_t c);
void LCD_char_put(uint8_t row, uint8_t col, uint8_t c);
void LCD_char_scroll(uint8_t rows);

#endif /* INC_LCD_CHAR_H_ */
