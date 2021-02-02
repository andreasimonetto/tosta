#ifndef INC_LCD_H_
#define INC_LCD_H_

#include <stdint.h>

#define LCD_WIDTH             84
#define LCD_HEIGHT            48
#define LCD_FRAMEBUF_SIZE     ((LCD_WIDTH * LCD_HEIGHT) >> 3)
#define LCD_BIAS              0x14
#define LCD_VOP               0xB4
#define LCD_TEMPC             0x04

typedef struct {
	uint16_t standby_ticks;
	uint8_t brightness;
} lcd_config_t;

extern volatile uint8_t lcd_framebuf[LCD_FRAMEBUF_SIZE];
extern volatile lcd_config_t lcd_config;

void LCD_init(void);
void LCD_blit(void);
void LCD_clear(void);
void LCD_standby_enter(void);
void LCD_standby_exit(void);
void LCD_standby_tick(uint32_t uwTickFreq);
uint8_t LCD_active(void);
void LCD_backlight_set(uint8_t brightness);

#endif /* INC_LCD_H_ */
