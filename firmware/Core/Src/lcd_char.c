#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "lcd.h"
#include "speaker.h"
#include "logo.h"
#include "font8x5.h"
#include "lcd_char.h"

enum {
	CHAR_DATA,
	CHAR_ESCAPE_START,
	CHAR_ESCAPE_PARAMS,
};

static uint8_t char_row = 0, char_col = 0;
static uint8_t char_status = CHAR_DATA, char_escapebuf_idx = 0, char_escape_params_num = 0;
static char char_escapebuf[CHAR_ESCAPEBUF_SIZE];
static int16_t char_escape_params[CHAR_PARAMS_MAX];
static uint8_t user_font[128][6];

static void char_escape_nextparam(void)
{
	char_escapebuf[char_escapebuf_idx] = 0;
	char_escapebuf_idx = 0;
	if(char_escape_params_num < CHAR_PARAMS_MAX)
		char_escape_params[char_escape_params_num++] = (uint16_t)atoi(char_escapebuf);
}

static void char_nextline()
{
	char_col = 0;
	if(char_row >= 5)
		LCD_char_scroll(1);
	else
		++char_row;
}

void LCD_char_put(uint8_t row, uint8_t col, uint8_t c)
{
uint32_t offset = row * 84 + col * 6;

	if(c < 0x20)
		return;

	if(c < 0x80) {
		memcpy((uint8_t*)lcd_framebuf + offset, FONT8x5[c - 0x20], 5);
		lcd_framebuf[offset + 5] = 0;
	}
	else
		memcpy((uint8_t*)lcd_framebuf + offset, user_font[c - 0x80], 6);
}

void LCD_char_scroll(uint8_t rows)
{
	uint32_t offset = rows * 84;
	memcpy((uint8_t*)lcd_framebuf, (uint8_t*)lcd_framebuf + offset, LCD_FRAMEBUF_SIZE - offset);
	memset((uint8_t*)lcd_framebuf + LCD_FRAMEBUF_SIZE - offset, 0, offset);
}

void LCD_char_inject(uint8_t c)
{
uint8_t i = 0;

	switch(char_status) {
		case CHAR_DATA:
			if(c == '\a') {
				speaker_play(1047, 250);
			}
			else if(c == '\b') {
				if(char_row > 0 || char_col > 0) {
					if(char_col > 0)
						--char_col;
					else {
						--char_row;
						char_col = 0;
					}
					LCD_char_put(char_row, char_col, ' ');
				}
			}
			else if(c == '\n') {
				char_nextline();
			}
			else if(c == '\f') {
				LCD_clear();
				char_col = char_row = 0;
			}
			else if(c == '\r') {
				char_col = 0;
			}
			else if(c == '\033') {
				char_status = CHAR_ESCAPE_START;
			}
			else if(c >= 0x20) {
				LCD_char_put(char_row, char_col++, c);
				if(char_col >= 14)
					char_nextline();
			}
			break;
		case CHAR_ESCAPE_START:
			switch(c) {
				case '[': // Control Sequence Introducer
					char_escapebuf_idx = 0;
					char_escape_params_num = 0;
					memset(char_escape_params, 0, sizeof(char_escape_params));
					break;
				case 's': // LCD standby
					LCD_standby_enter();
					break;
				case 'w': // LCD wakeup
					LCD_standby_exit();
					break;
				case 'l': // Show awesome logo
					memcpy((uint8_t*)lcd_framebuf, LOGO, LCD_FRAMEBUF_SIZE);
					break;
			}
			char_status = (c == '[' ? CHAR_ESCAPE_PARAMS : CHAR_DATA);
			break;
		case CHAR_ESCAPE_PARAMS:
			if(c >= 0x30 && c <= 0x3f) {
				if(char_escapebuf_idx < CHAR_ESCAPEBUF_SIZE - 1)
					char_escapebuf[char_escapebuf_idx++] = c;
			}
			else {
				char_escape_nextparam();
				if(c >= 0x40 && c <= 0x7e) {
					switch(c) {
						case 'b': // LCD brightness
							lcd_config.brightness = char_escape_params[0];
							LCD_backlight_set(lcd_config.brightness);
							break;
						case 'p': // speaker play
							speaker_play(char_escape_params[0], char_escape_params[1]);
							break;
						case 'q': // speaker play
							speaker_queue_play(char_escape_params[0], char_escape_params[1]);
							break;
						case 'c': // set user font character
							i = (uint8_t)char_escape_params[0];
							if(i >= 0x80) {
								i -= 0x80;
								user_font[i][0] = (uint8_t)char_escape_params[1];
								user_font[i][1] = (uint8_t)char_escape_params[2];
								user_font[i][2] = (uint8_t)char_escape_params[3];
								user_font[i][3] = (uint8_t)char_escape_params[4];
								user_font[i][4] = (uint8_t)char_escape_params[5];
								user_font[i][5] = (uint8_t)char_escape_params[6];
							}
							break;
						case 's': // LCD standby time
							lcd_config.standby_ticks = char_escape_params[0];
							break;
						case 'z': // cursor position
							char_col = char_escape_params[0];
							if(char_col >= LCD_CHAR_COLS)
								char_col = LCD_CHAR_COLS - 1;
							char_row = char_escape_params[1];
							if(char_row >= LCD_CHAR_ROWS)
								char_row = LCD_CHAR_ROWS - 1;
							break;
					}
					char_status = CHAR_DATA;
				}
			}
			break;
	}
}
