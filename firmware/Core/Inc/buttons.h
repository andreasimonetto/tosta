#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

#include <stdint.h>

typedef enum {
	BUTTON_SW1,
	BUTTON_SW2,
	BUTTON_SW3,
	BUTTON_SW4,
	BUTTON_JOYPAD,
	BUTTON_JOYPAD_LEFT,
	BUTTON_JOYPAD_RIGHT,
	BUTTON_JOYPAD_UP,
	BUTTON_JOYPAD_DOWN,
	BUTTON_ENCODER
} button_t;

#define BUTTONS_NUM     (BUTTON_ENCODER + 1)
#define _BUTTON_ENA     (BUTTONS_NUM + 0)
#define _BUTTON_ENB     (BUTTONS_NUM + 1)
#define _BUTTONS_ENNUM  (BUTTONS_NUM + 2)

uint8_t buttons_debouncing(void);

void on_button_pressed(button_t b);
void on_button_released(button_t b);
void on_encoder_change(int8_t sign);

#endif /* INC_BUTTONS_H_ */
