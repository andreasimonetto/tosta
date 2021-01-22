#include "main.h"
#include "buttons.h"

static volatile uint16_t button_status_prev = 0xffff, button_status = 0xffff;
static volatile uint8_t debouncing = 0;

extern TIM_HandleTypeDef htim14;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim14) { // end buttons debounce
		debouncing = 0;
		uint16_t i, ba, bb;

		// Save status
		button_status_prev = button_status;
		button_status = 0;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) << BUTTON_SW1;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) << BUTTON_SW2;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) << BUTTON_SW3;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) << BUTTON_SW4;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(JOY_BT_GPIO_Port, JOY_BT_Pin) << BUTTON_JOYPAD;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(JOY_LX_GPIO_Port, JOY_LX_Pin) << BUTTON_JOYPAD_LEFT;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(JOY_RX_GPIO_Port, JOY_RX_Pin) << BUTTON_JOYPAD_RIGHT;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(JOY_UP_GPIO_Port, JOY_UP_Pin) << BUTTON_JOYPAD_UP;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(JOY_DN_GPIO_Port, JOY_DN_Pin) << BUTTON_JOYPAD_DOWN;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(EN1SW_GPIO_Port, EN1SW_Pin) << BUTTON_ENCODER;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(EN1A_GPIO_Port, EN1A_Pin) << _BUTTON_ENA;
		button_status |= (uint16_t)HAL_GPIO_ReadPin(EN1B_GPIO_Port, EN1B_Pin) << _BUTTON_ENB;

		// Buttons callbacks
		for(i = 0; i < BUTTONS_NUM; ++i) {
			ba = (button_status >> i) & 0x1;
			bb = (button_status_prev >> i) & 0x1;
			if(ba != bb) {
				if(ba)
					on_button_released(i);
				else
					on_button_pressed(i);
			}
		}

		// Encoder callback
		ba = (button_status >> _BUTTON_ENA) & 0x1;
		bb = (button_status >> _BUTTON_ENB) & 0x1;

		if(ba && !((button_status_prev >> _BUTTON_ENA) & 0x1))
			on_encoder_change(bb ? -1 : 1);
	}
}

static void start_debounce(void)
{
  HAL_TIM_Base_Stop_IT(&htim14);
  __HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
  __HAL_TIM_SET_COUNTER(&htim14, 0);
  HAL_TIM_Base_Start_IT(&htim14);
  debouncing = 1;
}

void EXTI0_1_IRQHandler(void)
{
  __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_0 | GPIO_PIN_1);
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(GPIO_PIN_0 | GPIO_PIN_1);
  start_debounce();
}

void EXTI2_3_IRQHandler(void)
{
  __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_2 | GPIO_PIN_3);
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(GPIO_PIN_2 | GPIO_PIN_3);
  start_debounce();
}

void EXTI4_15_IRQHandler(void)
{
  __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15);
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15);
  start_debounce();
}

uint8_t buttons_debouncing(void)
{
	return (debouncing != 0);
}

__weak void on_button_pressed(button_t b)
{
	UNUSED(b);
}

__weak void on_button_released(button_t b)
{
	UNUSED(b);
}

__weak void on_encoder_change(int8_t sign)
{
	UNUSED(sign);
}
