#include <string.h>
#include "main.h"
#include "lcd.h"

typedef struct {
	uint16_t standby_ticks;
	uint8_t busy       : 1;
	uint8_t standby    : 1;
	uint8_t _unused    : 4;
} lcd_status_t;

static volatile uint8_t lcd_framebuf_dma[LCD_FRAMEBUF_SIZE];
static volatile lcd_status_t lcd_status = { 0, 0, 0, 0 };

volatile uint8_t lcd_framebuf[LCD_FRAMEBUF_SIZE];
volatile lcd_config_t lcd_config = { 5000, 150 };

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim3;

static void LCD_sync(void)
{
	while(lcd_status.busy);
}

static void LCD_write_command(uint8_t command)
{
	LCD_sync();
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_SPI_Transmit_DMA(&hspi1, &command, 1);
}

static void LCD_write_data(uint8_t *data, uint16_t size)
{
	LCD_sync();
	lcd_status.busy = 1;
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_SPI_Transmit_DMA(&hspi1, data, size);
}

void LCD_init(void)
{
	HAL_GPIO_WritePin(LCD_nRST_GPIO_Port, LCD_nRST_Pin, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(LCD_nRST_GPIO_Port, LCD_nRST_Pin, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();
	LCD_write_command(0x21); // LCD extended commands
	LCD_write_command(LCD_VOP); // Set LCD Vop
	LCD_write_command(LCD_TEMPC); // Set temperature coefficent
	LCD_write_command(LCD_BIAS); // LCD bias mode 1:40
	LCD_write_command(0x20); // LCD basic commands
	LCD_write_command(0x0C); // LCD normal
	LCD_clear();
	LCD_blit();
	LCD_standby_enter();
}

void LCD_blit(void)
{
	LCD_standby_exit();
	if(lcd_status.busy)
		LCD_sync();

	memcpy((void*)lcd_framebuf_dma, (void*)lcd_framebuf, LCD_FRAMEBUF_SIZE);
	LCD_write_data((void*)lcd_framebuf_dma, LCD_FRAMEBUF_SIZE);
}

void LCD_clear(void)
{
	memset((void*)lcd_framebuf, 0, LCD_FRAMEBUF_SIZE);
}

void LCD_standby_enter(void)
{
	if(lcd_status.standby)
		return;

	LCD_backlight_set(0);
	LCD_sync();
	memset((void*)lcd_framebuf_dma, 0, LCD_FRAMEBUF_SIZE);
	LCD_write_data((void*)lcd_framebuf_dma, LCD_FRAMEBUF_SIZE);
	LCD_write_command(0x21); // LCD extended commands
	LCD_write_command(0x80); // Turn off Vop pump
	LCD_write_command(0x20); // LCD basic commands
	LCD_write_command(0x08); // Display blank
	LCD_write_command(0x24); // Power down
	LCD_sync();
	lcd_status.standby = 1;
	lcd_status.standby_ticks = 0;
}

void LCD_standby_exit(void)
{
	lcd_status.standby_ticks = lcd_config.standby_ticks;
	if(!lcd_status.standby)
		return;

	lcd_status.standby = 0;
	LCD_write_command(0x21); // LCD extended commands
	LCD_write_command(LCD_VOP); // Set LCD Vop
	LCD_write_command(LCD_TEMPC); // Set temperature coefficent
	LCD_write_command(LCD_BIAS); // LCD bias mode 1:40
	LCD_write_command(0x20); // LCD basic commands
	LCD_write_command(0x0C); // LCD normal
	LCD_backlight_set(lcd_config.brightness);
	LCD_sync();
}

void LCD_standby_tick(uint32_t uwTickFreq)
{
	if(lcd_status.standby_ticks > uwTickFreq)
		lcd_status.standby_ticks -= uwTickFreq;
	else
		lcd_status.standby_ticks = 0;
}

uint8_t LCD_active(void)
{
	return (lcd_status.busy || !lcd_config.standby_ticks || lcd_status.standby_ticks);
}

void LCD_backlight_set(uint8_t brightness)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = LCD_BL_Pin;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if(brightness) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, brightness);
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
		HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	}
	else {
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
	if(hspi == &hspi1) {
		lcd_status.busy = 0;
	}
}
