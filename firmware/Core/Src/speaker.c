#include "main.h"
#include "speaker.h"

typedef struct {
	uint16_t freq;
	uint16_t ticks;
} cmd_t;

static volatile uint16_t standby_ticks = 0, cmd_queue_next_idx = 0, cmd_queue_play_idx = 0;
static volatile cmd_t cmd_queue[SPEAKER_CMD_QUEUE_SIZE];

extern TIM_HandleTypeDef htim1;

static void speaker_set(uint16_t freq)
{
uint16_t t;
GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = SPKR_Pin;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if(freq) {
		t = 65507 / freq;
		__HAL_TIM_SET_AUTORELOAD(&htim1, t);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, t >> 1);
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
		HAL_GPIO_Init(SPKR_GPIO_Port, &GPIO_InitStruct);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	}
	else {
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(SPKR_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(SPKR_GPIO_Port, SPKR_Pin, GPIO_PIN_RESET);
	}
}

void speaker_init(void)
{
	speaker_set(0);
}

void speaker_play(uint16_t freq, uint16_t time_ticks)
{
	speaker_set(freq);
	standby_ticks = time_ticks;
}

void speaker_queue_play(uint16_t freq, uint16_t time_ticks)
{
	cmd_queue[cmd_queue_next_idx].freq = freq;
	cmd_queue[cmd_queue_next_idx].ticks = time_ticks;
	if(cmd_queue_next_idx == SPEAKER_CMD_QUEUE_SIZE - 1)
		cmd_queue_next_idx = 0;
	else
		++cmd_queue_next_idx;
}

void speaker_queue_clear(void)
{
	cmd_queue_play_idx = cmd_queue_next_idx;
	speaker_set(0);
	standby_ticks = 0;
}

void speaker_standby_tick(uint32_t uwTickFreq)
{
	if(standby_ticks > uwTickFreq)
		standby_ticks -= uwTickFreq;
	else {
		if(cmd_queue_play_idx != cmd_queue_next_idx) {
			speaker_play(cmd_queue[cmd_queue_play_idx].freq, cmd_queue[cmd_queue_play_idx].ticks);
			if(cmd_queue_play_idx == SPEAKER_CMD_QUEUE_SIZE - 1)
				cmd_queue_play_idx = 0;
			else
				++cmd_queue_play_idx;
		}
		else {
			speaker_set(0);
			standby_ticks = 0;
		}
	}
}

uint8_t speaker_playing(void)
{
	return (standby_ticks > 0);
}
