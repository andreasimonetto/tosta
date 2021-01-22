#ifndef INC_SPEAKER_H_
#define INC_SPEAKER_H_

#include <stdint.h>

#define SPEAKER_CMD_QUEUE_SIZE  64

void speaker_init(void);
void speaker_play(uint16_t freq, uint16_t time_ticks);
uint8_t speaker_playing(void);
void speaker_queue_play(uint16_t freq, uint16_t time_ticks);
void speaker_queue_clear(void);
void speaker_standby_tick(uint32_t uwTickFreq);

#endif /* INC_SPEAKER_H_ */
