#ifndef __MUSIC_H__
#define __MUSIC_H__
#include "buzzer.h"
#include "cmsis_os2.h"

typedef struct
{
    float fre;
    uint16_t time;
} Note;



#define MUSIC_SET_FREQUENCY(fre) Buzzer_SetFrequency(fre)
#define MUSIC_ON() Buzzer_On()
#define MUSIC_OFF() Buzzer_Off()
#define MUSIC_DELAY(time) osDelay(time)

// ===== C大调低音区（C3-B3）=====
#define LOW_DO 130.81f  // C3
#define LOW_RE 146.83f  // D3
#define LOW_MI 164.81f  // E3
#define LOW_FA 174.61f  // F3
#define LOW_SOL 196.00f // G3
#define LOW_LA 220.00f  // A3
#define LOW_SI 246.94f  // B3

// ===== C大调中音区（C4-B4）=====
#define MID_DO 261.63f  // C4 (中央C)
#define MID_RE 293.66f  // D4
#define MID_MI 329.63f  // E4
#define MID_FA 349.23f  // F4
#define MID_SOL 392.00f // G4
#define MID_LA 440.00f  // A4 (标准音)
#define MID_SI 493.88f  // B4

// ===== C大调高音区（C5-B5）=====
#define HIGH_DO 523.25f  // C5
#define HIGH_RE 587.33f  // D5
#define HIGH_MI 659.26f  // E5
#define HIGH_FA 698.46f  // F5
#define HIGH_SOL 783.99f // G5
#define HIGH_LA 880.00f  // A5
#define HIGH_SI 987.77f  // B5

// ===== C大调结束标志=====
#define MUSIC_FINISH 0.00f // 结束

void Music_Init();
void Music_Play(Note *note);

extern Note melody[];

#endif