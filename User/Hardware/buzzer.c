#include "buzzer.h"
#include "cmsis_os2.h"

void Buzzer_Init(){
    __HAL_TIM_ENABLE(&BUZZER_TIM);
}

void Buzzer_On(){
    HAL_TIM_PWM_Start(&BUZZER_TIM,BUZZER_CHANNEL);  //PB15
}

void Buzzer_Off(){
    HAL_TIM_PWM_Stop(&BUZZER_TIM,BUZZER_CHANNEL);
}

void Buzzer_SetFrequency(float fre){
    __HAL_TIM_DISABLE(&BUZZER_TIM);
    BUZZER_TIM.Instance->CNT = 0;
    BUZZER_TIM.Instance->ARR = (1000000/fre-1)*1u;  //修改重装载值-》频率
    BUZZER_TIM.Instance->CCR2 = (500000/fre-1)*1u;
    __HAL_TIM_ENABLE(&BUZZER_TIM);
}