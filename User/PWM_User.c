#include "PWM_User.h"

void Duty_4(uint16_t Chna1, uint16_t Chna2, uint16_t Chna3, uint16_t Chna4)
{
  TIM3->CCR1 = Chna1;
  TIM3->CCR2 = Chna2;
  TIM3->CCR3 = Chna3;
  TIM3->CCR4 = Chna4;
}

