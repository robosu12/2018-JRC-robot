#ifndef __GUN_H__
#define __GUN_H__

#ifndef FRICTION_WHEEL
#define FRICTION_WHEEL
#endif 

#if defined(FRICTION_WHEEL)

#define PWM1  TIM5->CCR1
#define PWM2  TIM5->CCR2
#define PWM3  TIM9->CCR1


#define InitFrictionWheel()     \
        PWM1 = 0;             \
        PWM2 = 0;
#define SetFrictionWheelSpeed(x) \
        PWM1 = x;                \
        PWM2 = x;

#endif 

void PWM_Configuration(void);
void PWM_Out_Init(int hz);//400hz
 
#endif /* __GUN_H__*/

