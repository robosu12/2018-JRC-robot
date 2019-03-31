#ifndef _LASER_H_
#define _LASER_H_
#define LASER_ON()  GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LASER_OFF()  GPIO_ResetBits(GPIOA, GPIO_Pin_8)

void Laser_Configuration(void);

void set_motor_PWM(unsigned char addr,short PWM, u8 mode);
void move_by_track(int line,int turn,u8 mode);
void leg_control(u8 flag);
#endif

