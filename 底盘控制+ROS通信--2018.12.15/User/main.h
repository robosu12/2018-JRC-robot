#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"

#include "ad.h"
#include "gun.h"
#include "common.h"
#include "can1.h"
#include "can2.h"
#include "delay.h"
#include "timer.h"
#include "usart1.h"
#include "usart3.h"
#include "led.h"
#include "oled.h"
#include "bsp_flash.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include "mpu6050_interrupt.h"
#include "mpu6050_driver.h"
#include "mpu6050_i2c.h"
#include "imu.h"

#include "Can1BusTask.h"
#include "Can2BusTask.h"
    
#include "laser.h"
#include "bsp.h"
#include "encoder.h"
#include "ControlTask.h"

#include "RemoteTask.h"

#include "JY901.h"
#include "arm_control.h"

void send_data_pc(void);

#endif 
