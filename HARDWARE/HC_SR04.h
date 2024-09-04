#ifndef __HC_SR04_H__
#define __HC_SR04_H__

#include "gpio.h"

void HC_SR04_Delayus(uint32_t usdelay);
float HC_SR04_Read(void);

#endif
