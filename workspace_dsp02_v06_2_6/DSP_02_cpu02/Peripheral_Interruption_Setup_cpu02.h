/*
 * Peripheral_Setup.h
 *
 *  Created on: 17 de ago de 2020
 *      Author: xluca
 */

#ifndef PERIPHERAL_INTERRUPTION_SETUP_CPU02_H_
#define PERIPHERAL_INTERRUPTION_SETUP_CPU02_H_
#include "F28x_Project.h"

//ADCs
void ConfigureADC(void);

//ADC_A
void ConfigureADC(void);

//EPW
void ConfigureEPWM(void);

//GPIo
void  GPIO_Configure(void);

//Interrup��es
__interrupt void adca1_isr(void);
__interrupt void adca2_isr(void);
__interrupt void adca3_isr(void);
__interrupt void IPC2_INT(void);
__interrupt void IPC3_INT(void);




#endif /* PERIPHERAL_INTERRUPTION_SETUP_CPU02_H_ */
