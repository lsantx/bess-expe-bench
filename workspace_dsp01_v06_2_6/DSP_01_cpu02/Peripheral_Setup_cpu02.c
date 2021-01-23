/*
 * Peripheral_Setup.c
 *
 *  Created on: 17 de ago de 2020
 *      Author: xluca
 */


#include "F28x_Project.h"

#include "Peripheral_Interruption_Setup_cpu02.h"
#include "Tupa_parameters_cpu02.h"

void ConfigureADC(void)
{

    Uint16 acqps;
    EALLOW;

    //write configurations
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    // ADCs
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin A2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 15; //trigger on ePWM6 SOCA/C

    //Interrupção ADC_A INT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag (Flag para inicio da interrupção. Todos os Socs estão sincronizados)
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 21; //trigger on ePWM9 SOCA/C

    //Interrupção ADC_A INT2
    AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 1; //end of SOC1 will set INT1 flag (Flag para inicio da interrupção. Todos os Socs estão sincronizados)
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //make sure INT1 flag is cleared

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4;  //SOC2 will convert pin A4
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 23; //trigger on ePWM10 SOCA/C

    //Interrupção ADC_A INT3
    AdcaRegs.ADCINTSEL3N4.bit.INT3SEL = 2; //end of SOC2 will set INT1 flag (Flag para inicio da interrupção. Todos os Socs estão sincronizados)
    AdcaRegs.ADCINTSEL3N4.bit.INT3E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT3 = 1; //make sure INT1 flag is cleared

    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin C2
    AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 15; //trigger on ePWM6 SOCA/C

    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin C3
    AdccRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 15; //trigger on ePWM6 SOCA/C

    AdccRegs.ADCSOC2CTL.bit.CHSEL = 4;  //SOC2 will convert pin C4
    AdccRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 15; //trigger on ePWM6 SOCA/C

    AdccRegs.ADCSOC3CTL.bit.CHSEL = 5;  //SOC3 will convert pin C5
    AdccRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 15; //trigger on ePWM6 SOCA/C

    //power up the ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);

    EDIS;

}

void ConfigureEPWM(void)
{
    EALLOW;

    // EPWM Module6 config

    EPwm6Regs.ETSEL.bit.SOCAEN    = 1;                   // Disable SOC on A group
    EPwm6Regs.ETSEL.bit.SOCASEL   = ET_CTR_ZERO;         // These bits determine when a EPWMxSOCA pulse will be generated.
    EPwm6Regs.ETPS.bit.SOCAPRD    = 1;                    // Generate pulse on 1st event
    EPwm6Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;     // freeze counter
    EPwm6Regs.TBPRD = 50000000/PWM_FREQ;                   // Set period of PWM Fclock/(2*Fpwm)  F_clock = 100MHz
    EPwm6Regs.TBPHS.bit.TBPHS     = 0;                     // Phase is 0
    EPwm6Regs.TBCTL.bit.PHSEN     = TB_ENABLE;             // Disable phase loading
    EPwm6Regs.TBCTL.bit.SYNCOSEL  = TB_SYNC_IN;            // sync with the EPWM6
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;               // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV    = TB_DIV1;

    // Setup shadowing
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM6A on event A, up count
    EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM6A on event A, down count

    //Dead-band Configuration
    EPwm6Regs.DBCTL.bit.IN_MODE = 0;
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active the complementary between the EPWMxA e EPWMxB
    EPwm6Regs.DBFED.bit.DBFED = 10; // Dead-band for falling-edge (100TBCLKs = 1us)
    EPwm6Regs.DBRED.bit.DBRED = 10; // Dead-band for rising-edge (100TBCLKs = 1us)

    // EPWM Module 9 config

    EPwm9Regs.ETSEL.bit.SOCAEN    = 1;                   // Disable SOC on A group
    EPwm9Regs.ETSEL.bit.SOCASEL   = ET_CTR_ZERO;         // These bits determine when a EPWMxSOCA pulse will be generated.
    EPwm9Regs.ETPS.bit.SOCAPRD    = 1;                    // Generate pulse on 1st event
    EPwm9Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;     // freeze counter
    EPwm9Regs.TBPRD = 50000000/PWM_FREQ;                         // Set period of PWM Fclock/(2*Fpwm)  F_clock = 100MHz
    EPwm9Regs.TBPHS.bit.TBPHS     = (EPwm9Regs.TBPRD<<1)/3;                     // Phase is 120 degrees
    EPwm9Regs.TBCTL.bit.PHSEN     = TB_ENABLE;             // Disable phase loading
    EPwm9Regs.TBCTL.bit.SYNCOSEL  = TB_SYNC_IN;            // sync with the EPWM6
    EPwm9Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;               // Clock ratio to SYSCLKOUT
    EPwm9Regs.TBCTL.bit.CLKDIV    = TB_DIV1;

    // Setup shadowing
    EPwm9Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm9Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm9Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm9Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm9Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM6A on event A, up count
    EPwm9Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM6A on event A, down count

    //Dead-band Configuration
    EPwm9Regs.DBCTL.bit.IN_MODE = 0;
    EPwm9Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm9Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active the complementary between the EPWMxA e EPWMxB
    EPwm9Regs.DBFED.bit.DBFED = 10; // Dead-band for falling-edge (100TBCLKs = 1us)
    EPwm9Regs.DBRED.bit.DBRED = 10; // Dead-band for rising-edge (100TBCLKs = 1us)

    // EPWM Module 10 config
    EPwm10Regs.ETSEL.bit.SOCAEN    = 1;                   // Disable SOC on A group
    EPwm10Regs.ETSEL.bit.SOCASEL   = ET_CTR_ZERO;         // These bits determine when a EPWMxSOCA pulse will be generated.
    EPwm10Regs.ETPS.bit.SOCAPRD    = 1;                    // Generate pulse on 1st event
    EPwm10Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;     // freeze counter
    EPwm10Regs.TBPRD = 50000000/PWM_FREQ;                         // Set period of PWM Fclock/(2*Fpwm)  F_clock = 100MHz
    EPwm10Regs.TBPHS.bit.TBPHS     = (EPwm10Regs.TBPRD<<2)/3;     // Phase is 240 degrees
    EPwm10Regs.TBCTL.bit.PHSEN     = TB_ENABLE;             // Disable phase loading
    EPwm10Regs.TBCTL.bit.SYNCOSEL  = TB_SYNC_IN;            // sync with the EPWM6
    EPwm10Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;               // Clock ratio to SYSCLKOUT
    EPwm10Regs.TBCTL.bit.CLKDIV    = TB_DIV1;

    // Setup shadowing
    EPwm10Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm10Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm10Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm10Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm10Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM6A on event A, up count
    EPwm10Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM6A on event A, down count

    //Dead-band Configuration
    EPwm10Regs.DBCTL.bit.IN_MODE = 0;
    EPwm10Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm10Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active the complementary between the EPWMxA e EPWMxB
    EPwm10Regs.DBFED.bit.DBFED = 10; // Dead-band for falling-edge (100TBCLKs = 1us)
    EPwm10Regs.DBRED.bit.DBRED = 10; // Dead-band for rising-edge (100TBCLKs = 1us)


    EDIS;
}

