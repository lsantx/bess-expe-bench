/*
 * Peripheral_Setup.c
 *
 *  Created on: 17 de ago de 2020
 *      Author: xluca
 */


#include "F28x_Project.h"

#include "Peripheral_Interruption_Setup_cpu01.h"
#include "Tupa_parameters_cpu01.h"


void ConfigureADC(void)
{
    Uint16 acqps;
    EALLOW;

    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //Set pulse positions to late
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //Interrup��o ADC_B INT1
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag (Flag para inicio da interrup��o. Todos os Socs est�o sincronizados)
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCs
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4;  //SOC0 will convert pin B4
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC0 will convert pin B3
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC0 will convert pin B2
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 3;  //SOC0 will convert pin D3
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 4;  //SOC1 will convert pin D4
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 5;  //SOC2 will convert pin D5
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 2;  //SOC2 will convert pin D2
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    //power up the ADC
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);

    EDIS;
}

void ConfigureEPWM(void)
{
    // EPWM Module 1 config

    EALLOW;

    EPwm1Regs.ETSEL.bit.SOCAEN   = 1;                  // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL  = ET_CTR_PRD;        // These bits determine when a EPWMxSOCA pulse will be generated.
    EPwm1Regs.ETPS.bit.SOCAPRD  = 1;                   // Generate pulse on 1st event
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;     // updown counter
    EPwm1Regs.TBPRD = 50000000/PWM_FREQ;               // Set period of PWM Fclock/(2*Fpwm)  F_clock = 100MHz
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;                // Phase is 0
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;            // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;           // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;        // Sync down-stream module //TB_SYNC_IN; // sync flow-through
    // Setup shadowing
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;      // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;              // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;                // Clear PWM1A on event A, down count

    //Dead-band Configuration
    EPwm1Regs.DBCTL.bit.IN_MODE = 0;
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active the complementary between the EPWMxA e EPWMxB
    EPwm1Regs.DBFED.bit.DBFED = 50; // Dead-band for falling-edge (100TBCLKs = 1us)
    EPwm1Regs.DBRED.bit.DBRED = 50; // Dead-band for rising-edge (100TBCLKs =  1us)

    // EPWM Module 2 config

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;     // freeze counter
    EPwm2Regs.TBPRD = 50000000/PWM_FREQ;          // Set period of PWM Fclock/(2*Fpwm)  F_clock = 100MHz
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;                // Phase is 0
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;             // Disable phase loading
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;         // sync with the EPWM2
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;           // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM2A on event A, up count
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;              // Clear PWM2A on event A, down count

    //Dead-band Configuration
    EPwm2Regs.DBCTL.bit.IN_MODE = 0;
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active the complementary between the EPWMxA e EPWMxB
    EPwm2Regs.DBFED.bit.DBFED = 50; // Dead-band for falling-edge (100TBCLKs = 1us)
    EPwm2Regs.DBRED.bit.DBRED = 50; // Dead-band for rising-edge (100TBCLKs  =  1us)

    // EPWM Module 5 config

    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;     // freeze counter
    EPwm5Regs.TBPRD = 50000000/PWM_FREQ;          // Set period of PWM Fclock/(2*Fpwm)  F_clock = 100MHz
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000;                // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;             // Disable phase loading
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;         // sync with the EPWM5
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;           // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM5A on event A, up count
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM5A on event A, down count

    //Dead-band Configuration
    EPwm5Regs.DBCTL.bit.IN_MODE = 0;
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active the complementary between the EPWMxA e EPWMxB
    EPwm5Regs.DBFED.bit.DBFED = 50; // Dead-band for falling-edge (100TBCLKs = 1us)
    EPwm5Regs.DBRED.bit.DBRED = 50; // Dead-band for rising-edge (100TBCLKs  = 1us)

    EDIS;

    /* Vai ser ativado no n�cleo 2 (Apagar)
    // EPWM Module6 config

    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;     // freeze counter
    EPwm6Regs.TBPRD = 50000000/PWM_FREQ;          // Set period of PWM Fclock/(2*Fpwm)  F_clock = 100MHz
    EPwm6Regs.TBPHS.bit.TBPHS = 0x0000;                // Phase is 0
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;             // Disable phase loading
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;         // sync with the EPWM6
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;           // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM6A on event A, up count
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM6A on event A, down count

    //Dead-band Configuration
    EPwm6Regs.DBCTL.bit.IN_MODE = 0;
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active the complementary between the EPWMxA e EPWMxB
    EPwm6Regs.DBFED.bit.DBFED = 100; // Dead-band for falling-edge (100TBCLKs = 1us)
    EPwm6Regs.DBRED.bit.DBRED = 100; // Dead-band for rising-edge (100TBCLKs = 1us)

    // EPWM Module9 config

    EPwm9Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;     // freeze counter
    EPwm9Regs.TBPRD = 50000000/PWM_FREQ;          // Set period of PWM Fclock/(2*Fpwm)  F_clock = 100MHz
    EPwm9Regs.TBPHS.bit.TBPHS = 0x0000;                // Phase is 0
    EPwm9Regs.TBCTL.bit.PHSEN = TB_ENABLE;             // Disable phase loading
    EPwm9Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;         // sync with the EPWM9
    EPwm9Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;           // Clock ratio to SYSCLKOUT
    EPwm9Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm9Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm9Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm9Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm9Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm9Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM9A on event A, up count
    EPwm9Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM9A on event A, down count

    //Dead-band Configuration
    EPwm9Regs.DBCTL.bit.IN_MODE = 0;
    EPwm9Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm9Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active the complementary between the EPWMxA e EPWMxB
    EPwm9Regs.DBFED.bit.DBFED = 100; // Dead-band for falling-edge (100TBCLKs = 1us)
    EPwm9Regs.DBRED.bit.DBRED = 100; // Dead-band for rising-edge (100TBCLKs = 1us)

    // EPWM Module10 config

    EPwm10Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;     // freeze counter
    EPwm10Regs.TBPRD = 50000000/PWM_FREQ;          // Set period of PWM Fclock/(2*Fpwm)  F_clock = 100MHz
    EPwm10Regs.TBPHS.bit.TBPHS = 0x0000;                // Phase is 0
    EPwm10Regs.TBCTL.bit.PHSEN = TB_ENABLE;             // Disable phase loading
    EPwm10Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;         // sync with the EPWM9
    EPwm10Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;           // Clock ratio to SYSCLKOUT
    EPwm10Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm10Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm10Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm10Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm10Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm10Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM9A on event A, up count
    EPwm10Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM9A on event A, down count

    //Dead-band Configuration
    EPwm10Regs.DBCTL.bit.IN_MODE = 0;
    EPwm10Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm10Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active the complementary between the EPWMxA e EPWMxB
    EPwm10Regs.DBFED.bit.DBFED = 100; // Dead-band for falling-edge (100TBCLKs = 1us)
    EPwm10Regs.DBRED.bit.DBRED = 100; // Dead-band for rising-edge (100TBCLKs = 1us)
    */

    EDIS;
}

void GPIO_Configure()
{
    EALLOW;

    //////////////////////// Digital Outputs - Relays //////////////////
    //R1A - Relay K3
    GpioCtrlRegs.GPBPUD.bit.GPIO59   =  0; // Enable pullup on GPIO59
    GpioDataRegs.GPBCLEAR.bit.GPIO59 =  1;   // Load output latch
    GpioCtrlRegs.GPBMUX2.bit.GPIO59  =  0; // GPIO59 = GPIO59
    GpioCtrlRegs.GPBDIR.bit.GPIO59   =  1; // GPIO59 = output

    //R2A - Relay K4
    GpioCtrlRegs.GPBPUD.bit.GPIO58   =  0; // Enable pullup on GPIO58
    GpioDataRegs.GPBCLEAR.bit.GPIO58 =  1;   // Load output latch
    GpioCtrlRegs.GPBMUX2.bit.GPIO58  =  0; // GPIO58 = GPIO58
    GpioCtrlRegs.GPBDIR.bit.GPIO58   =  1; // GPIO58 = output

    //R3A - Relay K9
    GpioCtrlRegs.GPBPUD.bit.GPIO57   =  0; // Enable pullup on GPIO57
    GpioDataRegs.GPBCLEAR.bit.GPIO57 =  1;   // Load output latch
    GpioCtrlRegs.GPBMUX2.bit.GPIO57  =  0; // GPIO57 = GPIO57
    GpioCtrlRegs.GPBDIR.bit.GPIO57   =  1; // GPIO57 = output
    GpioCtrlRegs.GPBCSEL4.bit.GPIO57 =  GPIO_MUX_CPU2; // CPU02 controla esta GPIO

    //R4A - Relay K10
    GpioCtrlRegs.GPBPUD.bit.GPIO56   =  0; // Enable pullup on GPIO56
    GpioDataRegs.GPBCLEAR.bit.GPIO56 =  1;   // Load output latch
    GpioCtrlRegs.GPBMUX2.bit.GPIO56  =  0; // GPIO56 = GPIO56
    GpioCtrlRegs.GPBDIR.bit.GPIO56   =  1; // GPIO56 = output
    GpioCtrlRegs.GPBCSEL4.bit.GPIO56 =  GPIO_MUX_CPU2; // CPU02 controla esta GPIO

    //R1B - Relay K11
    GpioCtrlRegs.GPBPUD.bit.GPIO55   =  0; // Enable pullup on GPIO55
    GpioDataRegs.GPBCLEAR.bit.GPIO55 =  1;   // Load output latch
    GpioCtrlRegs.GPBMUX2.bit.GPIO55  =  0; // GPIO55 = GPIO55
    GpioCtrlRegs.GPBDIR.bit.GPIO55   =  1; // GPIO55 = output
    GpioCtrlRegs.GPBCSEL3.bit.GPIO55 =  GPIO_MUX_CPU2; // CPU02 controla esta GPIO

    //R2B
    GpioCtrlRegs.GPBPUD.bit.GPIO54   =  0; // Enable pullup on GPIO54
    GpioDataRegs.GPBCLEAR.bit.GPIO54 =  1; // Load output latch
    GpioCtrlRegs.GPBMUX2.bit.GPIO54  =  0; // GPIO54 = GPIO54
    GpioCtrlRegs.GPBDIR.bit.GPIO54   =  1; // GPIO54 = output

    //R3B
    GpioCtrlRegs.GPBPUD.bit.GPIO45   =  0; // Enable pullup on GPIO45
    GpioDataRegs.GPBCLEAR.bit.GPIO45 =  1; // Load output latch
    GpioCtrlRegs.GPBMUX1.bit.GPIO45  =  0; // GPIO45 = GPIO45
    GpioCtrlRegs.GPBDIR.bit.GPIO45   =  1; // GPIO45 = output

    //R4B
    GpioCtrlRegs.GPBPUD.bit.GPIO44   =  0; // Enable pullup on GPIO44
    GpioDataRegs.GPBCLEAR.bit.GPIO44 =  1; // Load output latch
    GpioCtrlRegs.GPBMUX1.bit.GPIO44  =  0; // GPIO44 = GPIO44
    GpioCtrlRegs.GPBDIR.bit.GPIO44   =  1; // GPIO44 = output

    //////////////////////// Digital Outputs  //////////////////
    //D1
    GpioCtrlRegs.GPBPUD.bit.GPIO62   =  0; // Enable pullup on GPIO62
    GpioDataRegs.GPBCLEAR.bit.GPIO62 =  1; // Load output latch
    GpioCtrlRegs.GPBMUX2.bit.GPIO62  =  0; // GPIO62 = GPIO62
    GpioCtrlRegs.GPBDIR.bit.GPIO62   =  1; // GPIO62 = output

    //D2
    GpioCtrlRegs.GPCPUD.bit.GPIO64   =  0; // Enable pullup on GPIO64
    GpioDataRegs.GPCCLEAR.bit.GPIO64 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX1.bit.GPIO64  =  0; // GPIO64 = GPIO64
    GpioCtrlRegs.GPCDIR.bit.GPIO64   =  1; // GPIO64 = output
    GpioCtrlRegs.GPCCSEL1.bit.GPIO64 =  GPIO_MUX_CPU2; // CPU02 controla esta GPIO

    //D3
    GpioCtrlRegs.GPCPUD.bit.GPIO66   =  0; // Enable pullup on GPIO66
    GpioDataRegs.GPCCLEAR.bit.GPIO66 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX1.bit.GPIO66  =  0; // GPIO66 = GPIO66
    GpioCtrlRegs.GPCDIR.bit.GPIO66   =  1; // GPIO66 = output
    GpioCtrlRegs.GPCCSEL1.bit.GPIO66 =  GPIO_MUX_CPU2; // CPU02 controla esta GPIO

    //D4
    GpioCtrlRegs.GPCPUD.bit.GPIO68   =  0; // Enable pullup on GPIO68
    GpioDataRegs.GPCCLEAR.bit.GPIO68 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX1.bit.GPIO68  =  0; // GPIO68 = GPIO68
    GpioCtrlRegs.GPCDIR.bit.GPIO68   =  1; // GPIO68 = output
    GpioCtrlRegs.GPCCSEL1.bit.GPIO68 =  GPIO_MUX_CPU2; // CPU02 controla esta GPIO

    //D5
    GpioCtrlRegs.GPCPUD.bit.GPIO70   =  0; // Enable pullup on GPIO70
    GpioDataRegs.GPCCLEAR.bit.GPIO70 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX1.bit.GPIO70  =  0; // GPIO70 = GPIO70
    GpioCtrlRegs.GPCDIR.bit.GPIO70   =  1; // GPIO70 = output

    //D6
    GpioCtrlRegs.GPCPUD.bit.GPIO72   =  0; // Enable pullup on GPIO72
    GpioDataRegs.GPCCLEAR.bit.GPIO72 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX1.bit.GPIO72  =  0; // GPIO72 = GPIO72
    GpioCtrlRegs.GPCDIR.bit.GPIO72   =  1; // GPIO72 = output

    //D7
    GpioCtrlRegs.GPCPUD.bit.GPIO74   =  0; // Enable pullup on GPIO74
    GpioDataRegs.GPCCLEAR.bit.GPIO74 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX1.bit.GPIO74  =  0; // GPIO74 = GPIO74
    GpioCtrlRegs.GPCDIR.bit.GPIO74   =  1; // GPIO74 = output

    //D8
    GpioCtrlRegs.GPCPUD.bit.GPIO76   =  0; // Enable pullup on GPIO76
    GpioDataRegs.GPCCLEAR.bit.GPIO76 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX1.bit.GPIO76  =  0; // GPIO76 = GPIO76
    GpioCtrlRegs.GPCDIR.bit.GPIO76   =  1; // GPIO76 = output

    //D9
    GpioCtrlRegs.GPCPUD.bit.GPIO78   =  0; // Enable pullup on GPIO78
    GpioDataRegs.GPCCLEAR.bit.GPIO78 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX1.bit.GPIO78  =  0; // GPIO78 = GPIO78
    GpioCtrlRegs.GPCDIR.bit.GPIO78   =  1; // GPIO78 = output

    //D10
    GpioCtrlRegs.GPCPUD.bit.GPIO80   =  0; // Enable pullup on GPIO80
    GpioDataRegs.GPCCLEAR.bit.GPIO80 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO80  =  0; // GPIO80 = GPIO80
    GpioCtrlRegs.GPCDIR.bit.GPIO80   =  1; // GPIO80 = output

    //D11
    GpioCtrlRegs.GPCPUD.bit.GPIO82   =  0; // Enable pullup on GPIO82
    GpioDataRegs.GPCCLEAR.bit.GPIO82 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO82  =  0; // GPIO82 = GPIO82
    GpioCtrlRegs.GPCDIR.bit.GPIO82   =  1; // GPIO82 = output

    //D12
    GpioCtrlRegs.GPCPUD.bit.GPIO84   =  0; // Enable pullup on GPIO84
    GpioDataRegs.GPCCLEAR.bit.GPIO84 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO84  =  0; // GPIO84 = GPIO84
    GpioCtrlRegs.GPCDIR.bit.GPIO84   =  1; // GPIO84 = output

    //D13
    GpioCtrlRegs.GPCPUD.bit.GPIO86   =  0; // Enable pullup on GPIO86
    GpioDataRegs.GPCCLEAR.bit.GPIO86 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO86  =  0; // GPIO86 = GPIO86
    GpioCtrlRegs.GPCDIR.bit.GPIO86   =  1; // GPIO86 = output

    //D14
    GpioCtrlRegs.GPCPUD.bit.GPIO88   =  0; // Enable pullup on GPIO88
    GpioDataRegs.GPCCLEAR.bit.GPIO88 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO88  =  0; // GPIO88 = GPIO88
    GpioCtrlRegs.GPCDIR.bit.GPIO88   =  1; // GPIO88 = output

    //D15
    GpioCtrlRegs.GPCPUD.bit.GPIO90   =  0; // Enable pullup on GPIO90
    GpioDataRegs.GPCCLEAR.bit.GPIO90 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO90  =  0; // GPIO90 = GPIO90
    GpioCtrlRegs.GPCDIR.bit.GPIO90   =  1; // GPIO90 = output

    //D16
    GpioCtrlRegs.GPCPUD.bit.GPIO92   =  0; // Enable pullup on GPIO92
    GpioDataRegs.GPCCLEAR.bit.GPIO92 =  1; // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO92  =  0; // GPIO92 = GPIO92
    GpioCtrlRegs.GPCDIR.bit.GPIO92   =  1; // GPIO92 = output

/////////////////////// Digital Inputs /////////////////////////////////
    //D1 - Contactor Q3
    GpioCtrlRegs.GPBPUD.bit.GPIO36  = 0;  // Enable pullup on GPIO36
    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;  // GPIO36 = GPIO36
    GpioCtrlRegs.GPBDIR.bit.GPIO36  = 0;  // GPIO36 = input

    //D2 - Contactor Q4
    GpioCtrlRegs.GPBPUD.bit.GPIO38  = 0;  // Enable pullup on GPIO38
    GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;  // GPIO38 = GPIO38
    GpioCtrlRegs.GPBDIR.bit.GPIO38  = 0;  // GPIO38 = input

    //D3 - Contactor Q9
    GpioCtrlRegs.GPBPUD.bit.GPIO61  = 0;  // Enable pullup on GPIO61
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;  // GPIO61 = GPIO61
    GpioCtrlRegs.GPBDIR.bit.GPIO61  = 0;  // GPIO61 = input
    GpioCtrlRegs.GPBCSEL4.bit.GPIO61 =  GPIO_MUX_CPU2; // CPU02 controla esta GPIO

    //D4 - Contactor Q10
    GpioCtrlRegs.GPBPUD.bit.GPIO63  = 0;  // Enable pullup on GPIO63
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0;  // GPIO63 = GPIO63
    GpioCtrlRegs.GPBDIR.bit.GPIO63  = 0;  // GPIO63 = input
    GpioCtrlRegs.GPBCSEL4.bit.GPIO63 =  GPIO_MUX_CPU2; // CPU02 controla esta GPIO

    //D5 - Contactor Q11
    GpioCtrlRegs.GPCPUD.bit.GPIO65  = 0;  // Enable pullup on GPIO65
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;  // GPIO65 = GPIO65
    GpioCtrlRegs.GPCDIR.bit.GPIO65  = 0;  // GPIO65 = input
    GpioCtrlRegs.GPCCSEL1.bit.GPIO65 =  GPIO_MUX_CPU2; // CPU02 controla esta GPIO

    //D6 - Contactor Q14
    GpioCtrlRegs.GPCPUD.bit.GPIO67  = 0;  // Enable pullup on GPIO67
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;  // GPIO67 = GPIO67
    GpioCtrlRegs.GPCDIR.bit.GPIO67  = 0;  // GPIO67 = input

    //D7 - Contactor TE3
    GpioCtrlRegs.GPCPUD.bit.GPIO69   = 0;  // Enable pullup on GPIO69
    GpioCtrlRegs.GPCMUX1.bit.GPIO69  = 0;  // GPIO69 = GPIO69
    GpioCtrlRegs.GPCDIR.bit.GPIO69   = 0;  // GPIO69 = input

    //D8 - Contactor TE4
    GpioCtrlRegs.GPCPUD.bit.GPIO71   = 0;  // Enable pullup on GPIO71
    GpioCtrlRegs.GPCMUX1.bit.GPIO71  = 0;  // GPIO71 = GPIO71
    GpioCtrlRegs.GPCDIR.bit.GPIO71   = 0;  // GPIO71 = input

    //D9
    GpioCtrlRegs.GPCPUD.bit.GPIO73   = 0;  // Enable pullup on GPIO73
    GpioCtrlRegs.GPCMUX1.bit.GPIO73  = 0;  // GPIO73 = GPIO73
    GpioCtrlRegs.GPCDIR.bit.GPIO73   = 0;  // GPIO73 = input

    //D10
    GpioCtrlRegs.GPCPUD.bit.GPIO75   = 0;  // Enable pullup on GPIO75
    GpioCtrlRegs.GPCMUX1.bit.GPIO75  = 0;  // GPIO75 = GPIO75
    GpioCtrlRegs.GPCDIR.bit.GPIO75   = 0;  // GPIO75 = input

    //D11
    GpioCtrlRegs.GPCPUD.bit.GPIO77   = 0;  // Enable pullup on GPIO77
    GpioCtrlRegs.GPCMUX1.bit.GPIO77  = 0;  // GPIO77 = GPIO77
    GpioCtrlRegs.GPCDIR.bit.GPIO77   = 0;  // GPIO77 = input

    //D12
    GpioCtrlRegs.GPCPUD.bit.GPIO79   = 0;  // Enable pullup on GPIO79
    GpioCtrlRegs.GPCMUX1.bit.GPIO79  = 0;  // GPIO79 = GPIO79
    GpioCtrlRegs.GPCDIR.bit.GPIO79   = 0;  // GPIO79 = input

    //D13
    GpioCtrlRegs.GPCPUD.bit.GPIO81   = 0;  // Enable pullup on GPIO81
    GpioCtrlRegs.GPCMUX2.bit.GPIO81  = 0;  // GPIO81 = GPIO81
    GpioCtrlRegs.GPCDIR.bit.GPIO81   = 0;  // GPIO81 = input

    //D14
    GpioCtrlRegs.GPCPUD.bit.GPIO83   = 0;  // Enable pullup on GPIO83
    GpioCtrlRegs.GPCMUX2.bit.GPIO83  = 0;  // GPIO83 = GPIO83
    GpioCtrlRegs.GPCDIR.bit.GPIO83   = 0;  // GPIO83 = input

    //D15
    GpioCtrlRegs.GPCPUD.bit.GPIO85   = 0;  // Enable pullup on GPIO85
    GpioCtrlRegs.GPCMUX2.bit.GPIO85  = 0;  // GPIO85 = GPIO85
    GpioCtrlRegs.GPCDIR.bit.GPIO85   = 0;  // GPIO85 = input

    //D16
    GpioCtrlRegs.GPCPUD.bit.GPIO87   = 0;  // Enable pullup on GPIO87
    GpioCtrlRegs.GPCMUX2.bit.GPIO87  = 0;  // GPIO87 = GPIO87
    GpioCtrlRegs.GPCDIR.bit.GPIO87   = 0;  // GPIO87 = input


/////////////////////////////// Chopper PWM GPIO Initialization - Output/////////////////////////////////
    GpioCtrlRegs.GPBPUD.bit.GPIO33   =  0; // Enable pullup on GPIO33
    GpioDataRegs.GPBCLEAR.bit.GPIO33 =  1; // Load output latch
    GpioCtrlRegs.GPBMUX1.bit.GPIO33  =  0; // GPIO33 = GPIO33
    GpioCtrlRegs.GPBDIR.bit.GPIO33   =  1; // GPIO33 = output

/////////////////////////////// Gatedriver reset GPIO Initialization - Chopper - Output///////////////////////////////// OBS: Esse gate drive n�o tem reset. O pino do gatedrive correspondente ao reset � NC. Por isso, todos os pinos de reset � for�ado para zero (Clear)
    GpioCtrlRegs.GPAPUD.bit.GPIO27   =  0; // Enable pullup on GPIO27
    GpioDataRegs.GPACLEAR.bit.GPIO27 =  1; // Load output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO27  =  0; // GPIO27 = GPIO27
    GpioCtrlRegs.GPADIR.bit.GPIO27   =  1; // GPIO27 = output

/////////////////////////////// Gatedriver error GPIO Initialization - Chopper - Input/////////////////////////////////
    GpioCtrlRegs.GPBPUD.bit.GPIO48   = 0;  // Enable pullup on GPIO48
    GpioCtrlRegs.GPBMUX2.bit.GPIO48  = 0;  // GPIO48 = GPIO48
    GpioCtrlRegs.GPBDIR.bit.GPIO48   = 0;  // GPIO48 = input

/////////////////////////////// Gatedriver reset GPIO Initialization - Converters - Output///////////////////////////////// OBS: Esse gate drive n�o tem reset. O pino do gatedrive correspondente ao reset � NC. Por isso, todos os pinos de reset � for�ado para zero (Clear)
    GpioCtrlRegs.GPBPUD.bit.GPIO32   =  0; // Enable pullup on GPIO32
    GpioDataRegs.GPBCLEAR.bit.GPIO32 =  1; // Load output latch
    GpioCtrlRegs.GPBMUX1.bit.GPIO32  =  0; // GPIO32 = GPIO32
    GpioCtrlRegs.GPBDIR.bit.GPIO32   =  1; // GPIO32 = output

/////////////////////////////// Gatedriver error GPIOs Initialization - Converters - Input/////////////////////////////////
    //GSC
    GpioCtrlRegs.GPBPUD.bit.GPIO41   = 0;  // Enable pullup on GPIO41
    GpioCtrlRegs.GPBMUX1.bit.GPIO41  = 0;  // GPIO41 = GPIO48
    GpioCtrlRegs.GPBDIR.bit.GPIO41   = 0;  // GPIO41 = input
    //BSC
    GpioCtrlRegs.GPBPUD.bit.GPIO40   = 0;  // Enable pullup on GPIO40
    GpioCtrlRegs.GPBMUX1.bit.GPIO40  = 0;  // GPIO40 = GPIO48
    GpioCtrlRegs.GPBDIR.bit.GPIO40   = 0;  // GPIO40 = input

    //////////////////////////////////LED/////////////////////////////////////////////////////////////////////////////
    //Led 2
    GpioCtrlRegs.GPAPUD.bit.GPIO31   =  1; // Enable pullup on GPIO31
    GpioDataRegs.GPACLEAR.bit.GPIO31 =  1; // Load output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO31  =  0; // GPIO31 = GPIO31
    GpioCtrlRegs.GPADIR.bit.GPIO31   =  1; // GPIO31 = output
    GpioCtrlRegs.GPACSEL4.bit.GPIO31 =  GPIO_MUX_CPU1; // CPU01 controla esta GPIO

    //Led 3
    GpioCtrlRegs.GPBPUD.bit.GPIO34   =  1; // Enable pullup on GPIO31
    GpioDataRegs.GPBCLEAR.bit.GPIO34 =  1; // Load output latch
    GpioCtrlRegs.GPBMUX1.bit.GPIO34  =  0; // GPIO31 = GPIO31
    GpioCtrlRegs.GPBDIR.bit.GPIO34   =  1; // GPIO31 = output
    GpioCtrlRegs.GPBCSEL1.bit.GPIO34 =  GPIO_MUX_CPU2; // CPU02 controla esta GPIO

    //////////////////////// Comunica��o DSP 2 - 1 //////////////////
    //GPIO24 (Input) - Prote��o
    GpioCtrlRegs.GPAPUD.bit.GPIO24  = 1;  // Enable pullup on GPIO24
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;  // GPIO24 = GPIO24
    GpioCtrlRegs.GPADIR.bit.GPIO24  = 0;  // GPIO24 = input

    //GPIO25 (Output) - Prote��o
    GpioCtrlRegs.GPAPUD.bit.GPIO25   =  1; // Disable pullup on GPIO25
    GpioDataRegs.GPACLEAR.bit.GPIO25 =  1;   // Load output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO25  =  0; // GPIO25 = GPIO25
    GpioCtrlRegs.GPADIR.bit.GPIO25   =  1; // GPIO25 = output

    //GPIO26 (Input) - Sincroniza��o entre as DSPs dos dois Kits
    GpioCtrlRegs.GPAPUD.bit.GPIO26  = 1;  // Enable pullup on GPIO26
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;  // GPIO26 = GPIO26
    GpioCtrlRegs.GPADIR.bit.GPIO26  = 0;  // GPIO26 = input

    EDIS;
}

// scia_fifo_init - Configure SCIA FIFO
//
void scia_fifo_init(void)
{
   SciaRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                      // No parity,8 char bits,
                                      // async mode, idle-line protocol
   SciaRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
   SciaRegs.SCICTL2.bit.TXINTENA = 1;
   SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
   SciaRegs.SCIHBAUD.all = ((uint16_t)SCI_PRD  & 0xFF00U) >> 8U;
   SciaRegs.SCILBAUD.all = (uint16_t)SCI_PRD  & 0x00FFU;
//   SciaRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back
   SciaRegs.SCIFFTX.all = 0xC023;
   SciaRegs.SCIFFRX.all = 0x023;
   SciaRegs.SCIFFCT.all = 0x00;

   SciaRegs.SCIFFTX.bit.TXFFIL = 8;
   SciaRegs.SCIFFRX.bit.RXFFIL = 8;

   SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
   SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

void Setup_DAC(void)
{
    EALLOW;
    DacaRegs.DACCTL.bit.SYNCSEL     =  0x00;
    DacaRegs.DACCTL.bit.LOADMODE    =  0x01;
    DacaRegs.DACCTL.bit.DACREFSEL   =  0x01;
    DacaRegs.DACVALS.bit.DACVALS    =  0;
    DacaRegs.DACOUTEN.bit.DACOUTEN  =  1;
    DacaRegs.DACLOCK.all            =  1;
    EDIS;
}
