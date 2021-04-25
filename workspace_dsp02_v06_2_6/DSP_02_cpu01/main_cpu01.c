//###########################################################################
//
// TITLE:  BESS

//
// Included Files
//
#include "F28x_Project.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Peripheral_Interruption_Setup_cpu01.h"
#include "Tupa_parameters_cpu01.h"

//////////////////////////////////////////////////////// Strutures //////////////////////////////////////////////////////////
// Dc-link and grid signals
typedef struct{
    float Vab;
    float Vbc;
    float Vca;
    float Va;
    float Vb;
    float Vc;
    float Vdc;
    float Ia;
    float Ib;
    float Ic;
    float Io;
}sSignal_red;

#define SIGNAL_RED_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0}
sSignal_red entradas_red = SIGNAL_RED_DEFAULTS;

//abc
typedef struct{
    float a;
    float b;
    float c;
} sABC;

#define ABC_DEFAULTS {0,0,0}
sABC Vabc = ABC_DEFAULTS;
sABC Iabc = ABC_DEFAULTS;
sABC Vabc_pwm = ABC_DEFAULTS;

//Alfa-beta
typedef struct{
    float alfa;
    float beta;
} sAlfaBeta;

#define ALFABETA_DEFAULTS {0,0}
sAlfaBeta Valfabeta = ALFABETA_DEFAULTS;
sAlfaBeta Ialfabeta = ALFABETA_DEFAULTS;
sAlfaBeta Valfabeta_pwm = ALFABETA_DEFAULTS;

//dq
typedef struct{
    float d;
    float q;
    float sindq;
    float cosdq;
} sDQ;

#define DQ_DEFAULTS {0,0,0,0}

//SOGI
typedef struct {
    float x;
    float y;
    float W;
    float b0;
    float b1;
    float b2;
    float b3;
    float a1;
    float a2;
    float V_sogi;
    float V_sogi1;
    float V_sogi2;
    float V_sogi_q;
    float V_sogi_q1;
    float V_sogi_q2;
    float Vm;
    float Vm1;
    float Vm2;
    float K_damp;
    float freq_res;
} sSOGI;

#define SOGI_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,60}
sSOGI SOG = SOGI_DEFAULTS;
sSOGI SOGB = SOGI_DEFAULTS;

//PI Controller
typedef struct {
    int enab;
    float feedback;
    float setpoint;
    float error;
    float error_ant;
    float integr;
    float integr_ant;
    float Kp;
    float Ki;
    float output;
    float outMin;
    float outMax;
    float feedforward;
} sPI;

#define PI_PLL_GRID_DEFAULTS {0,0, 0, 0, 0, 0, 0, PI_PLL_GRID_KP, PI_PLL_GRID_KI,0,PI_PLL_GRID_OUTMIN , PI_PLL_GRID_OUTMAX, 0}
#define PI_Vdc_DEFAULTS {0, 0, 0, 0, 0, 0, 0, PI_Vdc_KP, PI_Vdc_KI, 0,PI_Vdc_OUTMIN , PI_Vdc_OUTMAX, 0}
#define PI_Q_DEFAULTS   {0, 0, 0, 0, 0, 0, 0, 0, PI_Q_KI, 0, PI_Q_OUTMIN , PI_Q_OUTMAX, 0}
sPI pi_Vdc = PI_Vdc_DEFAULTS;
sPI pi_Q = PI_Q_DEFAULTS;

//PLL
typedef struct {
    float amplitude;
    float freq;
    float omega;
    float omega_ant;
    float omega_init;
    float sinth;
    float costh;
    float theta;
    float theta_ant;
    float alfa;
    float beta;
    float Dpos;
    float Qpos;
    sPI PI_pll;
    sAlfaBeta alfabeta;
    sDQ dq;
} sSRFPLL;

#define SRFPLL_DEFAULTS {180,60,376.99,376.99,376.99,0,0,0,0,0,0,0,0,PI_PLL_GRID_DEFAULTS,ALFABETA_DEFAULTS,DQ_DEFAULTS}
sSRFPLL pll_grid = SRFPLL_DEFAULTS;

//PWM
typedef struct{
    Uint16 Ta;
    Uint16 Tb;
    Uint16 Tc;
} sSvm;

#define PWM_DEFAULTS {0,0,0}
sSvm sv_grid = PWM_DEFAULTS;

//Firs order LPF
typedef struct {
    float Un;
    float Un_1;
    float Yn_1;
    float Yn;
    float c0;
    float c1;
    } sFilter1st;
#define FILTER_DEFAULTS {0,0,0,0,0.00069813170079773186,0.00069813170079773186}
#define FILTER_DEFAULTS_2_5_HZ {0,0,0,0,0.00174532925199432959,0.00174532925199432959}
#define FILTER_DEFAULTS_1_HZ {0,0,0,0,0.00069813170079773186,0.00069813170079773186}

sFilter1st Filt_freq_pll = FILTER_DEFAULTS;
sFilter1st Filt_freq_Vdc = FILTER_DEFAULTS_1_HZ;

typedef struct{
float x;
float x_ant;
float x_ant2;
float y;
float y_ant;
float y_ant2;
double c0;
double c1;
double c2;
double c3;
double c4;
} sFilter2nd;

#define  FILTER2ND_DEFAULTS_5Hz {0,0,0,0,0,0,0.00000303866583164036,0.00000607733166328071,0.00000303866583164036,-1.99506422020287144115,0.99507637486619804346}   //5Hz
#define  FILTER2ND_DEFAULTS_10Hz {0,0,0,0,0,0,0.00001212470404899192,0.00002424940809798384,0.00001212470404899192,-1.99012852279409080403,0.99017702161028686714}   //10Hz
sFilter2nd fil2nQ   = FILTER2ND_DEFAULTS_10Hz;
sFilter2nd fil2nP   = FILTER2ND_DEFAULTS_10Hz;

//Flags
typedef struct{
    unsigned int GSC_PulsesOn;
    unsigned int BSC_PulsesOn;
    unsigned int Chopper_On;
    unsigned int Shutdown;
    unsigned int AbleToStart;
    unsigned int Inv_on;
    unsigned int real_time_buff;
    unsigned int precharge_ok;
    unsigned int precharge_fail;
    unsigned int manual_pre_charge;
    unsigned int Com_DSP2_read;
    unsigned int data_logo_init;

}sFlags;

#define FLAGS_DEFAULTS {0,0,0,0,0,0,1,0,0,0,0,0}
sFlags flag = FLAGS_DEFAULTS;

//Ressonante
typedef struct {
    double c1;
    double c2;
    double c3;
    double c4;
    float feedback;
    float setpoint;
    float error;
    float error_ant;
    float error_ant2;
    float res;
    float res_ant;
    float res_ant2;
    float Kp;
    float Ki;
    float output;
    float res_init;
    float feedforward;
    Uint16 enab;
    } sPR;

#define PR_I_FUND_DEFAULTS {0.00005553931071838902,-0.00005553931071838902,-1.99824566019771654446,0.99999999999999977796,0,0,0,0,0,0,0,0,\
    PR_I_GRID_KP, PR_I_GRID_KI, 0, 0, 0, 0}

#define PR_I_5_DEFAULTS {0.00005515028886706705,-0.00005515028886706705,-1.95629520146761159971,1.00000000000000022204,0,0,0,0,0,0,0,0,\
    0, PR_I_GRID_KI, 0, 0, 0, 0}

#define PR_I_7_DEFAULTS {0.00005476290380291145,-0.00005476290380291145,-1.91463899506413470775,1.00000000000000022204,0,0,0,0,0,0,0,0,\
    0, PR_I_GRID_KI, 0, 0, 0, 0}

#define PR_I_11_DEFAULTS {0.00005361052018169078,-0.00005361052018169078,-1.79142352047882602584,1.00000000000000044409,0,0,0,0,0,0,0,0,\
    0, PR_I_GRID_KI, 0, 0, 0, 0}

sPR PR_Ia_fund = PR_I_FUND_DEFAULTS;
sPR PR_Ib_fund = PR_I_FUND_DEFAULTS;
sPR PR_Ia_5 = PR_I_5_DEFAULTS;
sPR PR_Ib_5 = PR_I_5_DEFAULTS;
sPR PR_Ia_7 = PR_I_7_DEFAULTS;
sPR PR_Ib_7 = PR_I_7_DEFAULTS;
sPR PR_Ia_11 = PR_I_11_DEFAULTS;
sPR PR_Ib_11 = PR_I_11_DEFAULTS;

//Ramp
typedef struct{
int   enab;
float final;
float final_ant;
float atual;
float in;
float delta;
int   flag;
int   flag2;
float range;
float inc;
} Ramp;

#define VRamp_default {0,0,0,0,0,0,0,0,0.1,0.005}
#define QRamp_default {0,0,0,0,0,0,0,0,0.1,0.06}
Ramp VRamp = VRamp_default;
Ramp QRamp = QRamp_default;

typedef struct{
    int CH_1;
    int CH_2;
    int CH_3;
    int CH_4;
    int CH_5;
    int CH_6;
}sChannel_adc;

#define CHANNEL_DEFAULTS {0,0,0,0,0,0}
sChannel_adc channel_offset = CHANNEL_DEFAULTS;

//Counters
typedef struct{
    Uint32 count1;
    Uint32 count2;
    Uint32 count3;
    Uint32 count4;
    Uint32 count5;
    Uint32 count6;
    Uint32 count7;
    Uint32 count8;
    Uint32 count9;
}counts;

#define COUNTS_DEFAULTS {0,0,0,0,0,0,0,0,0}
counts Counts = COUNTS_DEFAULTS;

//Variables to send data from CPU1 to CPU2
typedef struct{
    unsigned int send0;
    float send1;
}SsendCPU1toCPU2;

#define SEND_DEFAULTS {0,0}
SsendCPU1toCPU2 Send = SEND_DEFAULTS;

//Variables to receive data from CPU1 to CPU2
typedef struct{
    unsigned int *recv0;
    float *recv1;
}SrecvCPU2toCPU1;

#define RECV_DEFAULTS {0,0}
SrecvCPU2toCPU1 Recv = RECV_DEFAULTS;

///////////////////////////////////////////// Functions ////////////////////////////////////////
// Control
void TUPA_abc2alfabeta(sABC *, sAlfaBeta *);
void TUPA_alfabeta2dq(sAlfaBeta*, sDQ* );
void TUPA_alfabeta2abc(sAlfaBeta* , sABC* );
void TUPA_SOGI(sSOGI *);
void TUPA_SRFPLL(sSRFPLL *);
void TUPA_Pifunc(sPI *);
void TUPA_pwm(sABC *, sSvm *, float, Uint16);
void TUPA_protect(void);
void TUPA_StartSequence(void);
void TUPA_StopSequence(void);
void Offset_Calculation(void);
void TUPA_First_order_signals_filter(sFilter1st *);
void TUPA_PR(sPR *);
void TUPA_Ramp(Ramp *);
void TUPA_Second_order_filter(sFilter2nd *);
void TxBufferAqu(void);
////////////////////////////////////////////// Global Variables ////////////////////////////////////

//Variable for offset adjustment
float inv_nro_muestras = 0;

//Test variable
float  gn = 762;

//#pragma DATA_SECTION(gn,"SHARERAMGS1");         // Place the variable that will be shared with CPU02 in the following memory segment

//Buffers para plot
float32 AdcResults[RESULTS_BUFFER_SIZE];
float32 AdcResults2[RESULTS_BUFFER_SIZE];
float32 AdcResults3[RESULTS_BUFFER_SIZE];

//Buffers para armazenamento de dados
float aqui_sign1[N_data_log];
float aqui_sign2[N_data_log];

//CPU comunication variables
int send = 0;

// Control Variables
float Ts = TSAMPLE;
float Van = 0, Vbn = 0, Vcn = 0 , vmin = 0, vmax = 0, Vao = 0, Vbo = 0, Vco = 0;

float  Iref = 0;
float  Vdc_ref = 480;
float  Q_ref   = 0;
float  Qm      = 0;
float  Pm      = 0;

int selecao_plot = 0;
Uint16 fault = FAULT_OK;
Uint32 resultsIndex = 0;
Uint32 resultsIndex2 = 0;
//Variables for offset adjustment
Uint32 first_scan = 1;
Uint32 sum_CH1 = 0; Uint32 sum_CH2 = 0; Uint32 sum_CH3 = 0; Uint32 sum_CH4 = 0; Uint32 sum_CH5 = 0; Uint32 sum_CH6 = 0;
Uint32 N_amostras = 60000;

// SCI parameters
int pref = 0;
int qref = 0;
float pout = 0;
float qout = 0;
float soc = 0;
Uint16 i = 0;
char msg_tx[19];
char msg_rx[19];
char reset[19] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Uint16 len_tx = 0;

//Main
void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    InitGpio(); // Skipped for this example

// init the pins for the SCI-A port.
//  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
//  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
// These functions are found in the F2837xD_Gpio.c file.
//
   GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
   GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
   GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;  // Disable the PIE

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

//
// Map ISR functions
//
    EALLOW;
    PieVectTable.ADCB1_INT = &adcb1_isr;  //function for ADCB interrupt 1
    PieVectTable.IPC1_INT = &IPC1_INT; //function of the interruption of the IPC for communication of CPus
    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;  // SCI Tx interruption (Transmitter)
    PieVectTable.SCIA_TX_INT = &sciaTxFifoIsr;  // SCI Rx interruption (Reciever)
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;       //ADC_B interrupt. Enables column 2 of the interruptions, page 79 of the workshop material
    PieCtrlRegs.PIEIER1.bit.INTx14 = 1;      //IPC1 interruption of intercommunication between CPUs. Enables the corresponding column 14
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1 SCIA_RX
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;   // PIE Group 9, INT2 SCIA_TX
//
// Enable global Interrupts and higher priority real-time debug events:
//
    //IER |= M_INT1; //Enable the interrupt table row. corresponding to ADC_B, page 79 of the workshop material
    IER = M_INT1 | M_INT9; //Enable the interrupt table row 9, CORRESPONDING TO scia TX and RX.
    EDIS;

// Configure GPIOs
    GPIO_Configure();

// Configure Init SCI-A - fifo
    scia_fifo_init();

// Configure the ADC and power it up
//
    ConfigureADC();

// Configure DAC
    Setup_DAC();

//
// Configure the ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;         // Turn off the EPWM clock
    EDIS;

    ConfigureEPWM();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;        // Turn on the EPWM clock
    CpuSysRegs.PCLKCR0.bit.GTBCLKSYNC = 1;        // Turn on the Global clock
    EDIS;

//Transfer the Control of ADC and EPWM peripherals to core 2
    EALLOW;
    DevCfgRegs.CPUSEL0.bit.EPWM6 = 1;                   // Transfer ownership of EPWM6 to CPU2
    DevCfgRegs.CPUSEL0.bit.EPWM9 = 1;                   // Transfer ownership of EPWM9 to CPU2
    DevCfgRegs.CPUSEL0.bit.EPWM10 = 1;                   // Transfer ownership of EPWM10 to CPU2
    DevCfgRegs.CPUSEL11.bit.ADC_A = 1;                  // Transfer ownership of ADC_A to CPU2
    DevCfgRegs.CPUSEL11.bit.ADC_C = 1;                  // Transfer ownership of ADC_C to CPU2
    MemCfgRegs.GSxMSEL.bit.MSEL_GS8 = 1;                //Configura o Bloco GS8 da mem�ria RAM para o CPU2
    MemCfgRegs.GSxMSEL.bit.MSEL_GS9 = 1;                //Configura o Bloco GS9 da mem�ria RAM para o CPU2
    MemCfgRegs.GSxMSEL.bit.MSEL_GS10= 1;                //Configura o Bloco GS10 da mem�ria RAM para o CPU2
    EDIS;

    //
    // Enables CPU02 to load and wait for its loading loading through the finite loop
    ////Reminder. The CPIs that trigger interruption are 0,1,2 and 3. The others have no interruption and can be used as flags
    //
    // while (GpioDataRegs.GPADAT.bit.GPIO26 == 0);      //loop to wait for CPU02 to be loaded from DSP01 (via encoder output GPIO26)
    //IpcRegs.IPCSET.bit.IPC5 = 1;                             //Set the IPC5 bit to start CPU02 loading
    //while (IpcRegs.IPCSTS.bit.IPC4 == 0);    //loop to wait for CPU02 to load from DSP02
    //IpcRegs.IPCACK.bit.IPC4 = 1;                             //Clears the IPC4 flag

    // Enables ePwm GPIOs
    InitEPwmGpio();

    // Activate the PWM Tipzone and disables the pulses until the flag.GSC_PulsesOn command is enabled
    EALLOW;
    EPwm1Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM1
    EPwm1Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
    EPwm1Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
    EPwm2Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM2
    EPwm2Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
    EPwm2Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
    EPwm5Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM5
    EPwm5Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
    EPwm5Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
    EDIS;

    //Enables Interrupts. From that point, interruptions are called when requested
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

// Initialize results buffer
    for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
    {
       AdcResults[resultsIndex] = 0;
       AdcResults2[resultsIndex] = 0;
       AdcResults3[resultsIndex] = 0;
    }

    resultsIndex = 0;

    // Initialize signal acquisition buffers
     for(resultsIndex2 = 0; resultsIndex2 < N_data_log; resultsIndex2++)
     {
         aqui_sign1[resultsIndex2] = 0;
         aqui_sign2[resultsIndex2] = 0;
     }

     resultsIndex2 = 0;

    //Variable arrow for offset adjustment
    inv_nro_muestras = 1.0/N_amostras;

    //Enables the PLL PI controller
    pll_grid.PI_pll.enab = 1;

    //Infinite Loop
    while(1)
    {
        //Loads the flag related to the digital input responsible for checking if the Shutdown_Conv flag of set 1 has been triggered
        flag.Com_DSP2_read = GpioDataRegs.GPADAT.bit.GPIO24;    //Grid connection contactor status

        //
        // These functions are in the F2837xD_EPwm.c file
        //
        if(flag.GSC_PulsesOn == 1 && flag.precharge_ok == 1 && flag.Inv_on == 1)
        {
           //Enable the dc-link and reactive voltage controller
           pi_Vdc.enab = 1;
           pi_Q.enab   = 1;
           // Enable Inverter current controllers
           PR_Ia_fund.enab = 1;
           PR_Ia_5.enab = 1;
           PR_Ia_7.enab = 1;
           PR_Ia_11.enab = 1;
           PR_Ib_fund.enab = 1;
           PR_Ib_5.enab = 1;
           PR_Ib_7.enab = 1;
           PR_Ib_11.enab = 1;
           //Enable Ramps
           VRamp.enab = 1;
           QRamp.enab = 1;

           // Disable PWM Tipzone and enables pulses
           EALLOW;                // Enable EALLOW protected register access
           EPwm1Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM1
           EPwm1Regs.TZCTL.bit.TZA = 0x3;   // Do nothing, no action is taken on EPWMxA
           EPwm1Regs.TZCTL.bit.TZB = 0x3;   // Do nothing, no action is taken on EPWMxB
           EPwm2Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM2
           EPwm2Regs.TZCTL.bit.TZA = 0x3;   // Do nothing, no action is taken on EPWMxA
           EPwm2Regs.TZCTL.bit.TZB = 0x3;   // Do nothing, no action is taken on EPWMxB
           EPwm5Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM5
           EPwm5Regs.TZCTL.bit.TZA = 0x3;   // Do nothing, no action is taken on EPWMxA
           EPwm5Regs.TZCTL.bit.TZB = 0x3;   // Do nothing, no action is taken on EPWMxB
           EDIS;                  // Disable EALLOW protected register access

        }
        else
        {
           //Disable dc-link and reactive voltage controller
           pi_Vdc.enab = 0;
           pi_Q.enab   = 0;
           // Disable inverter current controllers
           PR_Ia_fund.enab = 0;
           PR_Ib_fund.enab = 0;
           PR_Ia_5.enab = 0;
           PR_Ia_7.enab = 0;
           PR_Ia_11.enab = 0;
           PR_Ib_5.enab = 0;
           PR_Ib_7.enab = 0;
           PR_Ib_11.enab = 0;
           //Disable ramps
           VRamp.enab = 0;
           QRamp.enab = 0;

           // Enable PWM Tipzone and disables pulses
           EALLOW;
           EPwm1Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM1
           EPwm1Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
           EPwm1Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
           EPwm2Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM2
           EPwm2Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
           EPwm2Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
           EPwm5Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM5
           EPwm5Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
           EPwm5Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
           EDIS;

        }


        // Variables that will be plotted on the Gui Composer chart
        if(flag.real_time_buff == 1)
        {
           switch(selecao_plot)
           {
              case 0: //Default
              AdcResults[resultsIndex]  = pll_grid.theta;
              AdcResults2[resultsIndex] = pll_grid.alfa;
              AdcResults3[resultsIndex] = pll_grid.beta;
              break;

              case 1:
              AdcResults[resultsIndex]  = entradas_red.Vab;
              AdcResults2[resultsIndex] = entradas_red.Vbc;
              AdcResults3[resultsIndex] = entradas_red.Vca;
              break;

              case 2:
              AdcResults[resultsIndex]  = entradas_red.Ia;
              AdcResults2[resultsIndex] = entradas_red.Ib;
              AdcResults3[resultsIndex] = entradas_red.Ic;
              break;

              case 3:
              AdcResults[resultsIndex]  = PR_Ia_fund.setpoint;
              AdcResults2[resultsIndex] = PR_Ia_fund.feedback;
              AdcResults3[resultsIndex] = 0;
              break;

              case 4:
              AdcResults[resultsIndex]  = Send.send0;
              AdcResults2[resultsIndex] = Send.send1;
              AdcResults3[resultsIndex] = Vcn;
              break;

              case 5:
              AdcResults[resultsIndex]  = pi_Q.setpoint;
              AdcResults2[resultsIndex] = pi_Q.feedback;
              AdcResults3[resultsIndex] = 0;
              break;
           }
        }

        //
        //wait while ePWM causes ADC conversions, which then cause interrupts,
        //which fill the results buffer, eventually setting the bufferFull
        //flag
        //
    }
}

// Interruption of IPC1 for communication with CPU02
interrupt void IPC1_INT(void)
{
    Recv.recv0 = IpcRegs.IPCRECVADDR;
    IpcRegs.IPCACK.bit.IPC1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// adca1_isr - Read ADC Buffer in ISR
interrupt void adcb1_isr(void)
{
    GpioDataRegs.GPBSET.bit.GPIO62 = 1;                            // GPIO para verificar a freq de amostragem

    // Fun��o de Prote��o
    TUPA_protect();

    // Fun��o de parada de funcionamento do sistema
    TUPA_StopSequence();

    // Fun��o de in�cio de funcionamento do sistema
    TUPA_StartSequence();

    //Vari�vel compartinlhada entre os n�cleos
    Send.send0 = flag.Shutdown;

    //Envia a v�riaveis para o npucleo 2
    IpcRegs.IPCSENDADDR = (Uint32) &Send.send0;
    IpcRegs.IPCSET.bit.IPC2 = 1;

    // It is determine when a EPWMxSOCA pulse will be generated (Defining the sample frequency)
    //if(EPwm1Regs.ETSEL.bit.SOCASEL == 2) EPwm1Regs.ETSEL.bit.SOCASEL = 1;
    //else EPwm1Regs.ETSEL.bit.SOCASEL = 2;

    //Piscar o LED 2 em uma determinada frequecia
    Counts.count7 ++;

    if(Counts.count7 >= 3600)
    {
        if(GpioDataRegs.GPADAT.bit.GPIO31 == 1) GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
        else GpioDataRegs.GPASET.bit.GPIO31 = 1;
        Counts.count7 = 0;
    }

    // Update the buffers with the ADCResults. Se flag.real_time_buff for igual a 1, os buffers s�o atualizados a cada per�odo de amostragem
    // Caso contr�rio, os buffers param de ser atualizados e os dados da mem�ria podem ser exportados

    if(flag.real_time_buff == 1)
    {
       resultsIndex++;
       if(resultsIndex >= RESULTS_BUFFER_SIZE)
       {
          resultsIndex = 0;
       }
    }

    /* Sa�da para o DAC
    EALLOW;
    DacaRegs.DACVALS.bit.DACVALS = (uint16_t) (1500 * (1 + __cos(376.99111*t)));
    EDIS;
    */

    //Verifica o offset das medi��es
    if(first_scan == 1)
    {
       Offset_Calculation();
    }
    else
    {
       ////////////////////////////////////////// Leitura dos sensores////////////////////////////////////////////////////////////////q
       // Tens�es de Linha
       entradas_red.Vca = 0.251556520094124*AdcdResultRegs.ADCRESULT0 - 0.251556520094124*channel_offset.CH_1;
       entradas_red.Vbc = 0.252383078685275*AdcdResultRegs.ADCRESULT1 - 0.252383078685275*channel_offset.CH_2;
       entradas_red.Vab = 0.252185841566323*AdcdResultRegs.ADCRESULT2 - 0.252185841566323*channel_offset.CH_3;

       // Estima��o das tens�es de fase
       entradas_red.Va = (entradas_red.Vab - entradas_red.Vca)*0.333333333333333;
       entradas_red.Vb = (entradas_red.Vbc - entradas_red.Vab)*0.333333333333333;
       entradas_red.Vc = (entradas_red.Vca - entradas_red.Vbc)*0.333333333333333;

       //Correntes do Inversor

       entradas_red.Ia = -(0.014552264736810*AdcbResultRegs.ADCRESULT0 - 0.014552264736810*channel_offset.CH_4);
       entradas_red.Ib = -(0.014542328746540*AdcbResultRegs.ADCRESULT1 - 0.014542328746540*channel_offset.CH_5);
       entradas_red.Ic = -(0.014663669894163*AdcbResultRegs.ADCRESULT2 - 0.014663669894163*channel_offset.CH_6);

       //Tens�o do dc-link
       Filt_freq_Vdc.Un = 0.3235*AdcdResultRegs.ADCRESULT3 - gn;
       TUPA_First_order_signals_filter(&Filt_freq_Vdc);          //filtra a tens�o Vdc com o filtro de segunda ordem
       entradas_red.Vdc = Filt_freq_Vdc.Yn;

       /////////////////////////////////Aquisi��o dos sinais//////////////////////////////////////////////////////

       if(flag.data_logo_init == 1)
         {
             Counts.count9++;
             if(Counts.count9 >= COUNT_LIM_LOG)
             {
                 resultsIndex2++;
                 Counts.count9 = 0;

                 aqui_sign1[resultsIndex2] = fil2nP.y;
                 aqui_sign2[resultsIndex2] = fil2nQ.y;
             }
         }

       ///////////////////////////////////////////////////////Inicio do Controle/////////////////////////////////////////////////////////////

       ////////////////////////////////DSOGI-PLL///////////////////////////////
       //Transformada abc para alfa-beta
       Vabc.a = entradas_red.Va;
       Vabc.b = entradas_red.Vb;
       Vabc.c = entradas_red.Vc;
       TUPA_abc2alfabeta(&Vabc,&Valfabeta);            // transformada abc para alfa beta da tens�o da rede

       //DSOGI
       SOG.Vm = Valfabeta.alfa;
       SOGB.Vm = Valfabeta.beta;
       if (Counts.count2 < 36000)
       {
           Counts.count2 += 1;
           Filt_freq_pll.Un = 60;                                             // Inicia a frequ�ncia de resson�ncia do SOG em 60Hz e, depois de um certo tempo, a freq da pll entra (adaptativo)
       }
       else
       {
            Filt_freq_pll.Un = pll_grid.freq;                                 // Freq de resson�ncia do SOG = Freq da PLL
            //Filt_freq_pll.Un = 60;                                 // Freq de resson�ncia do SOG = Freq da PLL
       }

       TUPA_First_order_signals_filter(&Filt_freq_pll);                    //filtra a frequencia da PLL
       SOG.freq_res  = Filt_freq_pll.Yn;
       SOGB.freq_res = Filt_freq_pll.Yn;
       TUPA_SOGI(&SOG);
       TUPA_SOGI(&SOGB);

       //PLL
       pll_grid.omega_init = DOISPI*60;
       pll_grid.alfa = (SOG.V_sogi - SOGB.V_sogi_q)*0.5;
       pll_grid.beta = (SOG.V_sogi_q + SOGB.V_sogi)*0.5;
       TUPA_SRFPLL(&pll_grid);

       // Elimina qualquer vest�gio da seq zero e realiza a transformada abc para alfa-beta da corrente medida do inversor
       entradas_red.Io = __divf32((entradas_red.Ia+entradas_red.Ib+entradas_red.Ic),3);
       Iabc.a = entradas_red.Ia - entradas_red.Io;
       Iabc.b = entradas_red.Ib - entradas_red.Io;
       Iabc.c = entradas_red.Ic - entradas_red.Io;

       TUPA_abc2alfabeta(&Iabc,&Ialfabeta);            // transformada abc para alfa-beta da corrente do inversor

       ////////////////////////////////Controle da tens�o do dc-link (malha externa)///////////////////////////////
       //Limita a refer�ncia de tens�o
       if(Vdc_ref>580) Vdc_ref = 580;

       //rampa da refer�ncia do Vdc
       VRamp.final = Vdc_ref;
       VRamp.in = entradas_red.Vdc;

       TUPA_Ramp(&VRamp);                                      //Rampa de refer�ncia de tens�o para o dc-link

       //controle PI
       pi_Vdc.setpoint = VRamp.atual*VRamp.atual;
       pi_Vdc.feedback = entradas_red.Vdc*entradas_red.Vdc;
       TUPA_Pifunc(&pi_Vdc);                                   // Controle PI

       ////////////////////////////////Controle do Reativo (malha externa)///////////////////////////////
       //Medi��o pot ativa Injetada
       Pm = 1.224744871391589*pll_grid.alfa*1.224744871391589*Ialfabeta.alfa + 1.224744871391589*pll_grid.beta*1.224744871391589*Ialfabeta.beta;
       //Medi��o pot reativa Injetada
       Qm = 1.224744871391589*pll_grid.beta*1.224744871391589*Ialfabeta.alfa - 1.224744871391589*pll_grid.alfa*1.224744871391589*Ialfabeta.beta;

       fil2nP.x = Pm;
       fil2nQ.x = Qm;
       TUPA_Second_order_filter(&fil2nQ);  //Filtragem do reativo medido
       TUPA_Second_order_filter(&fil2nP);  //Filtragem do ativo usado somente para aquisi��o por enquanto

       //Limita a refer�ncia de reativo
       if(Q_ref>5000) Q_ref = 5000;

       //rampa de varia��o da refer�ncia de reativo
       QRamp.final = Q_ref;
       QRamp.in    = fil2nQ.y;

       TUPA_Ramp(&QRamp);                      //Rampa de refer�ncia da pot�ncia reativa

       // Controle
       pi_Q.setpoint = QRamp.atual;
       pi_Q.feedback = fil2nQ.y;
       TUPA_Pifunc(&pi_Q);                     // Controle PI

       pi_Q.output = pi_Q.output + pi_Q.setpoint;

       ////////////////////////////////Controle de Corrente (Malha interna)///////////////////////////////
       //Sepoint do controle de corrente (Sem malha externa)
       //PR_Ia_fund.setpoint = Iref*pll_grid.costh;
       //PR_Ib_fund.setpoint = Iref*pll_grid.sinth;

       // Sepoint do controle de corrente - Teoria da pot�ncia instant�nea
       PR_Ia_fund.setpoint = __divf32((pll_grid.alfa*(-pi_Vdc.output) + pi_Q.output*pll_grid.beta),(pll_grid.alfa*pll_grid.alfa + pll_grid.beta*pll_grid.beta + 0.001));
       PR_Ib_fund.setpoint = __divf32((pll_grid.beta*(-pi_Vdc.output) - pi_Q.output*pll_grid.alfa),(pll_grid.alfa*pll_grid.alfa + pll_grid.beta*pll_grid.beta + 0.001));

       // satura��o da corrente
       if(PR_Ia_fund.setpoint>Ir)  PR_Ia_fund.setpoint =  Ir;
       if(PR_Ia_fund.setpoint<-Ir) PR_Ia_fund.setpoint = -Ir;
       if(PR_Ib_fund.setpoint>Ir)  PR_Ib_fund.setpoint =  Ir;
       if(PR_Ib_fund.setpoint<-Ir) PR_Ib_fund.setpoint = -Ir;

       PR_Ia_5.setpoint = PR_Ia_fund.setpoint;
       PR_Ia_7.setpoint = PR_Ia_fund.setpoint;
       PR_Ia_11.setpoint = PR_Ia_fund.setpoint;
       PR_Ib_5.setpoint = PR_Ib_fund.setpoint;
       PR_Ib_7.setpoint = PR_Ib_fund.setpoint;
       PR_Ib_11.setpoint = PR_Ib_fund.setpoint;

       //Feedback dos controladores
       PR_Ia_fund.feedback = Ialfabeta.alfa;
       PR_Ia_5.feedback = PR_Ia_fund.feedback;
       PR_Ia_7.feedback = PR_Ia_fund.feedback;
       PR_Ia_11.feedback = PR_Ia_fund.feedback;
       PR_Ib_fund.feedback = Ialfabeta.beta;
       PR_Ib_5.feedback = PR_Ib_fund.feedback;
       PR_Ib_7.feedback = PR_Ib_fund.feedback;
       PR_Ib_11.feedback = PR_Ib_fund.feedback;

       // Fun��es dos controladores ressonantes
       TUPA_PR(&PR_Ia_fund);
       TUPA_PR(&PR_Ia_5);
       TUPA_PR(&PR_Ia_7);
       TUPA_PR(&PR_Ia_11);
       TUPA_PR(&PR_Ib_fund);
       TUPA_PR(&PR_Ib_5);
       TUPA_PR(&PR_Ib_7);
       TUPA_PR(&PR_Ib_11);

       //Sa�da dos controladores ressonantes
       Valfabeta_pwm.alfa = PR_Ia_fund.output + PR_Ia_5.output + PR_Ia_7.output + pll_grid.alfa;
       Valfabeta_pwm.beta = PR_Ib_fund.output + PR_Ib_5.output + PR_Ib_7.output + pll_grid.beta;

       ////Comente as duas linhas anteriores e descomente as duas linhas seguintes para teste de malha aberta (N�o pode est� conectado � rede)/////
       //Valfabeta_pwm.alfa = Vd_ref * pll_grid.costh;
       //Valfabeta_pwm.beta = Vd_ref * pll_grid.sinth;

       TUPA_alfabeta2abc(&Valfabeta_pwm,&Vabc_pwm);

       TUPA_pwm(&Vabc_pwm, &sv_grid,entradas_red.Vdc,EPwm1Regs.TBPRD);

       // duty cycle
       EPwm1Regs.CMPA.bit.CMPA = sv_grid.Tc;
       EPwm2Regs.CMPA.bit.CMPA = sv_grid.Tb;
       EPwm5Regs.CMPA.bit.CMPA = sv_grid.Ta;
       //EPwm6Regs.CMPA.bit.CMPA = sv_grid.Ta;
       //EPwm9Regs.CMPA.bit.CMPA = sv_grid.Tb;
       //EPwm10Regs.CMPA.bit.CMPA = sv_grid.Tc;

       //TxBufferAqu();
    }

    GpioDataRegs.GPBCLEAR.bit.GPIO62 = 1;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //ADC Interrupt 1 Flag. Reading these flags indicates if the associated ADCINT pulse was generated since the last clear.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //Clear the flag for the interruption of the corresponding line. If you do not do this, a new interruption does not occur
}

// sciaTxFifoIsr - SCIA Transmit FIFO ISR - IAs9999Rs9999S9999F
interrupt void sciaTxFifoIsr(void)
{
    // GpioDataRegs.GPBSET.bit.GPIO62 = 1;

    Uint16 i;

    SciaRegs.SCIFFTX.bit.TXFFIL = 19;

    for(i=0; i<19; i++)
    {
        SciaRegs.SCITXBUF.all=msg_tx[i];  // Send data
    }

    //GpioDataRegs.GPBCLEAR.bit.GPIO62 = 1;

    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;       // Issue PIE ACK
}

// sciaRxFifoIsr - SCIA Receive FIFO ISR - IAs9999Rs9999F
interrupt void sciaRxFifoIsr(void)
{
    char *aux;
    aux = malloc(sizeof(char));
    int ant = 0;

    Uint16 i = 0;

    SciaRegs.SCIFFRX.bit.RXFFIL = 19;

    for(i=0; i < 19; i++)
    {
        msg_rx[i] = SciaRegs.SCIRXBUF.all;  // Read data
    }
//
//    i = 0;

//    while (1)
//    {
//        if (msg_rx[i] == 70 || i == 50) break;
//        if (ant == 65)
//        {
//            int j = 0;
//            while(msg_rx[i] != 82 || i == 50)
//            {
//                aux[j] = msg_rx[i];
//                j += 1;
//                i += 1;
//            }
//            pref = strtol(aux, NULL, 10);
//        }
//
//        if (ant == 82)
//        {
//            int j = 0;
//            while(msg_rx[i] != 70 || i == 50)
//            {
//                aux[j] = msg_rx[i];
//                j += 1;
//                i += 1;
//            }
//            qref = strtol(aux, NULL, 10);
//        }
//
//        ant = msg_rx[i];
//        i += 1;
//    }

    free(aux);

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;       // Issue PIE ack
}
//////////////////////////////////////////////////Control Functions//////////////////////////////////////
// abc-alfabeta transformation
void TUPA_abc2alfabeta(sABC *p_abc, sAlfaBeta *p_alfabeta)
{
    // Invariante em amplitude
    p_alfabeta->alfa = 0.66666667*(p_abc->a - 0.5*p_abc->b - 0.5*p_abc->c);
    p_alfabeta->beta = 0.5773502692*(p_abc->b - p_abc->c);

    // Invariante em pot�ncia
   //p_alfabeta->alfa = 0.816496580927726*(p_abc->a - 0.5*p_abc->b - 0.5*p_abc->c);
   //p_alfabeta->beta = 0.816496580927726*(0.866025403784439*p_abc->b - 0.866025403784439*p_abc->c);
}

// alfabeta-abc transformation
void TUPA_alfabeta2abc(sAlfaBeta* p_alfabeta, sABC* p_ABC)
{
    // Invariante em amplitude
    p_ABC->a = p_alfabeta->alfa;
    p_ABC->b = -0.5*p_alfabeta->alfa + 0.866025403784439*p_alfabeta->beta;
    p_ABC->c = -0.5*p_alfabeta->alfa - 0.866025403784439*p_alfabeta->beta;

    // Invariante em pot�ncia
    //p_ABC->a = 0.816496580927726*p_alfabeta->alfa;
    //p_ABC->b = 0.816496580927726*(-0.5*p_alfabeta->alfa + 0.866025403784439*p_alfabeta->beta);
    //p_ABC->c = 0.816496580927726*(-0.5*p_alfabeta->alfa - 0.866025403784439*p_alfabeta->beta);
}

// alfabeta-dq transformation
void TUPA_alfabeta2dq(sAlfaBeta *p_alfabeta, sDQ* p_DQ)
{
    p_DQ->d = (p_alfabeta->alfa* p_DQ->cosdq + p_alfabeta->beta* p_DQ->sindq);
    p_DQ->q = (-(p_alfabeta->alfa* p_DQ->sindq) + (p_alfabeta->beta* p_DQ->cosdq));
}

// SOGI
void TUPA_SOGI(sSOGI *P)
{
    P->x = 2* P->K_damp* Ts* DOISPI* P->freq_res;
    P->y = (DOISPI* P->freq_res*Ts)* (DOISPI*P->freq_res*Ts);

    P->b0 = __divf32(P->x,(P->x + P->y + 4));
    P->a1 = __divf32(2*(4 - P->y),(P->x + P->y + 4));
    P->a2 = __divf32((P->x - P->y - 4),(P->x + P->y + 4));
    P->W = 2*Ts*DOISPI*P->freq_res;

    P->V_sogi = P->b0 * P->Vm - P->b0 * P->Vm2 + P->a1 * P->V_sogi1 + P->a2 * P->V_sogi2;

    P->V_sogi_q = P->W * P->b0 * P->Vm1 + P->V_sogi_q1 * P->a1 + P->V_sogi_q2 * P->a2;

    P->Vm2 = P->Vm1;
    P->Vm1 = P->Vm;

    P->V_sogi2 = P->V_sogi1;
    P->V_sogi1 = P->V_sogi;

    P->V_sogi_q2 = P->V_sogi_q1;
    P->V_sogi_q1 = P->V_sogi_q;

}

//  PLL
void TUPA_SRFPLL(sSRFPLL *c)
{
    c->alfabeta.alfa = c->alfa;
    c->alfabeta.beta = c->beta;
    c->dq.sindq = c->sinth;
    c->dq.cosdq = c->costh;

    TUPA_alfabeta2dq(&(c->alfabeta),&(c->dq));
    c->Dpos = c->dq.d;
    c->Qpos = c->dq.q;
    c->amplitude = c->Dpos;

    c->PI_pll.setpoint = 0;

    c->PI_pll.feedback = __divf32(-c->Qpos,(__sqrt(c->Dpos*c->Dpos + c->Qpos*c->Qpos) + 0.001));

    TUPA_Pifunc(&(c->PI_pll));

    c->omega = c->PI_pll.output + c->omega_init;

    // ------------- Frequency (Hz) ----//
    c->freq = c->omega*0.159154943;

    // ------------- VCO -----------------------//
    c->theta = c->theta_ant + Ts* c->omega_ant;
    if(c->theta > DOISPI)
        c->theta -= DOISPI;
    if(c->theta < 0.0)
        c->theta += DOISPI;

    c->theta_ant = c->theta;
    c->omega_ant =  c->omega;

    c->sinth =  __sin(c->theta);
    c->costh =  __cos(c->theta);

}

// PI controller
void TUPA_Pifunc(sPI *reg)
{
    if(reg->enab==1)
    {
        reg->error = reg->setpoint - reg->feedback;
        reg->integr = reg->integr_ant + Ts_div2*(reg->error + reg->error_ant);
        reg->integr_ant = reg->integr;
        reg->error_ant = reg->error;
    }
    else
    {
        reg->error = 0;
        reg->integr = 0;
        reg->output = 0;
    }

    reg->output = reg->Kp*reg->error + reg->Ki*reg->integr;

    if (reg->output > reg->outMax)
    {
        reg->output = reg->outMax;
    }
    else if (reg->output < reg->outMin)
    {
        reg->output = reg->outMin;
    }
}

// PR controller
void TUPA_PR(sPR *r)
{
    if(r->enab==1)
    {
        r->error = r->setpoint - r->feedback;
        r->res = r->c1*r->error + r->c2*r->error_ant2 - r->c3*r->res_ant - r->c4*r->res_ant2;
        r->error_ant2 = r->error_ant;
        r->error_ant = r->error;
        r->res_ant2 = r->res_ant;
        r->res_ant = r->res;
    }
    else
    {
        r->error = 0.0;
        r->res = r->res_init;
    }

    r->output = r->Ki*r->res + r->Kp*r->error;
}

// Low pass filter
void TUPA_First_order_signals_filter(sFilter1st *x)
{

    x->Yn= (x->c0* x->Un) + (1-x->c1)*(x->Yn_1);
    x->Un_1= x->Un;
    x->Yn_1= x->Yn;

}

//Second order low pass filter
void TUPA_Second_order_filter(sFilter2nd *filt)
{
    filt->y = filt->x*filt->c0 + filt->x_ant*filt->c1 + filt->x_ant2*filt->c2 - filt->y_ant*filt->c3 - filt->y_ant2*filt->c4;
    filt->x_ant2 = filt->x_ant;
    filt->x_ant  = filt->x;
    filt->y_ant2 = filt->y_ant;
    filt->y_ant  = filt->y;
}

// SVPWM
void TUPA_pwm(sABC *p_ABC, sSvm *svp, float Vdc, Uint16 fpwm_cnt)
{
    // transform alphabeta to abc
    Van = __divf32(p_ABC->a*1.732050807568877,Vdc);
    Vbn = __divf32(p_ABC->b*1.732050807568877,Vdc);
    Vcn = __divf32(p_ABC->c*1.732050807568877,Vdc);

    // Satura��o da Tens�o
    if(Van > 1) Van = 1;
    if(Van < -1) Van = -1;
    if(Vbn > 1) Vbn = 1;
    if(Vbn < -1) Vbn = -1;
    if(Vcn > 1) Vcn = 1;
    if(Vcn < -1) Vcn = -1;

    //C�lculo da seq zero para o SVPWM
    if(Van<Vbn && Van<Vcn && Vbn>Vcn)
    {
      vmin = Van;
      vmax = Vbn;
    }
    else if(Van<Vbn && Van<Vcn && Vcn>Vbn)
    {
      vmin = Van;
      vmax = Vcn;
    }
    else if(Vbn<Van && Vbn<Vcn && Van>Vcn)
    {
      vmin = Vbn;
      vmax = Van;
    }
    else if(Vbn<Van && Vbn<Vcn && Vcn>Van)
    {
      vmin = Vbn;
      vmax = Vcn;
    }
    else if(Vcn<Van && Vcn<Vbn && Van>Vbn)
    {
      vmin = Vcn;
      vmax = Van;
    }
    else if(Vcn<Van && Vcn<Vbn && Vbn>Van)
    {
      vmin = Vcn;
      vmax = Vbn;
    }

    Vao = -0.5*(vmin+vmax)+Van;
    Vbo = -0.5*(vmin+vmax)+Vbn;
    Vco = -0.5*(vmin+vmax)+Vcn;

    svp->Ta = fpwm_cnt*0.5 + 1.154700538379252*Vao*fpwm_cnt*0.5;
    svp->Tb = fpwm_cnt*0.5 + 1.154700538379252*Vbo*fpwm_cnt*0.5;
    svp->Tc = fpwm_cnt*0.5 + 1.154700538379252*Vco*fpwm_cnt*0.5;

}

// Ramp function
void TUPA_Ramp(Ramp *rmp)
{
    if(rmp->enab)
    {
        if(rmp->final != rmp->final_ant)
        {
            rmp->flag = 0;
            rmp->flag2 = 1;
        }

        rmp->final_ant = rmp->final;

        if(rmp->flag == 0)
        {
            rmp->atual = rmp->in;
            rmp->flag = 1;
        }

        rmp->delta = rmp->final - rmp->atual;

        if(rmp->flag2 == 1)
        {
            if(rmp->delta > 0)
            {
                rmp->atual += rmp->inc;
                if(rmp->delta<=rmp->range)
                {
                    rmp->atual = rmp->final;
                    rmp->flag2 = 0;
                }
            }
            else if(rmp->delta < 0)
            {
                rmp->atual -= rmp->inc;
                if(rmp->delta>=rmp->range)
                {
                    rmp->atual = rmp->final;
                    rmp->flag2 = 0;
                }

            }
        }
    }
}

void TxBufferAqu(void)
{
    char aux[4] = {0, 0, 0, 0};

    pout = 4000.54;
    qout = 1000.54;
    soc = 25.4;

    strcpy(msg_tx, reset);

    strcat(msg_tx, "I");
    strcat(msg_tx, "A");

    if((int) pout > 0) strcat(msg_tx, "+");
    else if((int) pout < 0) strcat(msg_tx, "-");
    else strcat(msg_tx, "0");

    sprintf(aux, "%d", (int) pout);
    strcat(msg_tx, aux);

    strcat(msg_tx, "R");

    if((int) pout > 0) strcat(msg_tx, "+");
    else if((int) pout < 0) strcat(msg_tx, "-");
    else strcat(msg_tx, "0");

    sprintf(aux, "%d", (int) abs(qout));
    strcat(msg_tx, aux);

   strcat(msg_tx, "S");

    int n_decimal_points_precision = 100;
    int integerPart = (int)soc;
    int decimalPart = ((int)(soc*n_decimal_points_precision)%n_decimal_points_precision);
    sprintf(aux, "%d", integerPart);
    strcat(msg_tx, aux);
    sprintf(aux, "%d", decimalPart);
    strcat(msg_tx, aux);

    strcat(msg_tx, "F");

}
/////////////////////////////////////System Fucntions//////////////////////////////////////////
// Protection function
void TUPA_protect(void)
{
     // Prote��o de sobrecorrente no inversor
    if(fabs(entradas_red.Ia) > OVER_CURRENT_GRID_LIMIT || fabs(entradas_red.Ic) > OVER_CURRENT_GRID_LIMIT || fabs(entradas_red.Ib) > OVER_CURRENT_GRID_LIMIT)
    {
        Counts.count3++;

        if(Counts.count3 > 6)
        {
          flag.Shutdown = 1;
          fault = FAULT_OVERCURRENT;
          Counts.count3 = 0;
        }
    }

    else
    {
        Counts.count3 = 0;
    }

    // Prote��o de sobretens�o no dc-link
    if(entradas_red.Vdc > DC_OVERVOLTAGE_LIMIT)
    {
        Counts.count4++;

        if(Counts.count4 > 2)
        {
          flag.Shutdown = 1;
          fault = FAULT_DC_OVERVOLTAGE;
          Counts.count4 = 0;
        }
    }
    else
    {
        Counts.count4 = 0;
    }


    // Prote��o do Chopper
    if(entradas_red.Vdc > MAX_CHOPPER_LIMIT)
    {
        flag.Chopper_On = 1;
    }
    else if(entradas_red.Vdc < MIN_CHOPPER_LIMIT)
    {

        flag.Chopper_On = 0;
    }


    //Verifica se a flag Shutdown_Conv foi acionada na CPU02. Se sim, seta a flag Shutdown para a CPU1
    if(*Recv.recv0 == 1 && flag.AbleToStart == 1) flag.Shutdown = 1;


    //Verifica se a flag Group_com est� indicando que a prote��o foi acionada no Conjunto 1. Se sim, aciona a flag Shutdown
    if(flag.Com_DSP2_read == 1 && flag.AbleToStart == 1) flag.Shutdown = 1;


}

// System start function
void TUPA_StartSequence(void)
{

    //Verifica se a flag Shutdown est� acionado
     if(flag.Shutdown == 0)
     {
         if(*Recv.recv0 == 0 && flag.Com_DSP2_read == 0) flag.AbleToStart = 1;

         GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;           // Limpa a flag que informa para a DSP02 que a prote��o nesse conjunto foi acionada

          // Inicia o Start do sistema
          if(flag.Inv_on == 1)
          {
              if(flag.precharge_ok == 0 && flag.precharge_fail == 0)
              {
                  GpioDataRegs.GPBSET.bit.GPIO58 = 1;       //Fecha o Contator da pre-carga

                  // Inicia a contagem do tempo da precarga
                  if(Counts.count6 <= PRECHARGE_LIMIT) Counts.count6++;
                  //Verifica se o tempo m�ximo para a precarga foi atingido. Ser sim, aciona a prote��o e desliga tudo
                  if(Counts.count6 > PRECHARGE_LIMIT)
                  {
                      flag.precharge_fail = 1;
                      flag.Shutdown = 1;
                      fault = FAULT_PRECHARGE;
                  }

                  //Verifica se a tens�o minima foi alcan�ada no dc-link
                  if(entradas_red.Vdc >= DC_PRECHARGE_LIMIT)
                  {
                      if(Counts.count5 < 1000) Counts.count5++;

                      // Verifica se a condi��o anterior � atendida ap�s 1000 medi��es
                      if(Counts.count5>=1000)
                      {
                          GpioDataRegs.GPBSET.bit.GPIO59 =  1;     // Fecha o Contator principal de conex�o do inversor com a rede
                          GpioDataRegs.GPBCLEAR.bit.GPIO58 =  1;   // Abre o Contator da pr�-carga
                          flag.precharge_ok = 1;                   // Finaliza a precarga
                      }

                  }
                  else
                  {
                      Counts.count5 = 0;

                      // Abre o contator de pre carga e fecha o principal manualmente (importante quando a tens�o nos capacitores n�o atinge o minimo DC_PRECHARGE_LIMIT. OBS: Cuidado para n�o fechar este contator com uma tens�o baixa no dc-link)
                      if(flag.manual_pre_charge == 1 && entradas_red.Vdc>DC_MANUAL_PRECHARGE_LIMIT)
                      {
                          GpioDataRegs.GPBSET.bit.GPIO59 =  1;     // Fecha o Contator principal de conex�o do inversor com a rede
                          GpioDataRegs.GPBCLEAR.bit.GPIO58 =  1;   // Abre o Contator da pr�-carga
                          flag.precharge_ok = 1;                   // Finaliza a precarga
                      }

                  }
              }
           }
          //Verifica a flag do chopper de prote��o. Se o estado for alto, ativa o Chopper
          if(flag.Chopper_On == 1)
          {
              GpioDataRegs.GPBSET.bit.GPIO33 = 1;
          }

     }

     // Reseta as flags e contadores se flag.Inv_on for estado baixo
     if(flag.Inv_on == 0)
     {
         flag.precharge_ok = 0;
         flag.precharge_fail = 0;
         Counts.count6 = 0;
         Counts.count5 = 0;
     }

}

// System stop function
void TUPA_StopSequence(void)
{
    //Verifica se a flag Shutdown est� acionado ou se a Shutdown_Conv da CPU2 est� acionada (IPC6) e interrompe o chaveamento e abre os contatores
     if(flag.Shutdown == 1)
     {
         flag.AbleToStart = 0;
         flag.GSC_PulsesOn = 0;                        // Interrompe o chaveamento
         flag.Inv_on = 0;                              // Reseta o valor de flag.Inv_on
         Counts.count8 = 0;                            //Contador para a leitura do estado dos contatores
         // Abre todos os contatore
         GpioDataRegs.GPBCLEAR.bit.GPIO59 =  1;
         GpioDataRegs.GPBCLEAR.bit.GPIO58 =  1;

         GpioDataRegs.GPASET.bit.GPIO25 = 1;           // Informa para a DSP01 que a prote��o nesse conjunto foi acionada

         flag.manual_pre_charge = 0;                  // Limpa a flag de Pre carga manual

     }

     //Verifica a flag do chopper de prote��o. Se o estado for baixo, desativa o Chopper
     if(flag.Chopper_On == 0)
     {
         GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;
     }
}

// Calculation of the offset of the inverter measurements
void Offset_Calculation(void)
{
      if(Counts.count1 < N_amostras)
      {
          Counts.count1 += 1;
          sum_CH1 += AdcdResultRegs.ADCRESULT0;
          sum_CH2 += AdcdResultRegs.ADCRESULT1;
          sum_CH3 += AdcdResultRegs.ADCRESULT2;
          sum_CH4 += AdcbResultRegs.ADCRESULT0;
          sum_CH5 += AdcbResultRegs.ADCRESULT1;
          sum_CH6 += AdcbResultRegs.ADCRESULT2;


      }

      if(Counts.count1 == N_amostras)
      {
          first_scan = 0;
          Counts.count1 = 0;
          channel_offset.CH_1 = sum_CH1 * inv_nro_muestras;
          channel_offset.CH_2 = sum_CH2 * inv_nro_muestras;
          channel_offset.CH_3 = sum_CH3 * inv_nro_muestras;
          channel_offset.CH_4 = sum_CH4 * inv_nro_muestras;
          channel_offset.CH_5 = sum_CH5 * inv_nro_muestras;
          channel_offset.CH_6 = sum_CH6 * inv_nro_muestras;
      }

}

// End
