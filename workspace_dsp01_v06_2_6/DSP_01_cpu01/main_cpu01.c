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

//////////////////////////////////////////////////////// Estruturas //////////////////////////////////////////////////////////
//Sinais da rede e do dc-link
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

#define SOGI_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.01,60}
sSOGI SOG = SOGI_DEFAULTS;
sSOGI SOGB = SOGI_DEFAULTS;

//Controlador PI
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

//Filtro de primeira ordem
typedef struct {
    float Un;
    float Un_1;
    float Yn_1;
    float Yn;
    float c0;
    float c1;
    } sFilter1st;
#define FILTER_DEFAULTS {0,0,0,0,0.00034906585039886593,0.00034906585039886593} //1Hz

sFilter1st Filt_freq_pll = FILTER_DEFAULTS;

//M�dia m�vel
typedef struct{
float array[N];
float x;
float y;
float y_ant;
int j;
int Ns;
} sMAV;

#define  MAV_default {{0},0,0,0,0,N}
sMAV MAVQ = MAV_default;
sMAV MAVV = MAV_default;

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

#define  FILTER2ND_DEFAULTS {0,0,0,0,0,0,0.00001212470404899192,0.00002424940809798384,0.00001212470404899192,-1.990128522794091,0.990177021610287}   //20Hz
sFilter2nd fil2nVdc = FILTER2ND_DEFAULTS;

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

}sFlags;

#define FLAGS_DEFAULTS {0,0,0,0,0,0,1,0,0}
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

#define PR_I_FUND_DEFAULTS {0.00002777574703951879,-0.00002777574703951879,-1.999561366949691,1.000000000000000,0,0,0,0,0,0,0,0,\
    PR_I_GRID_KP, PR_I_GRID_KI, 0, 0, 0, 0}

#define PR_I_5_DEFAULTS {0.00002772703603807777,-0.00002772703603807777,-1.989043790736547,1.000000000000000,0,0,0,0,0,0,0,0,\
    0, PR_I_GRID_KI, 0, 0, 0, 0}

#define PR_I_7_DEFAULTS {0.00002767837630659803,-0.00002767837630659803,-1.978544665925977,1.000000000000000,0,0,0,0,0,0,0,0,\
    0, PR_I_GRID_KI, 0, 0, 0, 0}

sPR PR_Ia_fund = PR_I_FUND_DEFAULTS;
sPR PR_Ib_fund = PR_I_FUND_DEFAULTS;
sPR PR_Ia_5 = PR_I_5_DEFAULTS;
sPR PR_Ib_5 = PR_I_5_DEFAULTS;
sPR PR_Ia_7 = PR_I_7_DEFAULTS;
sPR PR_Ib_7 = PR_I_7_DEFAULTS;

//Rampa
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

//Contadores
typedef struct{
    Uint32 count1;
    Uint32 count2;
    Uint32 count3;
    Uint32 count4;
    Uint32 count5;
    Uint32 count6;
    Uint32 count7;
    Uint32 count8;
}counts;

#define COUNTS_DEFAULTS {0,0,0,0,0,0,0,0}
counts Counts = COUNTS_DEFAULTS;

//V�riaveis para enviar dados do CPU1 para o CPU2
typedef struct{
    unsigned int send0;
    float send1;
}SsendCPU1toCPU2;

#define SEND_DEFAULTS {0,0}
SsendCPU1toCPU2 Send = SEND_DEFAULTS;

//V�riaveis para receber dados do CPU1 para o CPU2
typedef struct{
    unsigned int *recv0;
    float *recv1;
}SrecvCPU2toCPU1;

#define RECV_DEFAULTS {0,0}
SrecvCPU2toCPU1 Recv = RECV_DEFAULTS;

//Status dos Contatores
typedef struct{
    Uint16 Q3s;
    Uint32 Time_cont_status;             //tempo*(Freq de amostragem)
}Scontatores;

#define CONTATORES_DEFAULTS {0,3.0*Nsample*PWM_FREQ}
Scontatores Contat_status = CONTATORES_DEFAULTS;

typedef struct{
    float sci_out;
    Uint16 asci;
    bool decimal;
}Ssci;

#define SCI_DEFAULTS {0,0,false}
Ssci scia_p = SCI_DEFAULTS;
Ssci scia_q = SCI_DEFAULTS;
Ssci scia_soc = SCI_DEFAULTS;
Ssci scia_check1 = SCI_DEFAULTS;
Ssci scia_check2 = SCI_DEFAULTS;
Ssci scia_check3 = SCI_DEFAULTS;
Ssci scib_p = SCI_DEFAULTS;
Ssci scib_q = SCI_DEFAULTS;
Ssci scib_soc = SCI_DEFAULTS;
Ssci scib_check1 = SCI_DEFAULTS;
Ssci scib_check2 = SCI_DEFAULTS;
Ssci scib_check3 = SCI_DEFAULTS;

typedef struct{
    float pref;
    float qref;
    float socref;
    float check1;
    float check2;
    float check3;
    Uint16 soma_tx;
    Uint16 len_msg;
    int16 count;
    char msg_tx[len_sci];
    char msg_rx[len_sci];
    Uint16 sdata[8];    // Send data
    Uint16 rdata[8];    // Received data
}Ssci_mesg;

#define SCI_MSG_DEFAULTS {0,0,0,0,0,0,0,0,0,{0},{0},{0},{0}}
Ssci_mesg sci_msgA = SCI_MSG_DEFAULTS;
Ssci_mesg sci_msgB = SCI_MSG_DEFAULTS;

//////////////////////////////////////////////Function//////////////////////////////////////////////
void TxBufferAqu(Ssci_mesg *);
float RxBufferAqu(Ssci *, Ssci_mesg *);
int sumAscii(char *string, int len);
////////////////////////////////////////////// Global Variables ////////////////////////////////////

//Variavel para ajuste do offset
float inv_nro_muestras = 0;

//V�riavel de teste
float  gn = 1;
//#pragma DATA_SECTION(gn,"SHARERAMGS1");         // Coloca a vari�vei que ser� compartilhada com o CPU02 no seguinte segmento de mem�ria

//Buffers para plot
float32 AdcResults[RESULTS_BUFFER_SIZE];
float32 AdcResults2[RESULTS_BUFFER_SIZE];
float32 AdcResults3[RESULTS_BUFFER_SIZE];

//Variaveis de comunica��o entre o CPUs
int send = 0;

// V�ri�veis de Controle
float Ts = TSAMPLE;
float Van = 0, Vbn = 0, Vcn = 0 , vmin = 0, vmax = 0, Vao = 0, Vbo = 0, Vco = 0;

float  Iref = 0;                               //Refer�ncia de corrente (sem malha externa)
float  Vdc_ref = 80;                      //Tens�o de refer�ncia da tens�o do dc-link;
float  Q_ref   = 0;                               //Refer�ncia de pot�ncia reativa;
float  Qm      = 0;                               //pot�ncia reativa medida

int selecao_plot = 0;
Uint16 fault = FAULT_OK;
Uint32 resultsIndex = 0;
//Variaveis para ajuste do offset
Uint32 first_scan = 1;
Uint32 sum_CH1 = 0; Uint32 sum_CH2 = 0; Uint32 sum_CH3 = 0; Uint32 sum_CH4 = 0; Uint32 sum_CH5 = 0; Uint32 sum_CH6 = 0;
Uint32 N_amostras = 60000;

// SCI parameters
float pout = 4000.54;
float qout = 1000.54;
float soc = 25.4;
char reset[len_sci] = {0, 0, 0, 0, 0, 0, 0, 0};
Uint16 reset_sci = 0;

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

// init the pins for the SCI ports.
//  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
//  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
// These functions are found in the F2837xD_Gpio.c file.
//
   GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
   GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
   GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

   GPIO_SetupPinMux(23, GPIO_MUX_CPU1, 3);
   GPIO_SetupPinOptions(23, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 3);
   GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_ASYNC);
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
    PieVectTable.IPC1_INT = &IPC1_INT; //function of the interruption of the IPC for communication of CPus
    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;  // SCI Tx interruption
    PieVectTable.SCIA_TX_INT = &sciaTxFifoIsr;  // SCI Rx interruption
    PieVectTable.SCIB_RX_INT = &scibRxFifoIsr;  // SCI Tx interruption
    PieVectTable.SCIB_TX_INT = &scibTxFifoIsr;  // SCI Rx interruption
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;       //ADC_B interrupt. Enables column 2 of the interruptions, page 79 of the workshop material
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1 SCIA_RX
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;   // PIE Group 9, INT2 SCIA_TX
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;   // PIE Group 9, INT1 SCIB_RX
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;   // PIE Group 9, INT2 SCIB_TX
// Enable global Interrupts and higher priority real-time debug events:
//
    IER = M_INT1 | M_INT9; //Habilita a linha da tabela de interrup��o. correspondente ao ADC_B, pg 79 do material do workshop

    EDIS;

// Configure GPIOs
    GPIO_Configure();

// Configure Init SCI-A - fifo
    scia_fifo_init();

// Configure Init SCI-B - fifo
    scib_fifo_init();
//
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

//Transfere o Controle dos perif�ricos ADC e EPWM para o n�cleo 2
    EALLOW;
    DevCfgRegs.CPUSEL0.bit.EPWM6 = 1;                   // Transfer ownership of EPWM6 to CPU2
    DevCfgRegs.CPUSEL0.bit.EPWM9 = 1;                   // Transfer ownership of EPWM6 to CPU2
    DevCfgRegs.CPUSEL0.bit.EPWM10 = 1;                   // Transfer ownership of EPWM6 to CPU2
    DevCfgRegs.CPUSEL11.bit.ADC_A = 1;                  // Transfer ownership of ADC_A to CPU2
    DevCfgRegs.CPUSEL11.bit.ADC_B = 1;                  // Transfer ownership of ADC_B to CPU2
    DevCfgRegs.CPUSEL11.bit.ADC_C = 1;                  // Transfer ownership of ADC_C to CPU2
    MemCfgRegs.GSxMSEL.bit.MSEL_GS8 = 1;                //Configura o Bloco GS8 da mem�ria RAM para o CPU2
    MemCfgRegs.GSxMSEL.bit.MSEL_GS9 = 1;                //Configura o Bloco GS9 da mem�ria RAM para o CPU2
    MemCfgRegs.GSxMSEL.bit.MSEL_GS10= 1;                //Configura o Bloco GS10 da mem�ria RAM para o CPU2
    EDIS;

    //
    //Habilita o CPU02 Carregar e Aguarda o seu carregamento carregamento atrav�s do loop finito
    ////Lembrete. Os IPCs que disparam interrup��o s�o o 0,1,2 e 3. Os outros n�o tem interrup��o e podem ser usados como flags
    //
    IpcRegs.IPCSET.bit.IPC5 = 1;                             //Seta o bit IPC5 para iniciar o carregamento do CPU02 carregar
    while (IpcRegs.IPCSTS.bit.IPC4 == 0);                    //Loop finito para aguardar o carregamento do CPU02
    IpcRegs.IPCACK.bit.IPC4 = 1;                             //Limpa a flag do IPC4

    // Habilita as GPIOs como ePwm
    InitEPwmGpio();

    // Ativa o Tipzone dos PWM e desabilita os pulsos at� o comando da flag.GSC_PulsesOn for habilitado
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

    //Habilita as Interrup��es. A partir desse ponto as interrup��es s�o chamadas quando requisitadas
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

    //Seta a variavel para ajuste do offset
    inv_nro_muestras = 1.0/N_amostras;

    //Habilita o controlador PI da PLL
    pll_grid.PI_pll.enab = 1;

    //Loop infinito
    while(1)
    {
         if ((reset_sci == 1) || (SciaRegs.SCIRXST.all != 0x0000))
         {
           SciaRegs.SCIFFTX.bit.TXFIFORESET = 0;
           SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;
           SciaRegs.SCIFFTX.bit.SCIRST = 0;
         }
         else
         {
           SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
           SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
           SciaRegs.SCIFFTX.bit.SCIRST = 1;
         }

         if ((reset_sci == 1) || (ScibRegs.SCIRXST.all != 0x0000))
         {
           ScibRegs.SCIFFTX.bit.TXFIFORESET = 0;
           ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;
           ScibRegs.SCIFFTX.bit.SCIRST = 0;
         }
         else
         {
           ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;
           ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
           ScibRegs.SCIFFTX.bit.SCIRST = 1;
         }
    }
}

//Interrup��o do IPC1 para comunica��o com a CPU02
interrupt void IPC1_INT(void)
{
    Recv.recv0 = IpcRegs.IPCRECVADDR;
    IpcRegs.IPCACK.bit.IPC1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// sciaTxFifoIsr - SCIA Transmit FIFO ISR - ex: IA+9999F
interrupt void sciaTxFifoIsr(void)
{
    // GpioDataRegs.GPBSET.bit.GPIO62 = 1;
    Uint16 i;

    for(i=0; i < len_sci; i++)
    {
       SciaRegs.SCITXBUF.all=sci_msgA.sdata[i];  // Send data
    }

    sci_msgA.count += 1;
    TxBufferAqu(&sci_msgA);

    for (i=0; i<len_sci; i++)
    {
        sci_msgA.sdata[i] = sci_msgA.msg_tx[i];
    }

    //GpioDataRegs.GPBCLEAR.bit.GPIO62 = 1;

    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;       // Issue PIE ACK
}

// scibTxFifoIsr - SCIB Transmit FIFO ISR - ex: IA+9999F
interrupt void scibTxFifoIsr(void)
{
    // GpioDataRegs.GPBSET.bit.GPIO62 = 1;
    Uint16 i;

    for(i=0; i < len_sci; i++)
    {
       ScibRegs.SCITXBUF.all=sci_msgB.sdata[i];  // Send data
    }

    sci_msgB.count += 1;
    TxBufferAqu(&sci_msgB);

    for (i=0; i<len_sci; i++)
    {
        sci_msgB.sdata[i] = sci_msgB.msg_tx[i];
    }

    //GpioDataRegs.GPBCLEAR.bit.GPIO62 = 1;

    ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;       // Issue PIE ACK
}

// sciaRxFifoIsr - SCIA Receive FIFO ISR
interrupt void sciaRxFifoIsr(void)
{
    Uint16 i;
    Uint16 soma_rx = 0;
    float pref_temp = 0;
    float qref_temp = 0;
    float soc_temp = 0;

    for(i=0;i<8;i++)
    {
        sci_msgA.rdata[i]=SciaRegs.SCIRXBUF.all;  // Read data
    }

    for (i=0; i<len_sci; i++)
    {
        sci_msgA.msg_rx[i] = sci_msgA.rdata[i];
    }

    scia_p.asci = 65;
    scia_p.decimal = false;
    pref_temp = RxBufferAqu(&scia_p, &sci_msgA);

    scia_check1.asci = 67;               // C
    scia_check1.decimal = false;
    sci_msgA.check1 = RxBufferAqu(&scia_check1, &sci_msgA);

    scia_q.asci = 82;
    scia_q.decimal = false;
    qref_temp = RxBufferAqu(&scia_q, &sci_msgA);

    scia_check2.asci = 68;             // D
    scia_check2.decimal = false;
    sci_msgA.check2 = RxBufferAqu(&scia_check2, &sci_msgA);

    scia_soc.asci = 83;
    scia_soc.decimal = true;
    soc_temp = RxBufferAqu(&scia_soc, &sci_msgA);

    scia_check3.asci = 79;             // O
    scia_check3.decimal = false;
    sci_msgA.check3 = RxBufferAqu(&scia_check3, &sci_msgA);

    soma_rx = sumAscii(sci_msgA.msg_rx, (int) len_sci);

    if ((int) sci_msgA.check1 == soma_rx)   sci_msgA.pref = pref_temp;
    if ((int) sci_msgA.check2 == soma_rx)   sci_msgA.qref = qref_temp;
    if ((int) sci_msgA.check3 == soma_rx) sci_msgA.socref = soc_temp;

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;       // Issue PIE ack
}

// scibRxFifoIsr - SCIB Receive FIFO ISR
interrupt void scibRxFifoIsr(void)
{
    Uint16 i;
    Uint16 soma_rx = 0;
    float pref_temp = 0;
    float qref_temp = 0;
    float soc_temp = 0;

    for(i=0;i<8;i++)
    {
        sci_msgB.rdata[i]=ScibRegs.SCIRXBUF.all;  // Read data
    }

    for (i=0; i<len_sci; i++)
    {
        sci_msgB.msg_rx[i] = sci_msgB.rdata[i];
    }

    scib_p.asci = 65;
    scib_p.decimal = false;
    pref_temp = RxBufferAqu(&scib_p, &sci_msgB);

    scib_check1.asci = 67;               // C
    scib_check1.decimal = false;
    sci_msgB.check1 = RxBufferAqu(&scib_check1, &sci_msgB);

    scib_q.asci = 82;
    scib_q.decimal = false;
    qref_temp = RxBufferAqu(&scib_q, &sci_msgB);

    scib_check2.asci = 68;             // D
    scib_check2.decimal = false;
    sci_msgB.check2 = RxBufferAqu(&scib_check2, &sci_msgB);

    scib_soc.asci = 83;
    scib_soc.decimal = true;
    soc_temp = RxBufferAqu(&scib_soc, &sci_msgB);

    scib_check3.asci = 79;             // O
    scib_check3.decimal = false;
    sci_msgB.check3 = RxBufferAqu(&scib_check3, &sci_msgB);

    soma_rx = sumAscii(sci_msgB.msg_rx, (int) len_sci);

    if ((int) sci_msgB.check1 == soma_rx)   sci_msgB.pref = pref_temp;
    if ((int) sci_msgB.check2 == soma_rx)   sci_msgB.qref = qref_temp;
    if ((int) sci_msgB.check3 == soma_rx) sci_msgB.socref = soc_temp;

    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;       // Issue PIE ack
}

// Tx funtion
void TxBufferAqu(Ssci_mesg *sci)
{
    char aux[4] = {0, 0, 0, 0};
    char aux2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    Uint16 i = 0;

    if (sci->count == 0)
    {
        strcpy(sci->msg_tx, reset);

        strcat(sci->msg_tx, "I");

        strcat(sci->msg_tx, "A");

        if((int) pout >= 0) strcat(sci->msg_tx, "+");
        else if((int) pout < 0) strcat(sci->msg_tx, "-");
        else strcat(sci->msg_tx, "0");

        sprintf(aux, "%d", (int) abs(pout));
        strcat(sci->msg_tx, aux);

        strcat(sci->msg_tx, "F");

        sci->len_msg = strlen(sci->msg_tx);

        if(sci->len_msg < len_sci)
        {
            for(i=0; i<(len_sci-sci->len_msg); i++)
                strcat(sci->msg_tx, "-");
        }

        sci->soma_tx = sumAscii(sci->msg_tx, (int) len_sci);
    }

    if (sci->count == 10)
    {
        strcpy(sci->msg_tx, reset);

        strcat(sci->msg_tx, "I");

        strcat(sci->msg_tx, "C");

        sprintf(aux2, "%d", (int) abs(sci->soma_tx));
        strcat(sci->msg_tx, aux2);

        strcat(sci->msg_tx, "F");

        sci->len_msg = strlen(sci->msg_tx);

        if(sci->len_msg < len_sci)
        {
            for(i=0; i<(len_sci-sci->len_msg); i++)
                strcat(sci->msg_tx, "-");
        }
    }

    if (sci->count == 20)
    {
        strcpy(sci->msg_tx, reset);

        strcat(sci->msg_tx, "I");

        strcat(sci->msg_tx, "R");

        if((int) qout >= 0) strcat(sci->msg_tx, "+");
        else if((int) qout < 0) strcat(sci->msg_tx, "-");
        else strcat(sci->msg_tx, "0");

        sprintf(aux, "%d", (int) abs(qout));
        strcat(sci->msg_tx, aux);

        strcat(sci->msg_tx, "F");

        sci->len_msg = strlen(sci->msg_tx);

        if(sci->len_msg < len_sci)
        {
            for(i=0; i<(len_sci-sci->len_msg); i++)
                strcat(sci->msg_tx, "-");
        }

        sci->soma_tx = sumAscii(sci->msg_tx, (int) len_sci);
    }

    if (sci->count == 30)
    {
        strcpy(sci->msg_tx, reset);

        strcat(sci->msg_tx, "I");

        strcat(sci->msg_tx, "D");

        sprintf(aux2, "%d", (int) abs(sci->soma_tx));
        strcat(sci->msg_tx, aux2);

        strcat(sci->msg_tx, "F");

        sci->len_msg = strlen(sci->msg_tx);

        if(sci->len_msg < len_sci)
        {
            for(i=0; i<(len_sci-sci->len_msg); i++)
                strcat(sci->msg_tx, "-");
        }
    }

    if (sci->count == 40)
    {
        strcpy(sci->msg_tx, reset);

        strcat(sci->msg_tx, "I");

        strcat(sci->msg_tx, "S");

        int n_decimal_points_precision = 100;
        int integerPart = (int)soc;
        int decimalPart = ((int)(soc*n_decimal_points_precision)%n_decimal_points_precision);

        sprintf(aux, "%d", integerPart);
        strcat(sci->msg_tx, aux);

        strcat(sci->msg_tx, ".");

        sprintf(aux, "%d", decimalPart);
        strcat(sci->msg_tx, aux);

        strcat(sci->msg_tx, "F");

        sci->len_msg = strlen(sci->msg_tx);

        if(sci->len_msg < len_sci)
        {
            for(i=0; i<(len_sci-sci->len_msg); i++)
                strcat(sci->msg_tx, "-");
        }

        sci->soma_tx = sumAscii(sci->msg_tx, (int) len_sci);
    }

    if (sci->count == 50)
    {
        strcpy(sci->msg_tx, reset);

        strcat(sci->msg_tx, "I");

        strcat(sci->msg_tx, "O");

        sprintf(aux2, "%d", (int) abs(sci->soma_tx));
        strcat(sci->msg_tx, aux2);

        strcat(sci->msg_tx, "F");

        sci->len_msg = strlen(sci->msg_tx);

        if(sci->len_msg < len_sci)
        {
            for(i=0; i<(len_sci-sci->len_msg); i++)
                strcat(sci->msg_tx, "-");
        }

        sci->count = -10;
    }
}

// Rx funtion
float RxBufferAqu(Ssci *sci, Ssci_mesg *scimsg)
{
    Uint16 rstart = 0;
    Uint16 aq1 = 0;
    char aux[5] = {0, 0, 0, 0, 0};
    Uint16 k = 0;
    Uint16 i = 0;
    Uint16 j = 0;

    while (1)
    {
        if (scimsg->msg_rx[i] == 73 && rstart == 0)
        {
            rstart = 1;
        }
        if(rstart == 1)
        {
            if(scimsg->msg_rx[i] == 70) break;

            if(aq1==1)
            {
                aux[j] = scimsg->msg_rx[i];
                j += 1;
            }
            if(scimsg->msg_rx[i] == sci->asci)
            {
                aq1 = 1;
                j = 0;
            }
        }

        i += 1;
        k += 1;

        if(i>=len_sci) i = 0;

        if(k>=50) break;
    }

    if (aq1 == 1 && sci->decimal == false) sci->sci_out = strtol(aux, NULL, 10);
    if (aq1 == 1 && sci->decimal == true) sci->sci_out = strtof(aux, NULL);

    return sci->sci_out;
}

int sumAscii(char *string, int len)
{
    int sum = 0;
    int j = 0;

    for (j = 0; j < len; j++)
    {
        sum = sum + string[j];
    }
    return sum;
}

// Fim do c�digo
