//###########################################################################
//
// TITLE:  BESS

//
// Included Files
//
#include "F28x_Project.h"
#include "math.h"

#include "Peripheral_Interruption_Setup_cpu02.h"
#include "Tupa_parameters_cpu02.h"

//////////////////////////////////////////////////////// Estruturas //////////////////////////////////////////////////////////
//Sinais dc
typedef struct{
    float I1;
    float I2;
    float I3;
    float Vb1;
    float Vb2;
    float Vb3;
    float Vbt;
    float Vbt_filt;
    float Vdc;
}sSignal_dc;

#define SIGNAL_DC_DEFAULTS {0,0,0,0,0,0,0,0,0}
sSignal_dc entradas_dc = SIGNAL_DC_DEFAULTS;

//Controlador PI
typedef struct {
    int enab;
    float feedback;
    float setpoint;
    float error;
    float error_ant;
    float errorpi;
    float errorpi_ant;
    float integr;
    float integr_ant;
    float dif;
    float Kp;
    float Ki;
    float output;
    float output_sat;
    float outMin;
    float outMax;
    float feedforward;
} sPI;

#define PI_IDIS_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PI_DIS_KP, PI_DIS_KI, 0, 0, PI_DIS_OUTMIN , PI_DIS_OUTMAX, 0}
#define PI_ICH_DEFAULTS  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PI_ICH_KP, PI_ICH_KI, 0, 0, PI_ICH_OUTMIN , PI_ICH_OUTMAX, 0}
#define PI_VCH_DEFAULTS  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PI_VCH_KP, PI_VCH_KI, 0, 0, 0, 0, 0}
sPI pi_I1_dis = PI_IDIS_DEFAULTS;
sPI pi_I2_dis = PI_IDIS_DEFAULTS;
sPI pi_I3_dis = PI_IDIS_DEFAULTS;
sPI pi_I1_ch  = PI_ICH_DEFAULTS;
sPI pi_I2_ch  = PI_ICH_DEFAULTS;
sPI pi_I3_ch  = PI_ICH_DEFAULTS;
sPI piv_ch    = PI_VCH_DEFAULTS;

//PWM
typedef struct{
    float din;
    Uint16 Ta;
} sPWM;

#define PWM_DEFAULTS {0,0}
sPWM pwm_dc = PWM_DEFAULTS;
sPWM pwm_dc2 = PWM_DEFAULTS;
sPWM pwm_dc3 = PWM_DEFAULTS;

//Flags
typedef struct{
    unsigned int BSC_PulsesOn;
    unsigned int Conv_on;
    unsigned int Shutdown_Conv;
    unsigned int Stand_by;
    unsigned int AbleToStart;
    unsigned int Bat_Charge;
    unsigned int Bat_Discharge;
    unsigned int Bat_Mode;
    unsigned int BVCM;
    unsigned int data_logo_init;
    unsigned int real_time_buff;
}sFlags;

#define FLAGS_DEFAULTS {0,0,0,0,0,0,0,1,0,0,1}
sFlags flag = FLAGS_DEFAULTS;

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

#define IRamp_default {0,0,0,0,0,0,0,0,0.01,0.008}
#define VRamp_default {0,0,0,0,0,0,0,0,0.01,0.01}
Ramp I1_Ramp = IRamp_default;
Ramp I2_Ramp = IRamp_default;
Ramp I3_Ramp = IRamp_default;
Ramp VRamp   = VRamp_default;

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

#define  FILTER2ND_50Hz_DEFAULTS {0,0,0,0,0,0,0.00029719248951703286,0.00059438497903406572,0.00029719248951703286,-1.95065639854559402799,0.95184516850366218677}
sFilter2nd fil2nVbat  = FILTER2ND_50Hz_DEFAULTS;
sFilter2nd fil2nVdc   = FILTER2ND_50Hz_DEFAULTS;

//Firs order LPF
typedef struct {
    float Un;
    float Un_1;
    float Yn_1;
    float Yn;
    float c0;
    float c1;
    } sFilter1st;
#define FILTER_DEFAULTS_0_1_HZ {0,0,0,0,0.00006981317007977319,0.00006981317007977319}
#define FILTER_DEFAULTS_2_HZ {0,0,0,0,0.00139626340159546372,0.00139626340159546372}

sFilter1st fil1nVbat = FILTER_DEFAULTS_0_1_HZ;
sFilter1st fil1nVbat2 = FILTER_DEFAULTS_0_1_HZ;
sFilter1st fil1nVbat3 = FILTER_DEFAULTS_0_1_HZ;
sFilter1st Filt_current = FILTER_DEFAULTS_2_HZ;

typedef struct{
float qn;
float tsc;
int enable;
float qinit;
float q;
float VbatIn_soc_est;
float Vin;
float soc_ocv_ant;
float soc_init;
float Iin;
float inte;
float inte_ant;
float x;
float x_ant;
float soc_out;
int flag_init;
sFilter1st Filt_current;
float soc_ocv[51];
float vbat_ocv[51];
} sSoc;

#define SOC_default {20.8,3600,0,0,0,0,0,0,0,0,0,0,0,0,0,0,FILTER_DEFAULTS_2_HZ,{0,   2,   4,   6,   8,  10,  12,  14,  16,  18,  20,  22,  24,  26,   28,  30,  32,  34,  36,  38,  40,  42,  44,  46,  48,  50,  52,  54, 56,  58,  60, 62, 64,  66,  68,  70,  72,  74,  76,  78,  80,  82, 84,  86,  88,  90,  92,  94,  96,  98, 100}, {177.34693563, 181.01869441, 183.71131752, 185.77038225, 187.39595967, 188.71190329, 189.79898716, 190.7121376, 191.4900065,192.16058314, 192.74463375,193.25789036, 193.71248906, 194.11794197, 194.48180995, 194.81017862, 195.10800137, 195.37935099, 195.62760702, 195.85559725, 196.0657059,  196.25995729, 196.4400813, 196.60756504, 196.76369394, 196.90958489, 197.04621291, 197.17443306, 197.29499828, 197.4085742, 197.51575148,  197.61705604, 197.71295768, 197.80387742, 197.89019363, 197.97224732, 198.0503466,  198.12477063, 198.19577286, 198.26358398, 198.32841439, 198.3904564,  198.44988611, 198.50686512, 198.56154194, 198.61405334, 198.66452546, 198.71307484, 198.75980928, 198.80482872, 210.97278976}}
sSoc soc_est = SOC_default;

//Canais para retirar offset
typedef struct{
    int CH_1;
    int CH_2;
    int CH_3;
}sChannel_adc;

#define CHANNEL_DEFAULTS {0,0,0}
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
    Uint32 count9;
    Uint32 count10;
    Uint32 count11;
}counts;

#define COUNTS_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0}
counts Counts = COUNTS_DEFAULTS;

//V�riaveis para enviar dados do CPU2 para o CPU1
typedef struct{
    unsigned int send0;
    float send1;
}SsendCPU2toCPU1;

#define SEND_DEFAULTS {0,0}
SsendCPU2toCPU1 Send = SEND_DEFAULTS;

//V�riaveis para receber dados do CPU1 para o CPU2
typedef struct{
    unsigned int *recv0;
    float *recv1;
}SrecvCPU1toCPU2;

#define RECV_DEFAULTS {0,0}
SrecvCPU1toCPU2 Recv = RECV_DEFAULTS;

///////////////////////////////////////////// Fun��es ////////////////////////////////////////
// Control
void TUPA_Pifunc(sPI *);
void TUPA_protect(void);
void Stand_by_mode(void);
void TUPA_StartSequence(void);
void TUPA_pwm(sPWM *, Uint16);
void TUPA_StopSequence(void);
void Offset_Calculation(void);
void TUPA_Ramp(Ramp *);
void TUPA_Second_order_filter(sFilter2nd *);
void TUPA_First_order_signals_filter(sFilter1st *);
void soc_estimation(sSoc *);

////////////////////////////////////////////// Global Variables ////////////////////////////////////
//Variavel para ajuste do offset
float inv_nro_muestras = 0;

//Buffers para plot
float32 AdcResults[RESULTS_BUFFER_SIZE];
float32 AdcResults2[RESULTS_BUFFER_SIZE];
float32 AdcResults3[RESULTS_BUFFER_SIZE];

//Buffers para armazenamento de dados
float Vbat_vec[N_data_log] ;
float Ibat_vec[N_data_log];

//Variaveis de comunica��o entre o CPUs
int send = 0;

//V�riaveis de teste
float  gn  = 937.5;

// V�ri�veis de Controle
float Ts = TSAMPLE;
float Van = 0, Vbn = 0, Vcn = 0 , vmin = 0, vmax = 0, Vao = 0, Vbo = 0, Vco = 0;

float I_dis_ref   =  5;                         //Refer�ncia da corrente de descarga (modo Boost)
float I_ch_ref    =  5;                         //Refer�ncia da corrente de carga (modo Buck)
float Vboost      =  14.4;                      //Tens�o de Boost
float Vfloat      =  13.6;                      //Tens�o de Float
float Vref        =  0;                         //Refer�ncia da tens�o de carga

int selecao_plot = 0;
Uint16 fault = FAULT_OK;
Uint32 resultsIndex = 0;
Uint32 resultsIndex2 = 0;
//Variaveis para ajuste do offset
Uint16 first_scan = 1; Uint16 first_scan2 = 1; Uint16 first_scan3 = 1;
Uint32 sum_CH1 = 0; Uint32 sum_CH2 = 0; Uint32 sum_CH3 = 0;
Uint32 N_amostras = 60000;

//Main
void main(void)

{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

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

    PieVectTable.ADCA1_INT = &adca1_isr;     //function for ADCA interrupt 1
    PieVectTable.ADCA2_INT = &adca2_isr;     //function for ADCA interrupt 2
    PieVectTable.ADCA3_INT = &adca3_isr;     //function for ADCA interrupt 2
    PieVectTable.IPC2_INT = &IPC2_INT;       //fun��o da interrup��o do IPC para comunica��o das CPus
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;       //Interrup��o ADC_A. Habilita a coluna 1 das interrup��es, pg 79 do material do workshop
    PieCtrlRegs.PIEIER10.bit.INTx2 = 1;      //Interrup��o ADC_A2. Habilita a coluna 1 das interrup��es, pg 79 do material do workshop
    PieCtrlRegs.PIEIER10.bit.INTx3 = 1;      //Interrup��o ADC_A3. Habilita a coluna 1 das interrup��es, pg 79 do material do workshop
    PieCtrlRegs.PIEIER1.bit.INTx15 = 1;      //interrup��o IPC2 de inter comunica��o entre os CPUS. Habilita a coluna 15 correspondente
//
// Enable global Interrupts and higher priority real-time debug events:
//
    IER |= (M_INT10+M_INT1); //Habilita a linha da tabela de interrup��o. correspondente ao ADC_B, pg 79 do material do workshop

    EDIS;

//Aguarda o CPU1 carregar as configura��es da mem�ria e dos perif�ricos. Ap�s isso, o CPU02 est� habilitado para continuar o carregamento dos perif�ricos associados a ele
//Lembrete. Os IPCs que disparam interrup��o s�o o 0,1,2 e 3. Os outros n�o tem interrup��o e podem ser usados como flags

    while(IpcRegs.IPCSTS.bit.IPC5 == 0);          //Loop finito para aguardar o carregamento do CPU02
    IpcRegs.IPCACK.bit.IPC5 = 1;                  //Limpa a flag do IPC5

//
// Configure the ADC and power it up
//
    ConfigureADC();

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

    // Ativa o Tipzone dos PWM e desabilita os pulsos at� o comando da flag.GSC_PulsesOn for habilitado
    EALLOW;
    EPwm6Regs.TZSEL.bit.OSHT1  = 0x1; // TZ1 configured for OSHT trip of ePWM6
    EPwm6Regs.TZCTL.bit.TZA    = 0x2;   // Trip action set to force-low for output A
    EPwm6Regs.TZCTL.bit.TZB    = 0x2;   // Trip action set to force-low for output B
    EPwm9Regs.TZSEL.bit.OSHT1  = 0x1; // TZ1 configured for OSHT trip of ePWM9
    EPwm9Regs.TZCTL.bit.TZA    = 0x2;   // Trip action set to force-low for output A
    EPwm9Regs.TZCTL.bit.TZB    = 0x2;   // Trip action set to force-low for output B
    EPwm10Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM10
    EPwm10Regs.TZCTL.bit.TZA   = 0x2;   // Trip action set to force-low for output A
    EPwm10Regs.TZCTL.bit.TZB   = 0x2;   // Trip action set to force-low for output B
    EDIS;

    //Informa ao CPU1 que o n�cleo 2 j� foi carregado
    IpcRegs.IPCSET.bit.IPC4 = 1;

    //Habilita as Interrup��es. A partir desse ponto as interrup��es s�o chamadas quando requisitadas
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

// Initialize results buffers (Buffers para plot)
    for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
    {
       AdcResults[resultsIndex] = 0;
       AdcResults2[resultsIndex] = 0;
       AdcResults3[resultsIndex] = 0;
    }

    resultsIndex = 0;

    // Inicializa os buffers de aquisi��o de sinal
     for(resultsIndex2 = 0; resultsIndex2 < N_data_log; resultsIndex2++)
     {
         Vbat_vec[resultsIndex2] = 0;
         Ibat_vec[resultsIndex2] = 0;
     }

     resultsIndex2 = 0;

//Seta a variavel para ajuste do offset
inv_nro_muestras = 1.0/N_amostras;

//Loop infinito
 while(1)
{
         //Alterna entre modo de carga e descarga do banco de baterias
         if(flag.Bat_Mode == 1)        //Descarga
         {
             flag.Bat_Discharge = 1;
             flag.Bat_Charge    = 0;

         }
         else if(flag.Bat_Mode == 2)   //Carga
         {
             flag.Bat_Discharge = 0;
             flag.Bat_Charge    = 1;

         }
        //
        // These functions are in the F2837xD_EPwm.c file
        //
        if(flag.BSC_PulsesOn == 1 && flag.Conv_on == 1) ////OBS:Amarrar depois aqui se a pre carga foi conclu�da
        //if(flag.BSC_PulsesOn == 1)
        {
           //Habilita os controladores PIs
           pi_I1_dis.enab = 1;
           pi_I2_dis.enab = 1;
           pi_I3_dis.enab = 1;
           pi_I1_ch.enab = 1;
           pi_I2_ch.enab = 1;
           pi_I3_ch.enab = 1;
           piv_ch.enab   = 1;

           //Habilita as rampas
           I1_Ramp.enab = 1;           //rampa da corrente do bra�o 1
           I2_Ramp.enab = 1;           //rampa da corrente do bra�o 2
           I3_Ramp.enab = 1;           //rampa da corrente do bra�o 3
           VRamp.enab   = 1;            //rampa da tens�o para o modo Buck  (Carga)


           if(flag.Bat_Charge == 1 && flag.Bat_Discharge == 0)
           {
               EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;        // Clear PWM6A on event A, up count
               EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;          // Set PWM6A on event A, down count
               EPwm9Regs.AQCTLA.bit.CAU = AQ_CLEAR;        // Clear PWM9A on event A, up count
               EPwm9Regs.AQCTLA.bit.CAD = AQ_SET;          // Set PWM9A on event A, down count
               EPwm10Regs.AQCTLA.bit.CAU = AQ_CLEAR;       // Clear PWM10A on event A, up count
               EPwm10Regs.AQCTLA.bit.CAD = AQ_SET;         // Set PWM10A on event A, down count

               // Ativa o Tipzone e desabilita os pulsos dos ePWMxA e Desativa o Tipzone e habilita os pulsos dos ePWMxB
               EALLOW;
               EPwm6Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM6
               EPwm6Regs.TZCTL.bit.TZA = 0x3;   // Do nothing, no action is taken on EPWMxA
               EPwm6Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
               EPwm9Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM9
               EPwm9Regs.TZCTL.bit.TZA = 0x3;   // Do nothing, no action is taken on EPWMxA
               EPwm9Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
               EPwm10Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM10
               EPwm10Regs.TZCTL.bit.TZA = 0x3;   // Do nothing, no action is taken on EPWMxA
               EPwm10Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
               EDIS;

           }

           if(flag.Bat_Discharge == 1 && flag.Bat_Charge == 0)
           {
               EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM6A on event A, up count
               EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM6A on event A, down count
               EPwm9Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM9A on event A, up count
               EPwm9Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM9A on event A, down count
               EPwm10Regs.AQCTLA.bit.CAU = AQ_SET;           // Set PWM10A on event A, up count
               EPwm10Regs.AQCTLA.bit.CAD = AQ_CLEAR;         // Clear PWM10A on event A, down count

               // Ativa o Tipzone e desabilita os pulsos dos ePWMxA e Desativa o Tipzone e habilita os pulsos dos ePWMxB
               EALLOW;
               EPwm6Regs.TZSEL.bit.OSHT1 = 0x1;  // TZ1 configured for OSHT trip of ePWM6
               EPwm6Regs.TZCTL.bit.TZA = 0x2;    // Trip action set to force-low for output A
               EPwm6Regs.TZCTL.bit.TZB = 0x3;    // Do nothing, no action is taken on EPWMxB
               EPwm9Regs.TZSEL.bit.OSHT1 = 0x1;  // TZ1 configured for OSHT trip of ePWM9
               EPwm9Regs.TZCTL.bit.TZA = 0x2;    // Trip action set to force-low for output A
               EPwm9Regs.TZCTL.bit.TZB = 0x3;    // Do nothing, no action is taken on EPWMxB
               EPwm10Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM10
               EPwm10Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
               EPwm10Regs.TZCTL.bit.TZB = 0x3;   // Do nothing, no action is taken on EPWMxB
               EDIS;
           }
        }
        else
        {
           //Desabilita os controladores PIs
           pi_I1_dis.enab = 0;
           pi_I2_dis.enab = 0;
           pi_I3_dis.enab = 0;
           pi_I1_ch.enab  = 0;
           pi_I2_ch.enab  = 0;
           pi_I3_ch.enab  = 0;
           piv_ch.enab    = 0;

           //Desabilita as rampas
           I1_Ramp.enab = 0;           //rampa da corrente do bra�o 1
           I2_Ramp.enab = 0;           //rampa da corrente do bra�o 2
           I3_Ramp.enab = 0;           //rampa da corrente do bra�o 3
           VRamp.enab   = 0;            //rampa da tens�o para o modo Buck  (Carga)

           // Ativa o Tipzone dos PWM e desabilita os pulsos
           EALLOW;
           EPwm6Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM6
           EPwm6Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
           EPwm6Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
           EPwm9Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM9
           EPwm9Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
           EPwm9Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
           EPwm10Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM10
           EPwm10Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
           EPwm10Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
           EDIS;

        }


        // Sele��o das vari�veis que ser�o plotadas no gr�fico do Gui Composer
        if(flag.real_time_buff == 1)
        {
           switch(selecao_plot)
           {
              case 0: //Default
              AdcResults[resultsIndex]  = entradas_dc.I1;
              AdcResults2[resultsIndex] = I1_Ramp.atual;
              AdcResults3[resultsIndex] = 0;
              break;

              case 1:
              AdcResults[resultsIndex]  = entradas_dc.I2;
              AdcResults2[resultsIndex] = I2_Ramp.atual;
              AdcResults3[resultsIndex] = 0;
              break;

              case 2:
              AdcResults[resultsIndex]  = entradas_dc.I3;
              AdcResults2[resultsIndex] = I3_Ramp.atual;
              AdcResults3[resultsIndex] = 0;
              break;

              case 3:
              AdcResults[resultsIndex]  = pwm_dc.din;
              AdcResults2[resultsIndex] = 0;
              AdcResults3[resultsIndex] = 0;
              break;

              case 4:
              AdcResults[resultsIndex]  =  entradas_dc.Vb1;
              AdcResults2[resultsIndex] =  entradas_dc.Vb2;
              AdcResults3[resultsIndex] =  entradas_dc.Vb3;
              break;

              case 5:
              AdcResults[resultsIndex]  = 0;
              AdcResults2[resultsIndex] = 0;
              AdcResults3[resultsIndex] = 0;
              break;
           }
        }

        GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;            // GPIO para verificar a freq de amostragem
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;            // GPIO para verificar a freq de amostragem
        GpioDataRegs.GPCCLEAR.bit.GPIO68 = 1;            // GPIO para verificar a freq de amostragem
}
}

//Interrup��o do IPC2 para comunica��o com a CPU02
interrupt void IPC2_INT(void)
{
    Recv.recv0 = IpcRegs.IPCRECVADDR;
    IpcRegs.IPCACK.bit.IPC2 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/////////////////////////////////////////////////Interrup��es: Controle///////////////////////////////////
// adca1_isr - Read ADC Buffer in ISR
interrupt void adca1_isr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO64 = 1;                            // GPIO para verificar a freq de amostragem

    // Fun��o de Prote��o
    TUPA_protect();

    // Fun��o de parada de funcionamento do sistema
    TUPA_StopSequence();

    // Fun��o de in�cio de funcionamento do sistema
    TUPA_StartSequence();

    //Modo de Stand by, importante para desconectar este conversor do sistema sem acionar a prote��o do inversor e do outro conversor
    Stand_by_mode();

    //Vari�vel compartilhada entre os n�cleos
    Send.send0 = flag.Shutdown_Conv;

    //Envia a v�riaveis para o npucleo 2
    IpcRegs.IPCSENDADDR = (Uint32) &Send.send0;
    IpcRegs.IPCSET.bit.IPC1 = 1;

    //Piscar o LED 3 em uma determinada frequecia
    Counts.count7 ++;

    if(Counts.count7 >= 3600)
    {
        if(GpioDataRegs.GPBDAT.bit.GPIO34 == 1) GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
        else GpioDataRegs.GPBSET.bit.GPIO34 = 1;
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

    //Verifica o offset das medi��es
    if(first_scan == 1)
    {
        Offset_Calculation();
    }
    else
    {
           //Correntes do Bra�o 1 do Conv cc/cc
           entradas_dc.I1 = 0.007316831214635*AdcaResultRegs.ADCRESULT0 - 0.007316831214635*channel_offset.CH_1;

           //Tens�o do Dc-link
           fil2nVdc.x = 0.328687*AdccResultRegs.ADCRESULT0 - 764.5;
           TUPA_Second_order_filter(&fil2nVdc);
           entradas_dc.Vdc = fil2nVdc.y;

           //Medi��o 1 da Tens�o do banco de baterias
           entradas_dc.Vb1 = 0.400610162445055*AdccResultRegs.ADCRESULT1 - gn;
           fil1nVbat.Un = entradas_dc.Vb1;
           TUPA_First_order_signals_filter(&fil1nVbat);          //filtra a tens�o da bateria

           /*
           //Medi��o 2 da Tens�o do banco de baterias
           entradas_dc.Vb2 = 0.401798078111343*AdccResultRegs.ADCRESULT2 - 0.401798078111343*channel_offset.CH_5 + 12.45;
           MAVv2bat.x = entradas_dc.Vb2;
           TUPA_Moving_Average(&MAVv2bat);          //filtra a tens�o da bateria com a m�dia m�vel

           //Medi��o 3 da Tens�o do banco de baterias
           entradas_dc.Vb3 = 0.403297325473409*AdccResultRegs.ADCRESULT3 - 0.403297325473409*channel_offset.CH_6 + 12.45;
           MAVv3bat.x = entradas_dc.Vb3;
           TUPA_Moving_Average(&MAVv3bat);          //filtra a tens�o da bateria com a m�dia m�vel
            */

           //M�dia das medi��es do banco de baterias
           entradas_dc.Vbt_filt = fil1nVbat.Yn;                //filtrado
           entradas_dc.Vbt      = entradas_dc.Vb1; //N�o filtrado
           /////////////////////////////////Aquisi��o dos sinais//////////////////////////////////////////////////////
           if(flag.data_logo_init == 1)
           {
               Counts.count9++;
               if(Counts.count9 >= COUNT_LIM_LOG)
               {
                   resultsIndex2++;
                   Counts.count9 = 0;

                   Vbat_vec[resultsIndex2] = entradas_dc.Vbt_filt;
                   Ibat_vec[resultsIndex2] = entradas_dc.I1+entradas_dc.I2+entradas_dc.I3;
               }

           }

         ///////////////////////////////Rampas////////////////////////////////////////////////
         TUPA_Ramp(&I1_Ramp);                      //Rampa de refer�ncia da corrente para o modo de descarga
         TUPA_Ramp(&VRamp);                          //Rampa da refer�ncia da tens�o para o modo de descarga


         ///////////////////////////////SOC Estimation/////////////////////////////////////////
         if(soc_est.enable == 1)
         {
             soc_est.Iin = entradas_dc.I1+entradas_dc.I2+entradas_dc.I3;
             soc_est.Vin = entradas_dc.Vbt_filt;
             soc_estimation(&soc_est);
         }
        //////////////////////////////////controle de Corrente modo Boost (Descarga)//////////////////////////////
        if(flag.Bat_Discharge == 1 && flag.Bat_Charge == 0)
        {
            //rampa de varia��o das correntes nos tr�s bra�os
            if(I_dis_ref>Ir_dis)  I_dis_ref = Ir_dis;                       //Trava I_dis_ref no valor m�ximo Ir_dis (Refer�ncia m�xima de corrente)
            I1_Ramp.final = __divf32(I_dis_ref,Nb_int);
            I1_Ramp.in    = __divf32(entradas_dc.I1+entradas_dc.I2+entradas_dc.I3,Nb_int);
            I2_Ramp.final = I1_Ramp.final;
            I2_Ramp.in    = I1_Ramp.in;
            I3_Ramp.final = I1_Ramp.final;
            I3_Ramp.in    = I1_Ramp.in;

            //Controle de Corrente
            pi_I1_dis.setpoint = I1_Ramp.atual;
            pi_I1_dis.feedback = entradas_dc.I1;
            TUPA_Pifunc(&pi_I1_dis);

            //PWM
            pwm_dc.din = pi_I1_dis.output;
        }
        else
        {
            //Reseta as rampas de corrente: Importante para que no pr�ximo ciclo a rampa atue
            I1_Ramp.final = 0;
            I2_Ramp.final = 0;
            I3_Ramp.final = 0;
        }

        //////////////////////////////////controle do modo Buck (Carga a Corrente e Tens�o Constante)/////////////////////
        if(flag.Bat_Charge == 1 && flag.Bat_Discharge == 0)
        {
            //Refer�ncia e rampa da tens�o
            Vref = Vboost*Nbat_series;       //seta a refer�ncia de tens�o para a tens�o de boost
            VRamp.final = Vref;
            VRamp.in =  entradas_dc.Vbt_filt;

           // Malha externa - controle da tens�o

            //Controlador PI
            piv_ch.setpoint =  VRamp.atual;
            piv_ch.feedback =  entradas_dc.Vbt_filt;
            if(I_ch_ref>Ir_ch)  I_ch_ref = Ir_ch;                       //Trava I_dis_ref no valor m�ximo Ir_ch (Refer�ncia m�xima de corrente)
            piv_ch.outMin   = -10;
            piv_ch.outMax   = __divf32(I_ch_ref,3);
            TUPA_Pifunc(&piv_ch);

            //setpoint para a malha interna
            pi_I1_ch.setpoint = piv_ch.output;

            //Malha interna - controle de corrente

            //Controle de Corrente (PI)
            pi_I1_ch.feedback = -entradas_dc.I1;
            TUPA_Pifunc(&pi_I1_ch);
            //refer�ncia para o PWM
            pwm_dc.din = pi_I1_ch.output;

        }
        else
        {
            //Reseta a rampa de tens�o: Importante para que no pr�ximo ciclo a rampa atue
            VRamp.final = 0;
        }

        //PWM
        TUPA_pwm(&pwm_dc,EPwm6Regs.TBPRD);

       // duty cycle
          EPwm6Regs.CMPA.bit.CMPA = pwm_dc.Ta;
        //EPwm6Regs.CMPA.bit.CMPA = (50000000/PWM_FREQ)*0.5;
    }

   //Limpa a Flag da interrup��o. Se n�o limpar, a interrup��o n�o � chamada novamente
   AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //ADC Interrupt 1 Flag. Reading these flags indicates if the associated ADCINT pulse was generated since the last clear.
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //Limpa a flag da interrup��o da correspondente linha. Se n�o fazer isso, uma nova interrup��o n�o � poss�vel pq essa flag n�o � limpa
}

// adca2_isr - Read ADC Buffer in ISR
interrupt void adca2_isr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;                            // GPIO para verificar a freq

    // Fun��o de Prote��o
    TUPA_protect();

    //Verifica o offset das medi��es
    if(first_scan2 == 1)
    {
        Offset_Calculation();
    }
    else
    {
       //Corrente do Bra�o 2 do Conv cc/cc
        entradas_dc.I2 = 0.007310862860686*AdcaResultRegs.ADCRESULT1 - 0.007310862860686*channel_offset.CH_2;

        ///////////////////////////////Rampas////////////////////////////////////////////////
        TUPA_Ramp(&I2_Ramp);                      //Rampa de refer�ncia da corrente para o modo de descarga


        //////////////////////////////////controle de Corrente modo Boost (Descarga)/////////////////////
        if(flag.Bat_Discharge == 1 && flag.Bat_Charge == 0)
        {

            //Controle de Corrente
            pi_I2_dis.setpoint = I2_Ramp.atual;
            pi_I2_dis.feedback = entradas_dc.I2;
            TUPA_Pifunc(&pi_I2_dis);

            //PWM
            pwm_dc2.din = pi_I2_dis.output;
        }

        //////////////////////////////////controle de Corrente modo Buck (Carga a Corrente Constante)/////////////////////
        if(flag.Bat_Charge == 1 && flag.Bat_Discharge == 0)
        {

            // Refer�ncia de corrente para equaliza��o
            pi_I2_ch.setpoint = pi_I1_ch.setpoint;    //setpoint para a malha interna para balancear as correntes

            //Controle de Corrente
            pi_I2_ch.feedback = -entradas_dc.I2;
            TUPA_Pifunc(&pi_I2_ch);
            //refer�ncia para o PWM
            pwm_dc2.din = pi_I2_ch.output;
        }

        //PWM
        TUPA_pwm(&pwm_dc2,EPwm9Regs.TBPRD);

          EPwm9Regs.CMPA.bit.CMPA = pwm_dc2.Ta;
        //EPwm9Regs.CMPA.bit.CMPA = (50000000/PWM_FREQ)*0.5;
    }

    //Limpa a Flag da interrup��o. Se n�o limpar, a interrup��o n�o � chamada novamente
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;  //ADC Interrupt 2 Flag. Reading these flags indicates if the associated ADCINT pulse was generated since the last clear.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10; //Limpa a flag da interrup��o da correspondente linha. Se n�o fazer isso, uma nova interrup��o n�o � poss�vel pq essa flag n�o � limpa
}

// adca3_isr - Read ADC Buffer in ISR
interrupt void adca3_isr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO68 = 1;                            // GPIO para verificar a freq

    // Fun��o de Prote��o
    TUPA_protect();

    //Verifica o offset das medi��es
    if(first_scan3 == 1)
    {
        Offset_Calculation();
    }
    else
    {
        //Corrente do Bra�o 2 do Conv cc/cc
         entradas_dc.I3 = 0.007432592601990*AdcaResultRegs.ADCRESULT2 - 0.007432592601990*channel_offset.CH_3;

         ///////////////////////////////Rampas////////////////////////////////////////////////
         TUPA_Ramp(&I3_Ramp);                      //Rampa de refer�ncia da corrente

         //////////////////////////////////controle de Corrente modo Boost (Descarga)/////////////////////
         if(flag.Bat_Discharge == 1 && flag.Bat_Charge == 0)
         {
             //Controle de Corrente
             pi_I3_dis.setpoint = I3_Ramp.atual;
             pi_I3_dis.feedback = entradas_dc.I3;
             TUPA_Pifunc(&pi_I3_dis);

             //PWM
             pwm_dc3.din = pi_I3_dis.output;
         }

         //////////////////////////////////controle de Corrente modo Buck (Carga a Corrente Constante)/////////////////////
         if(flag.Bat_Charge == 1 && flag.Bat_Discharge == 0)
         {
             // Refer�ncia de corrente para equaliza��o
             pi_I3_ch.setpoint = pi_I1_ch.setpoint;    //setpoint para a malha interna para balancear as correntes

             //Controle de Corrente
             pi_I3_ch.feedback = -entradas_dc.I3;
             TUPA_Pifunc(&pi_I3_ch);
             //refer�ncia para o PWM
             pwm_dc3.din = pi_I3_ch.output;
         }

         //PWM
         TUPA_pwm(&pwm_dc3,EPwm10Regs.TBPRD);

           EPwm10Regs.CMPA.bit.CMPA = pwm_dc3.Ta;
         //EPwm10Regs.CMPA.bit.CMPA = (50000000/PWM_FREQ)*0.5;
    }

    //Limpa a Flag da interrup��o. Se n�o limpar, a interrup��o n�o � chamada novamente
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;  //ADC Interrupt 3 Flag. Reading these flags indicates if the associated ADCINT pulse was generated since the last clear.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10; //Limpa a flag da interrup��o da correspondente linha. Se n�o fazer isso, uma nova interrup��o n�o � poss�vel pq essa flag n�o � limpa
}

//////////////////////////////////////////////////Fun��es de Controle//////////////////////////////////////
// Fun��o Rampa
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

//Filtro segunda ordem
void TUPA_Second_order_filter(sFilter2nd *filt)
{
    filt->y = filt->x*filt->c0 + filt->x_ant*filt->c1 + filt->x_ant2*filt->c2 - filt->y_ant*filt->c3 - filt->y_ant2*filt->c4;
    filt->x_ant2 = filt->x_ant;
    filt->x_ant  = filt->x;
    filt->y_ant2 = filt->y_ant;
    filt->y_ant  = filt->y;
}

// Low pass filter
void TUPA_First_order_signals_filter(sFilter1st *x)
{

    x->Yn= (x->c0* x->Un) + (1-x->c1)*(x->Yn_1);
    x->Un_1= x->Un;
    x->Yn_1= x->Yn;

}

// Controlador PI
void TUPA_Pifunc(sPI *reg)
{
    if(reg->enab==1)
    {
        reg->error = reg->setpoint - reg->feedback;
        reg->errorpi = reg->error - (__divf32(1,reg->Kp))*reg->dif;
        reg->integr = reg->integr_ant + Ts_div2*(reg->errorpi + reg->errorpi_ant);
        reg->integr_ant = reg->integr;
        reg->errorpi_ant = reg->errorpi;
    }
    else
    {
        reg->error       = 0;
        reg->errorpi     = 0;
        reg->integr      = 0;
        reg->output      = 0;
        reg->output_sat  = 0;
    }

    reg->output_sat = reg->Kp*reg->error + reg->Ki*reg->integr;

    reg->output = reg->output_sat;

    if (reg->output > reg->outMax)
    {
        reg->output = reg->outMax;
    }
    else if (reg->output < reg->outMin)
    {
        reg->output = reg->outMin;
    }

    reg->dif = reg->output_sat - reg->output;

}

// PWM
void TUPA_pwm(sPWM *svp, Uint16 fpwm_cnt)
{

    svp->Ta = fpwm_cnt*svp->din;

}

void soc_estimation(sSoc *soc)
{
    int i = 0;

    if (soc->flag_init < 100)
    {
        soc->VbatIn_soc_est = soc->Vin;

        for(i=0; i<51; i++)
        {
            if(soc->VbatIn_soc_est < soc->vbat_ocv[i])
            {
                soc->soc_init = (soc->soc_ocv[i] + soc->soc_ocv_ant)/2;
                break;
            }
            if (i >= 50)
            {
                soc->soc_init = 100;
                break;
            }

            soc->soc_ocv_ant = soc->soc_ocv[i];
        }

        soc->flag_init += 1;
    }

    soc->Filt_current.Un = soc->Iin;
    TUPA_First_order_signals_filter(&(soc->Filt_current));
    soc->x = soc->Filt_current.Yn;

    soc->inte = soc->inte_ant + Ts_div2 * (soc->x  + soc->x_ant);
    soc->x_ant = soc->x;
    soc->inte_ant = soc->inte;

    soc->q = soc->inte + soc->qn*(1-soc->soc_init/100)*soc->tsc;

    soc->soc_out = 100*(1-(soc->q/soc->tsc)/soc->qn);
}

/////////////////////////////////////Fun��es de Sistema//////////////////////////////////////////
// Fun��o de Prote��o do Sistema
void TUPA_protect(void)
{
    // Prote��o de sobrecorrente no Conversor cc/cc
    //Descarga
   if (flag.Bat_Discharge == 1)
   {
       if(fabs(entradas_dc.I1) > OVER_CURRENT_DC_LIMIT_DISCHARGE || fabs(entradas_dc.I2) > OVER_CURRENT_DC_LIMIT_DISCHARGE || fabs(entradas_dc.I3) > OVER_CURRENT_DC_LIMIT_DISCHARGE)
       {
           Counts.count3++;

           if(Counts.count3 > 12)
           {
               flag.Shutdown_Conv = 1;
               fault = FAULT_OVERCURRENT;
               Counts.count3 = 0;
           }
       }

       else
       {
           Counts.count3 = 0;
       }
   }

   //Carga
  if (flag.Bat_Charge == 1)
  {
      if(fabs(entradas_dc.I1) > OVER_CURRENT_DC_LIMIT_CHARGE || fabs(entradas_dc.I2) > OVER_CURRENT_DC_LIMIT_CHARGE || fabs(entradas_dc.I3) > OVER_CURRENT_DC_LIMIT_CHARGE)
      {
          Counts.count11++;

          if(Counts.count11 > 12)
          {
              flag.Shutdown_Conv = 1;
              fault = FAULT_OVERCURRENT;
              Counts.count11 = 0;
          }
      }

      else
      {
          Counts.count11 = 0;
      }
  }

   // Prote��o de sobretens�o na bateria
   if(entradas_dc.Vbt_filt > BAT_OVERVOLTAGE_LIMIT)
   {
       Counts.count8++;

       if(Counts.count8 > 6)
       {
         flag.Shutdown_Conv = 1;
         fault = FAULT_VBAT_OVERVOLTAGE;
         Counts.count8 = 0;
       }
   }
   else
   {
       Counts.count8 = 0;
   }

   /*
   // Prote��o de subtens�o na bateria
   if(entradas_dc.Vbt_mav < BAT_UNDERVOLTAGE_LIMIT && flag.Shutdown_Conv == 1)
   {
         Counts.count10++;

         if(Counts.count10 > 6)
         {
             flag.Shutdown_Conv = 1;
             fault = FAULT_VBAT_UNDERVOLTAGE;
         }
   }
   else
   {
       Counts.count10 = 0;
   }
   */


   // Prote��o de sobretens�o no dc-link
   if(entradas_dc.Vdc > DC_OVERVOLTAGE_LIMIT)
   {
       Counts.count4++;

       if(Counts.count4 > 6)
       {
         flag.Shutdown_Conv = 1;
         fault = FAULT_DC_OVERVOLTAGE;
         Counts.count4 = 0;
       }
   }
   else
   {
       Counts.count4 = 0;
   }

   //Verifica se a flag Shutdown foi acionada na CPU01. Se sim, seta a flag Shutdown_Conv para a CPU2
   if(*Recv.recv0 == 1 && flag.AbleToStart == 1) flag.Shutdown_Conv = 1;

}

// Fun��o de in�cio de funcionamento do sistema
void TUPA_StartSequence(void)
{
    //Verifica se a flag Shutdown est� acionado
    ////OBS: (Amarrar depois dos testes)
     if(flag.Shutdown_Conv == 0)
     {
         if(*Recv.recv0 == 0) flag.AbleToStart = 1;

         //Verifica se a flag do interleaved est� habilitada. Fecha contatores entre baterias e Conversor cc/cc
         //OBS:Ap�s conectar o conv no dc-link, colocar que esses contatores s� podem ser acionados se   flag.precharge_ok est� ok.
          // Inicia o Start do sistema
          if(flag.Conv_on == 1)
          {
              GpioDataRegs.GPBSET.bit.GPIO57 = 1;   //fecha contator dc 1
              GpioDataRegs.GPBSET.bit.GPIO56 = 1;   //fecha contator dc 2
              GpioDataRegs.GPBSET.bit.GPIO55 = 1;   //fecha contator dc 3
          }
     }
}

// Fun��o de parada de funcionamento do sistema
void TUPA_StopSequence(void)
{
    //Verifica se a flag Shutdown_conv est� acionado ou se a Shutdown da CPU1 est� acionada (IPC6) e interrompe o chaveamento e abre os contatores
     if(flag.Shutdown_Conv == 1)
     {
         flag.AbleToStart = 0;
         flag.Conv_on = 0;                             // Reseta o valor de flag.Conv_on
         flag.BSC_PulsesOn = 0;                        // Interrompe o Chaveamtno do conv cc/cc
         // Abre todos os contatore
         GpioDataRegs.GPBCLEAR.bit.GPIO57 =  1;       //Abre contator dc 1
         GpioDataRegs.GPBCLEAR.bit.GPIO56 =  1;       //Abre contator dc 2
         GpioDataRegs.GPBCLEAR.bit.GPIO55 =  1;       //Abre contator dc 3

     }

}

// Modo de Standy by do conversor
void Stand_by_mode(void)
{
    //Verifica se a flag Shutdown_conv est� acionado ou se a Shutdown da CPU1 est� acionada (IPC6) e interrompe o chaveamento e abre os contatores
     if(flag.Stand_by == 1)
     {
         flag.Conv_on = 0;                             // Reseta o valor de flag.Conv_on
         flag.BSC_PulsesOn = 0;                        // Interrompe o Chaveamtno do conv cc/cc

         // Abre todos os contatore
         GpioDataRegs.GPBCLEAR.bit.GPIO57 =  1;       //Abre contator dc 1
         GpioDataRegs.GPBCLEAR.bit.GPIO56 =  1;       //Abre contator dc 2
         GpioDataRegs.GPBCLEAR.bit.GPIO55 =  1;       //Abre contator dc 3
     }

}

// Calculo do offset das medi��es do conv 1
void Offset_Calculation(void)
{
    if(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 1) // Veirifica se a interrup��o 1 foi setada
    {
        // Calcula o offset
        if(Counts.count1 < N_amostras)
        {
            Counts.count1 += 1;
            sum_CH1 += AdcaResultRegs.ADCRESULT0;
        }

        if(Counts.count1 == N_amostras)
        {
            first_scan = 0;
            Counts.count1 = 0;
            channel_offset.CH_1 = sum_CH1 * inv_nro_muestras;
        }
    }

    if(AdcaRegs.ADCINTFLG.bit.ADCINT2 == 1) // Veirifica se a interrup��o 2 foi setada
    {
        // Calcula o offset
        if(Counts.count5 < N_amostras)
        {
            Counts.count5 += 1;
            sum_CH2 += AdcaResultRegs.ADCRESULT1;
        }

        if(Counts.count5 == N_amostras)
        {
            first_scan2 = 0;
            Counts.count5 = 0;
            channel_offset.CH_2 = sum_CH2 * inv_nro_muestras;
        }

    }

    if(AdcaRegs.ADCINTFLG.bit.ADCINT3 == 1) // Veirifica se a interrup��o 3 foi setada
    {
        // Calcula o offset
        if(Counts.count6 < N_amostras)
        {
            Counts.count6 += 1;
            sum_CH3 += AdcaResultRegs.ADCRESULT2;
        }

        if(Counts.count6 == N_amostras)
        {
            first_scan3 = 0;
            Counts.count6 = 0;
            channel_offset.CH_3 = sum_CH3 * inv_nro_muestras;
        }
    }

}










