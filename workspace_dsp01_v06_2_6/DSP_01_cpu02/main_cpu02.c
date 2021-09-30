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

#define PI_I_DEFAULTS    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PI_I_KP, PI_I_KI, 0, 0, PI_I_OUTMIN , PI_I_OUTMAX, 0}
#define PI_VCH_DEFAULTS     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PI_VCH_KP, PI_VCH_KI, 0, 0, 0, 0, 0}
#define PI_VOUT_DEFAULTS  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PI_VOUT_KP, PI_VOUT_KI, 0, 0, PI_VOUT_OUTMIN , PI_VOUT_OUTMAX, 0}
sPI pi_I1 = PI_I_DEFAULTS;
sPI pi_I2 = PI_I_DEFAULTS;
sPI pi_I3 = PI_I_DEFAULTS;
sPI piv_ch    = PI_VCH_DEFAULTS;
sPI PI_vout = PI_VOUT_DEFAULTS;

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
    unsigned int BVCM;
    unsigned int data_logo_init;
    unsigned int real_time_buff;
    unsigned int Com_DSP1_read;
}sFlags;

#define FLAGS_DEFAULTS {0,0,0,0,0,0,0,0,0,1}
sFlags flag = FLAGS_DEFAULTS;

//Rampa
typedef struct{
    float t1;
    float t1_ant;
    float y;
    float y_ant;
    float uin;
    float rate;
    float Ts;
    float rising;
    float falling;
} sRamp;
#define IRamp_default {0,0,0,0,0,0,TSAMPLE,(15),(-15)}
#define VRamp_default {0,0,0,0,0,0,TSAMPLE,(250),(-250)}
sRamp  I1_Ramp   = IRamp_default;
sRamp  I2_Ramp   = IRamp_default;
sRamp  I3_Ramp   = IRamp_default;
sRamp  VRamp     = VRamp_default;
sRamp  VoutRamp  = VRamp_default;

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
#define FILTER_DEFAULTS_1_HZ {0,0,0,0,0.00069813170079773186,0.00069813170079773186}
#define FILTER_DEFAULTS_0_1_HZ {0,0,0,0,0.00006981317007977319,0.00006981317007977319}
sFilter1st fil1nVbat = FILTER_DEFAULTS_0_1_HZ;
sFilter1st fil1nVbat2 = FILTER_DEFAULTS_0_1_HZ;
sFilter1st fil1nVbat3 = FILTER_DEFAULTS_0_1_HZ;
sFilter1st Filt_freq_Vdc = FILTER_DEFAULTS_0_1_HZ;

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

//Váriaveis para enviar dados do CPU2 para o CPU1
typedef struct{
    unsigned int send0;
    float send1;
}SsendCPU2toCPU1;

#define SEND_DEFAULTS {0,0}
SsendCPU2toCPU1 Send = SEND_DEFAULTS;

//Váriaveis para receber dados do CPU1 para o CPU2
typedef struct{
    unsigned int *recv0;
    unsigned int *recv1;
    float *recv2;

}SrecvCPU1toCPU2;

#define RECV_DEFAULTS {0,0,0}
SrecvCPU1toCPU2 Recv = RECV_DEFAULTS;

///////////////////////////////////////////// Funções ////////////////////////////////////////
// Control
void TUPA_Pifunc(sPI *);
void TUPA_protect(void);
void Stand_by_mode(void);
void TUPA_StartSequence(void);
void TUPA_pwm(sPWM *, Uint16);
void TUPA_StopSequence(void);
void Offset_Calculation(void);
void TUPA_Ramp(sRamp *);
void TUPA_Second_order_filter(sFilter2nd *);
void TUPA_First_order_signals_filter(sFilter1st *);

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

//Variaveis de comunicação entre o CPUs
int send = 0;

//Váriaveis de teste
float  gn   = 754.5;

// Váriáveis de Controle
float Ts = TSAMPLE;
float Van = 0, Vbn = 0, Vcn = 0 , vmin = 0, vmax = 0, Vao = 0, Vbo = 0, Vco = 0;

float I_dis_ref   =  5;                         //Referência da corrente de descarga (modo Boost)
float I_ch_ref    =  5;                         //Referência da corrente de carga (modo Buck)
float Vboost      =  14.4;                      //Tensão de Boost
float Vfloat      =  13.6;                      //Tensão de Float
float Vref        =  0;                         //Referência da tensão de carga
float Pref        = 0;
float Voutref     = 240;

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
    PieVectTable.ADCA3_INT = &adca3_isr;     //function for ADCA interrupt 3
    PieVectTable.IPC3_INT = &IPC3_INT;       //Funcao da interrupcao do IPC para comunicacao das CPus
    PieVectTable.IPC0_INT = &IPC0_INT;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;       //Interrupção ADC_A. Habilita a coluna 1 das interrupções, pg 79 do material do workshop
    PieCtrlRegs.PIEIER10.bit.INTx2 = 1;      //Interrupção ADC_A2. Habilita a coluna 1 das interrupções, pg 79 do material do workshop
    PieCtrlRegs.PIEIER10.bit.INTx3 = 1;      //Interrupção ADC_A3. Habilita a coluna 1 das interrupções, pg 79 do material do workshop
    PieCtrlRegs.PIEIER1.bit.INTx15 = 1;      //interrupção IPC2 de inter comunicação entre os CPUS. Habilita a coluna 15 correspondente
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;      //IPC0 interruption
    PieCtrlRegs.PIEIER1.bit.INTx16 = 1;      //interrupcao IPC3 de inter comunicacao entre os CPUS

// Enable global Interrupts and higher priority real-time debug events:
//
    IER |= (M_INT10+M_INT1); //Habilita a linha da tabela de interrupção. correspondente ao ADC_B, pg 79 do material do workshop

    EDIS;

//Aguarda o CPU1 carregar as configurações da memória e dos periféricos. Após isso, o CPU02 está habilitado para continuar o carregamento dos periféricos associados a ele
//Lembrete. Os IPCs que disparam interrupção são o 0,1,2 e 3. Os outros não tem interrupção e podem ser usados como flags

    while(IpcRegs.IPCSTS.bit.IPC5 == 0);          //Loop finito para aguardar o carregamento do CPU02
    IpcRegs.IPCACK.bit.IPC5 = 1;                  //Limpa a flag do IPC5
    GpioDataRegs.GPASET.bit.GPIO26 = 1;           // Informa para a DSP02 para iniciar o carregamento a partir deste ponto

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

    // Ativa o Tipzone dos PWM e desabilita os pulsos até o comando da flag.GSC_PulsesOn for habilitado
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

    //Informa ao CPU1 que o núcleo 2 já foi carregado
    IpcRegs.IPCSET.bit.IPC4 = 1;

    //Habilita as Interrupções. A partir desse ponto as interrupções são chamadas quando requisitadas
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

    // Inicializa os buffers de aquisição de sinal
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
       //Carrega a flag relacionada com a entrada digital responsável por verificar se a flag Shutdown do conjunto 1 foi acionada
        flag.Com_DSP1_read = GpioDataRegs.GPADAT.bit.GPIO25;    //Estado do contator de conexão com a rede
        //
        // These functions are in the F2837xD_EPwm.c file
        //
        if(flag.BSC_PulsesOn == 1 && flag.Conv_on == 1) ////OBS:Amarrar depois aqui se a pre carga foi concluida
                //if(flag.BSC_PulsesOn == 1)
        {
           //Habilita os controladores PIs
           pi_I1.enab = 1;
           pi_I2.enab = 1;
           pi_I3.enab = 1;
           piv_ch.enab   = 1;
           PI_vout.enab = 1;

           //Reseta para Descarga (1) e Carga (2)
           if(Pref > 0)
           {
             flag.Bat_Discharge = 1;               //Habilita Descarga
             flag.Bat_Charge = 0;               //Desabilita Carga
           }

           else if(Pref < 0)
           {
             flag.Bat_Discharge = 0;               //Desabilita Descarga
             flag.Bat_Charge = 1;               //Aciona o modo de carga
           }

           else if(Pref == 0)
           {
             flag.Bat_Discharge = 0;
             flag.Bat_Charge = 0;
           }


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
           else if(flag.Bat_Discharge == 0 && flag.Bat_Charge == 0)
           {
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
        }
        else
        {
           //Desabilita os controladores PIs
           pi_I1.enab = 0;
           pi_I2.enab = 0;
           pi_I3.enab = 0;
           piv_ch.enab    = 0;
           PI_vout.enab = 0;

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

           VoutRamp.uin = Filt_freq_Vdc.Yn;
           VoutRamp.y = Filt_freq_Vdc.Yn;
        }


        // Seleção das variáveis que serão plotadas no gráfico do Gui Composer
        if(flag.real_time_buff == 1)
        {
           switch(selecao_plot)
           {
              case 0: //Default
              AdcResults[resultsIndex]  = entradas_dc.I1;
              AdcResults2[resultsIndex] = I1_Ramp.y;
              AdcResults3[resultsIndex] = 0;
              break;

              case 1:
              AdcResults[resultsIndex]  = entradas_dc.I2;
              AdcResults2[resultsIndex] = I2_Ramp.y;
              AdcResults3[resultsIndex] = 0;
              break;

              case 2:
              AdcResults[resultsIndex]  = entradas_dc.I3;
              AdcResults2[resultsIndex] = I3_Ramp.y;
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

/////////////////////////////////////////////////Interrupções: Controle///////////////////////////////////
interrupt void IPC3_INT(void)
{
    Recv.recv1 = IpcRegs.IPCRECVADDR;
    flag.BSC_PulsesOn = *Recv.recv1;
    IpcRegs.IPCACK.bit.IPC3 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void IPC0_INT(void)
{
    Recv.recv2 = IpcRegs.IPCRECVADDR;
    Pref = *Recv.recv2;
    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// adca1_isr - Read ADC Buffer in ISR
interrupt void adca1_isr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO64 = 1;                            // GPIO para verificar a freq de amostragem

    // Função de Proteção
    TUPA_protect();

    // Função de parada de funcionamento do sistema
    TUPA_StopSequence();

    // Função de início de funcionamento do sistema
    TUPA_StartSequence();

    //Modo de Stand by, importante para desconectar este conversor do sistema sem acionar a proteção do inversor e do outro conversor
    Stand_by_mode();

    //Piscar o LED 3 em uma determinada frequecia
    Counts.count7 ++;

    if(Counts.count7 >= 3600)
    {
        if(GpioDataRegs.GPBDAT.bit.GPIO34 == 1) GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
        else GpioDataRegs.GPBSET.bit.GPIO34 = 1;
        Counts.count7 = 0;
    }

    // Update the buffers with the ADCResults. Se flag.real_time_buff for igual a 1, os buffers são atualizados a cada período de amostragem
    // Caso contrário, os buffers param de ser atualizados e os dados da memória podem ser exportados

    if(flag.real_time_buff == 1)
    {
       resultsIndex++;
       if(resultsIndex >= RESULTS_BUFFER_SIZE)
       {
          resultsIndex = 0;
       }
    }

    //Verifica o offset das medições
    if(first_scan == 1)
    {
        Offset_Calculation();
    }
    else
    {
           //Correntes do Braço 1 do Conv cc/cc
           entradas_dc.I1 = 0.007432592601990*AdcaResultRegs.ADCRESULT0 - 0.007432592601990*channel_offset.CH_1;

           //Tensão do Dc-link
           Filt_freq_Vdc.Un = 0.322*AdccResultRegs.ADCRESULT0 - 754.5;
           TUPA_First_order_signals_filter(&Filt_freq_Vdc);
           entradas_dc.Vdc = Filt_freq_Vdc.Yn;


           //Medição 1 da Tensão do banco de baterias
           entradas_dc.Vb1 = 0.4049*AdccResultRegs.ADCRESULT1 - 934.0;
           fil1nVbat.Un = entradas_dc.Vb1;
           TUPA_First_order_signals_filter(&fil1nVbat);          //filtra a tensão da bateria

//           //Medição 2 da Tensão do banco de baterias
//           entradas_dc.Vb2 = 0.401798078111343*AdccResultRegs.ADCRESULT2 - gn2;
//           fil1nVbat2.Un = entradas_dc.Vb2;
//           TUPA_First_order_signals_filter(&fil1nVbat2);
//
//           //Medição 3 da Tensão do banco de baterias
//           entradas_dc.Vb3 = 0.403297325473409*AdccResultRegs.ADCRESULT3 - gn3;
//           fil1nVbat3.Un = entradas_dc.Vb3;
//           TUPA_First_order_signals_filter(&fil1nVbat3);

           //Média das medições do banco de baterias
           entradas_dc.Vbt_filt = fil1nVbat.Yn;                //filtrado
           entradas_dc.Vbt     = entradas_dc.Vb1; //Não filtrado
           /////////////////////////////////Aquisição dos sinais//////////////////////////////////////////////////////
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
         TUPA_Ramp(&VRamp);

         //////////////////////////////////controle de Corrente modo Boost (Descarga)//////////////////////////////
         if(flag.Bat_Charge != 0 || flag.Bat_Discharge != 0)
         {
             VoutRamp.uin = Voutref;

             PI_vout.setpoint = VoutRamp.y;
             PI_vout.feedback = Filt_freq_Vdc.Yn;

             TUPA_Pifunc(&PI_vout);                   // Controle
             //Controle de Corrente
             if(flag.Bat_Discharge == 1)
             {
                 pi_I1.setpoint = PI_vout.output / Nb_int;
                 pi_I1.feedback = entradas_dc.I1;
             }
             else if(flag.Bat_Charge == 1)
             {
                 if(PI_vout.output > 10) PI_vout.output = 10;
                 if(PI_vout.output < -10) PI_vout.output = -10;
                 pi_I1.setpoint = -PI_vout.output / Nb_int;
                 pi_I1.feedback = -entradas_dc.I1;
             }

             TUPA_Pifunc(&pi_I1);

             //PWM
             pwm_dc.din = pi_I1.output;
         }

         //PWM
         TUPA_pwm(&pwm_dc,EPwm6Regs.TBPRD);

         // duty cycle
         EPwm6Regs.CMPA.bit.CMPA = pwm_dc.Ta;
         //EPwm6Regs.CMPA.bit.CMPA = (50000000/PWM_FREQ)*0.5;
    }

   //Limpa a Flag da interrupção. Se não limpar, a interrupção não é chamada novamente
   AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //ADC Interrupt 1 Flag. Reading these flags indicates if the associated ADCINT pulse was generated since the last clear.
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //Limpa a flag da interrupção da correspondente linha. Se não fazer isso, uma nova interrupção não é possível pq essa flag não é limpa
}

// adca2_isr - Read ADC Buffer in ISR
interrupt void adca2_isr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;                            // GPIO para verificar a freq

    // Função de Proteção
    TUPA_protect();

    //Verifica o offset das medições
    if(first_scan2 == 1)
    {
        Offset_Calculation();
    }
    else
    {
       //Corrente do Braço 2 do Conv cc/cc
        entradas_dc.I2 =  0.007443243187925*AdcaResultRegs.ADCRESULT1 -  0.007443243187925*channel_offset.CH_2;

        //////////////////////////////////controle de Corrente modo Boost (Descarga)/////////////////////
        if(flag.Bat_Charge != 0 || flag.Bat_Discharge != 0)
        {
            pi_I2.setpoint = pi_I1.setpoint;

            if(flag.Bat_Discharge == 1)
            {
                pi_I2.feedback = entradas_dc.I2;
            }
            else if(flag.Bat_Charge == 1)
            {
                pi_I2.feedback = -entradas_dc.I2;
            }

            //Controle de Corrente
            TUPA_Pifunc(&pi_I2);

            //PWM
            pwm_dc2.din = pi_I2.output;
        }

        //PWM
        TUPA_pwm(&pwm_dc2,EPwm9Regs.TBPRD);

        EPwm9Regs.CMPA.bit.CMPA = pwm_dc2.Ta;
        //EPwm9Regs.CMPA.bit.CMPA = (50000000/PWM_FREQ)*0.5;
    }

    //Limpa a Flag da interrupção. Se não limpar, a interrupção não é chamada novamente
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;  //ADC Interrupt 2 Flag. Reading these flags indicates if the associated ADCINT pulse was generated since the last clear.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10; //Limpa a flag da interrupção da correspondente linha. Se não fazer isso, uma nova interrupção não é possível pq essa flag não é limpa
}

// adca3_isr - Read ADC Buffer in ISR
interrupt void adca3_isr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO68 = 1;                            // GPIO para verificar a freq

    // Função de Proteção
    TUPA_protect();

    //Verifica o offset das medições
    if(first_scan3 == 1)
    {
        Offset_Calculation();
    }
    else
    {
        //Corrente do Braço 2 do Conv cc/cc
         entradas_dc.I3 =  0.007305787609146*AdcaResultRegs.ADCRESULT2 - 0.007305787609146*channel_offset.CH_3;

         //////////////////////////////////controle de Corrente modo Boost (Descarga)/////////////////////
         if(flag.Bat_Charge != 0 || flag.Bat_Discharge != 0)
         {
              //Controle de Corrente
              pi_I3.setpoint = pi_I1.setpoint;

              if(flag.Bat_Discharge == 1)
              {
                  pi_I3.feedback = entradas_dc.I3;
              }
              else if(flag.Bat_Charge == 1)
              {
                  pi_I3.feedback = -entradas_dc.I3;
              }

              TUPA_Pifunc(&pi_I3);

              //PWM
              pwm_dc3.din = pi_I3.output;
         }

         //PWM
         TUPA_pwm(&pwm_dc3,EPwm10Regs.TBPRD);

         EPwm10Regs.CMPA.bit.CMPA = pwm_dc3.Ta;
         //EPwm10Regs.CMPA.bit.CMPA = (50000000/PWM_FREQ)*0.5;
    }

    //Limpa a Flag da interrupção. Se não limpar, a interrupção não é chamada novamente
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;  //ADC Interrupt 3 Flag. Reading these flags indicates if the associated ADCINT pulse was generated since the last clear.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10; //Limpa a flag da interrupção da correspondente linha. Se não fazer isso, uma nova interrupção não é possível pq essa flag não é limpa
}

//////////////////////////////////////////////////Funções de Controle//////////////////////////////////////
// Rampa
void TUPA_Ramp(sRamp *rmp)
{
  if(rmp->uin != rmp->y) rmp->t1 = rmp->t1 + rmp->Ts;

  if(rmp->t1 != rmp->t1_ant)
  {
    rmp->rate = (rmp->uin - rmp->y_ant)/(rmp->t1 - rmp->t1_ant);
  }
  else rmp->rate = 0;

  if(rmp->rate > rmp->rising) rmp->y = (rmp->t1 - rmp->t1_ant)*rmp->rising + rmp->y_ant;
  else if(rmp->rate < rmp->falling) rmp->y = (rmp->t1 - rmp->t1_ant)*rmp->falling + rmp->y_ant;
  else rmp->y = rmp->uin;

  rmp->t1_ant = rmp->t1;
  rmp->y_ant = rmp->y;
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

/////////////////////////////////////Funções de Sistema//////////////////////////////////////////
// Função de Proteção do Sistema
void TUPA_protect(void)
{
    // Proteção de sobrecorrente no Conversor cc/cc
    //Descarga
   if (flag.Bat_Discharge == 1)
   {
       if(fabs(entradas_dc.I1) > OVER_CURRENT_DC_LIMIT_DISCHARGE || fabs(entradas_dc.I2) > OVER_CURRENT_DC_LIMIT_DISCHARGE || fabs(entradas_dc.I3) > OVER_CURRENT_DC_LIMIT_DISCHARGE)
       {
           Counts.count3++;

           if(Counts.count3 > 2)
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

          if(Counts.count11 > 4)
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

   // Proteção de sobretensão na bateria
   if(entradas_dc.Vbt_filt > BAT_OVERVOLTAGE_LIMIT)
   {
       Counts.count8++;

       if(Counts.count8 > 3)
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
   // Proteção de subtensão na bateria
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

   // Proteção de sobretensão no dc-link
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

   //Verifica se a flag Group_com está indicando que a proteção foi acionada no Conjunto 1. Se sim, aciona a flag Shutdown
   if(flag.Com_DSP1_read == 1 && flag.AbleToStart == 1) flag.Shutdown_Conv = 1;

}

// Função de início de funcionamento do sistema
void TUPA_StartSequence(void)
{
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;           // Limpa a flag que informa para a DSP02 que a proteção nesse conjunto foi acionada

    //Verifica se a flag Shutdown está acionado
    ////OBS: (Amarrar depois dos testes)
     if(flag.Shutdown_Conv == 0)
     {
         if(flag.Com_DSP1_read == 0) flag.AbleToStart = 1;
         //Verifica se a flag do interleaved está habilitada. Fecha contatores entre baterias e Conversor cc/cc
         //OBS:Após conectar o conv no dc-link, colocar que esses contatores só podem ser acionados se   flag.precharge_ok está ok.
          // Inicia o Start do sistema
          if(flag.Conv_on == 1)
          {
              GpioDataRegs.GPBSET.bit.GPIO57 = 1;   //fecha contator dc 1
              GpioDataRegs.GPBSET.bit.GPIO56 = 1;   //fecha contator dc 2
              GpioDataRegs.GPBSET.bit.GPIO55 = 1;   //fecha contator dc 3
          }
     }
}

// Função de parada de funcionamento do sistema
void TUPA_StopSequence(void)
{
    //Verifica se a flag Shutdown_conv está acionado ou se a Shutdown da CPU1 está acionada (IPC6) e interrompe o chaveamento e abre os contatores
     if(flag.Shutdown_Conv == 1)
     {
         flag.AbleToStart = 0;
         flag.Conv_on = 0;                             // Reseta o valor de flag.Conv_on
         flag.BSC_PulsesOn = 0;                        // Interrompe o Chaveamtno do conv cc/cc
         Counts.count2 = 0;                            //Contador para a leitura do estado dos contatores
         // Abre todos os contatore
         GpioDataRegs.GPBCLEAR.bit.GPIO57 =  1;       //Abre contator dc 1
         GpioDataRegs.GPBCLEAR.bit.GPIO56 =  1;       //Abre contator dc 2
         GpioDataRegs.GPBCLEAR.bit.GPIO55 =  1;       //Abre contator dc 3

         GpioDataRegs.GPASET.bit.GPIO24 = 1;           // Informa para a DSP02 que a proteção nesse conjunto foi acionada

     }

}

// Modo de Standy by do conversor
void Stand_by_mode(void)
{
    //Verifica se a flag Shutdown_conv está acionado ou se a Shutdown da CPU1 está acionada (IPC6) e interrompe o chaveamento e abre os contatores
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

// Calculo do offset das medições do conv 1
void Offset_Calculation(void)
{
    if(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 1) // Veirifica se a interrupção 1 foi setada
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

    if(AdcaRegs.ADCINTFLG.bit.ADCINT2 == 1) // Veirifica se a interrupção 2 foi setada
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

    if(AdcaRegs.ADCINTFLG.bit.ADCINT3 == 1) // Veirifica se a interrupção 3 foi setada
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










