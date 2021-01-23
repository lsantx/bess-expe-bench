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
    unsigned int Com_DSP1_read;
}sFlags;

#define FLAGS_DEFAULTS {0,0,0,0,0,0,0,1,0,0,1,0}
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

#define  FILTER2ND_50Hz_DEFAULTS {0,0,0,0,0,0,0.00007522044556316118,0.0001504408911263224,0.00007522044556316118,-1.975322809872539,0.975623691654792}
sFilter2nd fil2nVbat  = FILTER2ND_50Hz_DEFAULTS;
sFilter2nd fil2nVdc   = FILTER2ND_50Hz_DEFAULTS;

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
    float *recv1;
}SrecvCPU1toCPU2;

#define RECV_DEFAULTS {0,0}
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
void TUPA_Ramp(Ramp *);
void TUPA_Second_order_filter(sFilter2nd *);

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
float  gn  = 940.80;

// Váriáveis de Controle
float Ts = TSAMPLE;
float Van = 0, Vbn = 0, Vcn = 0 , vmin = 0, vmax = 0, Vao = 0, Vbo = 0, Vco = 0;

float I_dis_ref   =  5;                         //Referência da corrente de descarga (modo Boost)
float I_ch_ref    =  5;                         //Referência da corrente de carga (modo Buck)
float Vboost      =  13.00;                      //Tensão de Boost
float Vfloat      =  13.6;                      //Tensão de Float
float Vref        =  0;                         //Referência da tensão de carga

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
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;       //Interrupção ADC_A. Habilita a coluna 1 das interrupções, pg 79 do material do workshop
    PieCtrlRegs.PIEIER10.bit.INTx2 = 1;      //Interrupção ADC_A2. Habilita a coluna 1 das interrupções, pg 79 do material do workshop
    PieCtrlRegs.PIEIER10.bit.INTx3 = 1;      //Interrupção ADC_A3. Habilita a coluna 1 das interrupções, pg 79 do material do workshop
    PieCtrlRegs.PIEIER1.bit.INTx15 = 1;      //interrupção IPC2 de inter comunicação entre os CPUS. Habilita a coluna 15 correspondente
//
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
        //Carrega a flag relacionada com a entrada digital responsável por verificar se a flag Shutdown do conjunto 1 foi acionada
         flag.Com_DSP1_read = GpioDataRegs.GPADAT.bit.GPIO25;    //Estado do contator de conexão com a rede
        //
        // These functions are in the F2837xD_EPwm.c file
        //
        if(flag.BSC_PulsesOn == 1 && flag.Conv_on == 1) ////OBS:Amarrar depois aqui se a pre carga foi concluída
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
           I1_Ramp.enab = 1;           //rampa da corrente do braço 1
           I2_Ramp.enab = 1;           //rampa da corrente do braço 2
           I3_Ramp.enab = 1;           //rampa da corrente do braço 3
           VRamp.enab   = 1;            //rampa da tensão para o modo Buck  (Carga)


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

           //Desaabilita as rampas
           I1_Ramp.enab = 0;           //rampa da corrente do braço 1
           I2_Ramp.enab = 0;           //rampa da corrente do braço 2
           I3_Ramp.enab = 0;           //rampa da corrente do braço 3
           VRamp.enab   = 0;            //rampa da tensão para o modo Buck  (Carga)

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


        // Seleção das variáveis que serão plotadas no gráfico do Gui Composer
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

/////////////////////////////////////////////////Interrupções: Controle///////////////////////////////////
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
           fil2nVdc.x= 0.322*AdccResultRegs.ADCRESULT0 - 754.5;
           TUPA_Second_order_filter(&fil2nVdc);
           entradas_dc.Vdc = fil2nVdc.y;


           //Medição 1 da Tensão do banco de baterias
           entradas_dc.Vb1 = 0.4049*AdccResultRegs.ADCRESULT1 - gn;
           fil2nVbat.x = entradas_dc.Vb1;
           TUPA_Second_order_filter(&fil2nVbat);          //filtra a tensão da bateria

           /*
           //Medição 2 da Tensão do banco de baterias
           entradas_dc.Vb1 = 0.401798078111343*AdccResultRegs.ADCRESULT2 - gn;
           fil2nVbat.x = entradas_dc.Vb1;
           TUPA_Second_order_filter(&fil2nVbat);           //filtra a tensão da bateria com a média móvel
           */
           /*
           //Medição 3 da Tensão do banco de baterias
           entradas_dc.Vb3 = 0.403297325473409*AdccResultRegs.ADCRESULT3 - 0.403297325473409*channel_offset.CH_6 + 12.45;
           MAVv3bat.x = entradas_dc.Vb3;
           TUPA_Moving_Average(&MAVv3bat);          //filtra a tensão da bateria com a média móvel
            */

           //Média das medições do banco de baterias
           entradas_dc.Vbt_filt = fil2nVbat.y;                //filtrado
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
         TUPA_Ramp(&I1_Ramp);                      //Rampa de referência da corrente para o modo de descarga
         TUPA_Ramp(&VRamp);                          //Rampa da referência da tensão para o modo de descarga

         //////////////////////////////////controle de Corrente modo Boost (Descarga)//////////////////////////////
         if(flag.Bat_Discharge == 1 && flag.Bat_Charge == 0)
         {
             //rampa de variação das correntes nos três braços
             if(I_dis_ref>Ir_dis)  I_dis_ref = Ir_dis;                       //Trava I_dis_ref no valor máximo Ir_dis (Referência máxima de corrente)
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
             //Reseta as rampas de corrente: Importante para que no próximo ciclo a rampa atue
             I1_Ramp.final = 0;
             I2_Ramp.final = 0;
             I3_Ramp.final = 0;
         }

        //////////////////////////////////controle do modo Buck (Carga a Corrente e Tensão Constante)/////////////////////
        if(flag.Bat_Charge == 1 && flag.Bat_Discharge == 0)
        {
            //Referência e rampa da tensão
            Vref = Vboost*Nbat_series;       //seta a referência de tensão para a tensão de boost
            VRamp.final = Vref;
            VRamp.in =  entradas_dc.Vbt_filt;

           // Malha externa - controle da tensão

            //Controlador PI
            piv_ch.setpoint =  VRamp.atual;
            piv_ch.feedback =  entradas_dc.Vbt_filt;
            if(I_ch_ref>Ir_ch)  I_ch_ref = Ir_ch;                       //Trava I_dis_ref no valor máximo Ir_ch (Referência máxima de corrente)
            piv_ch.outMin   = -10;
            piv_ch.outMax   = __divf32(I_ch_ref,3);
            TUPA_Pifunc(&piv_ch);

            //setpoint para a malha interna
            pi_I1_ch.setpoint = piv_ch.output;

            //Malha interna - controle de corrente

            //Controle de Corrente (PI)
            pi_I1_ch.feedback = -entradas_dc.I1;
            TUPA_Pifunc(&pi_I1_ch);
            //referência para o PWM
            pwm_dc.din = pi_I1_ch.output;

        }
        else
        {
            //Reseta a rampa de tensão: Importante para que no próximo ciclo a rampa atue
            VRamp.final = 0;
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

        ///////////////////////////////Rampas////////////////////////////////////////////////
        TUPA_Ramp(&I2_Ramp);                      //Rampa de referência da corrente para o modo de descarga

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

            // Referência de corrente para equalização
            pi_I2_ch.setpoint = pi_I1_ch.setpoint;    //setpoint para a malha interna para balancear as correntes

            //Controle de Corrente
            pi_I2_ch.feedback = -entradas_dc.I2;
            TUPA_Pifunc(&pi_I2_ch);
            //referência para o PWM
            pwm_dc2.din = pi_I2_ch.output;
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

         ///////////////////////////////Rampas////////////////////////////////////////////////
         TUPA_Ramp(&I3_Ramp);                      //Rampa de referência da corrente

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
             // Referência de corrente para equalização
             pi_I3_ch.setpoint = pi_I1_ch.setpoint;    //setpoint para a malha interna para balancear as correntes

             //Controle de Corrente
             pi_I3_ch.feedback = -entradas_dc.I3;
             TUPA_Pifunc(&pi_I3_ch);
             //referência para o PWM
             pwm_dc3.din = pi_I3_ch.output;
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
// Função Rampa
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

   // Proteção de sobretensão na bateria
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










