#ifndef     __TUPA_PARAMETERS_H__
#define     __TUPA_PARAMETERS_H__



// PWM DEFINITIONS
#define PWM_FREQ                9000                             //Frequ�ncia de chaveamento
#define Nsample                 1                           // Raz�o entre freq de amostragem e de chaveamento
#define TSAMPLE                 1.0/(Nsample*PWM_FREQ)            //Per�odo de amorstrage
#define N                       (Nsample*PWM_FREQ)/20             // N Usado para o c�lculo da m�dia movel

// SCI Definitions
#define CPU_FREQ        200E6
#define LSPCLK_FREQ     CPU_FREQ/4
#define SCI_FREQ        2700           // SCI Assync Baud (Baud rate).
#define SCI_PRD         ((LSPCLK_FREQ/(SCI_FREQ*8))-1)
#define len_sci         8

//Constantes
#define DOISPI               6.283185307179586  // 2*PI
#define Ts_div2              TSAMPLE*0.500000

// Tamanho do Buffer de aquisi��o de dados
#define RESULTS_BUFFER_SIZE  2*(Nsample*PWM_FREQ/60)        //Ciclos*(Freq. de amostragem)/(freq.fundamental)

// Tamanho do Buffer de aquisi��o de dados e limite de contagem
#define N_data_log  600               // Aquisi��o de 5h
//#define COUNT_LIM_LOG  270000         //Aquisi��o a cada 30s
#define COUNT_LIM_LOG  4500           //Aquisi��o a cada 0.5s

// Ganhos do controlador PI da PLL
#define PI_PLL_GRID_KP         177
#define PI_PLL_GRID_KI         15791
#define PI_PLL_GRID_OUTMAX     550
#define PI_PLL_GRID_OUTMIN    -550

// Ganhos do controlador PI do Vdc (0.5Hz)
#define PI_Vdc_KP         0.019180928276777
#define PI_Vdc_KI         0.034790355513840
#define PI_Vdc_OUTMAX     50000
#define PI_Vdc_OUTMIN    -50000

// Ganhos do controlador PI do Reativo
#define PI_Q_KI         12.5664
#define PI_Q_OUTMAX     50000
#define PI_Q_OUTMIN    -50000

// Limite de tempo para pr�-carga
#define PRECHARGE_LIMIT        30/(TSAMPLE)      //6 segundos

//Ganhos controladores PR
#define PR_I_GRID_KP    5       //11
#define PR_I_GRID_KI    1000
#define Ir              36

// Limites para prote��es
#define OVER_CURRENT_GRID_LIMIT          30
#define DC_OVERVOLTAGE_LIMIT             680
#define DC_PRECHARGE_LIMIT               280
#define DC_MANUAL_PRECHARGE_LIMIT        260
#define MAX_CHOPPER_LIMIT                620
#define MIN_CHOPPER_LIMIT                450

// Defini��o das faltas
#define FAULT_OK                0
#define FAULT_DC_OVERVOLTAGE    1
#define FAULT_DC_UNDERVOLTAGE   2
#define FAULT_OVERCURRENT       3
#define FAULT_PRECHARGE         4
#define FAULT_THERMAL           5
#define FAULT_OVERFREQUENCY     6
#define FAULT_UNDERFREQUENCY    7


#endif

