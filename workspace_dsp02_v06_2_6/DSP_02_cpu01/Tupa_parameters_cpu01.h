#ifndef 	__TUPA_PARAMETERS_H__
#define 	__TUPA_PARAMETERS_H__



// PWM DEFINITIONS
#define PWM_FREQ                9000                             //Frequï¿½ncia de chaveamento
#define Nsample                 1                           // Razï¿½o entre freq de amostragem e de chaveamento
#define TSAMPLE                 1.0/(Nsample*PWM_FREQ)            //Perï¿½odo de amorstrage
#define N                       (Nsample*PWM_FREQ)/20             // N Usado para o cï¿½lculo da mï¿½dia movel

//Constantes
#define DOISPI               6.283185307179586  // 2*PI
#define Ts_div2              TSAMPLE*0.500000

// Tamanho do Buffer de aquisição de dados
#define RESULTS_BUFFER_SIZE  2*(Nsample*PWM_FREQ/60)        //Ciclos*(Freq. de amostragem)/(freq.fundamental)

// Ganhos do controlador PI da PLL
#define PI_PLL_GRID_KP         177
#define PI_PLL_GRID_KI         15791
#define PI_PLL_GRID_OUTMAX     550
#define PI_PLL_GRID_OUTMIN    -550

// Ganhos do controlador PI do Vdc
#define PI_Vdc_KP         -0.1805
#define PI_Vdc_KI         -0.1031
#define PI_Vdc_OUTMAX     30
#define PI_Vdc_OUTMIN    -30

// Ganhos do controlador PI do Reativo
#define PI_Q_KP         -0.00082304
#define PI_Q_KI         -0.0052
#define PI_Q_OUTMAX     30
#define PI_Q_OUTMIN    -30

// Limite de tempo para pré-carga
#define PRECHARGE_LIMIT        30/(TSAMPLE)      //6 segundos

//Ganhos controladores PR
#define PR_I_GRID_KP    10       //11
#define PR_I_GRID_KI    1000
#define Ir              36

// Limites para proteções
#define OVER_CURRENT_GRID_LIMIT          30
#define DC_OVERVOLTAGE_LIMIT             680
#define DC_PRECHARGE_LIMIT               280
#define DC_MANUAL_PRECHARGE_LIMIT        260
#define MAX_CHOPPER_LIMIT                620
#define MIN_CHOPPER_LIMIT                450

// Definição das faltas
#define FAULT_OK                0
#define FAULT_DC_OVERVOLTAGE    1
#define FAULT_DC_UNDERVOLTAGE   2
#define FAULT_OVERCURRENT       3
#define FAULT_PRECHARGE         4
#define FAULT_THERMAL           5
#define FAULT_OVERFREQUENCY     6
#define FAULT_UNDERFREQUENCY    7


#endif

