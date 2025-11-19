/**
 * @file Generador-Osciloscopio.c
 * @brief Generador de funciones / Osciloscopio con LPC1769
 *
 * Este programa implementa un generador de señales (senoidal, cuadrada y triangular)
 * y una función de osciloscopio que permite visualizar señales adquiridas.
 *
 * @authores Choque, Fernando  Muñoz, Jose Luis  Mendoza, Marcos Julian
 * @date Noviembre 2025
 * @version 1.0
 */


//INCLUDES:
#include <LPC17xx.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_exti.h>
#include <lpc17xx_timer.h>
#include <lpc17xx_adc.h>
#include <lpc17xx_dac.h>
#include <lpc17xx_gpdma.h>
#include <lpc17xx_uart.h>
#include <lpc17xx_gpio.h>

//DEFINICION DE MACROS Y CONSTANTES:
#define PIN_27 ((uint32_t) (1<<27))
#define PIN_28 ((uint32_t) (1<<28))
#define OUTPUT 1
#define PORT_0 0

#define FREC_ADC 200000
#define DAC_BUFFER_START_0 0X2007C000 //CUADRADA
#define DAC_BUFFER_START_1 0X2007D000	//TRIANGULAR
#define DAC_BUFFER_START_2 0X2007E000 //SENOIDAL

#define WAVEFORM_SIZE 382
#define WAVEFORM_SIZE_HALF 191
#define CUADRANTE 95
#define DAC_OFFSET 512
#define DAC_MAX	1023
#define DAC_MIN 0
//GENERAMOS LAS OPCIONES DE FRECUENCIA PARA EL TIMEOUT DEL DAC
typedef enum {
  FS_1 = 0,
  FS_10,
  FS_100,
  FS_1K,
 FS_2K,
  FS_SIZE	//TAMAÑO TOTAL
}fs_t;

static fs_t select_fs = FS_1; //INICIALIZO EN 1HZ, ESTA LUEGO SE LLAMARA EN DAC PARA CAMBIAR SU FREC

static const uint32_t DAC_frec[FS_SIZE] ={  //EL NUMERO LUEGO HAY QUE CALCULARLO PARA CADA FREC CORRESPONDIENTE
	65445,   //TICKS -> 1Hz
  6545,		//TICKS -> 10Hz
  654,		//TICKS -> 100Hz
  65,			//TICKS -> 1kHz
  33				//TICKS -> 2KHz
};

//GENERAMOS LAS OPCIONES DE FORMA DE ONDA
typedef enum {
  CUADRADA = 0,
  TRIANGULAR,
  SENOIDAL,
 // DIENTE_SIERRA,
  CANT_WF	//TAMAÑO TOTAL
}wf_t;

static wf_t select_wf = CUADRADA; //INICIALIZO CON LA FORMA DE ONDA CUADRADA

//VARIABLES GLOBALES:
volatile uint32_t *dac_samples_0 = (volatile uint32_t *)DAC_BUFFER_START_0;
volatile uint32_t *dac_samples_1 = (volatile uint32_t *)DAC_BUFFER_START_1;
volatile uint32_t *dac_samples_2 = (volatile uint32_t *)DAC_BUFFER_START_2;

uint32_t adc_value_CH0;

uint16_t triangular[WAVEFORM_SIZE];		//VECTORES CON FORMAS DE ONDAS
uint16_t cuadrada[WAVEFORM_SIZE];
uint16_t senoidal[WAVEFORM_SIZE] = {
	    512, 520, 528, 537, 545, 554, 562, 570, 579, 587, 595, 604, 612, 620, 628, 636,
	    645, 653, 661, 669, 677, 685, 693, 700, 708, 716, 724, 731, 739, 746, 754, 761,
	    768, 776, 783, 790, 797, 804, 811, 818, 824, 831, 837, 844, 850, 856, 863, 869,
	    875, 881, 886, 892, 898, 903, 908, 914, 919, 924, 929, 934, 938, 943, 947, 952,
	    956, 960, 964, 968, 972, 975, 979, 982, 985, 988, 991, 994, 997, 1000, 1002, 1004,
	    1006, 1009, 1010, 1012, 1014, 1015, 1017, 1018, 1019, 1020, 1021, 1022, 1022, 1023, 1023, 1023,
	    1023, 1023, 1023, 1022, 1022, 1021, 1020, 1019, 1018, 1017, 1015, 1014, 1012, 1010, 1009, 1006,
	    1004, 1002, 1000, 997, 994, 991, 988, 985, 982, 979, 975, 972, 968, 964, 960, 956,
	    952, 947, 943, 938, 934, 929, 924, 919, 914, 908, 903, 898, 892, 886, 881, 875,
	    869, 863, 856, 850, 844, 837, 831, 824, 818, 811, 804, 797, 790, 783, 776, 768,
	    761, 754, 746, 739, 731, 724, 716, 708, 700, 693, 685, 677, 669, 661, 653, 645,
	    636, 628, 620, 612, 604, 595, 587, 579, 570, 562, 554, 545, 537, 528, 520, 512,
	    503, 495, 486, 478, 469, 461, 453, 444, 436, 428, 419, 411, 403, 395, 387, 378,
	    370, 362, 354, 346, 338, 330, 323, 315, 307, 299, 292, 284, 277, 269, 262, 255,
	    247, 240, 233, 226, 219, 212, 205, 199, 192, 186, 179, 173, 167, 160, 154, 148,
	    142, 137, 131, 125, 120, 115, 109, 104, 99, 94, 89, 85, 80, 76, 71, 67,
	    63, 59, 55, 51, 48, 44, 41, 38, 35, 32, 29, 26, 23, 21, 19, 17,
	    14, 13, 11, 9, 8, 6, 5, 4, 3, 2, 1, 1, 0, 0, 0, 0,
	    0, 0, 1, 1, 2, 3, 4, 5, 6, 8, 9, 11, 13, 14, 17, 19,
	    21, 23, 26, 29, 32, 35, 38, 41, 44, 48, 51, 55, 59, 63, 67, 71,
	    76, 80, 85, 89, 94, 99, 104, 109, 115, 120, 125, 131, 137, 142, 148, 154,
	    160, 167, 173, 179, 186, 192, 199, 205, 212, 219, 226, 233, 240, 247, 255, 262,
	    269, 277, 284, 292, 299, 307, 315, 323, 330, 338, 346, 354, 362, 370, 378, 387,
	    395, 403, 411, 419, 428, 436, 444, 453, 461, 469, 478, 486, 495, 503
	};


//PROTOTIPO DE FUNCIONES:
void confPines(void);
void confIntExt(void);
void confTimer(void);
void confADC(void);
void confDAC(void);
void confGPDMA(wf_t select);
void saveWaveForm();
void gen_cuadrada(void);
void gen_triangular(void);
void genWaveForms(void);
void confUART(void);
void uart_send_uint16_ascii(uint16_t value);


//FUNCION PRINCIPAL:
int main()
{
  genWaveForms(); //GENERAMOS TODAS LAS FORMAS DE ONDAS
  saveWaveForm(); // LA QUE ESTA POR DEFECTO
  confPines();
  confIntExt();
  confADC();
  confDAC();
  confUART();
 // confGPDMA(select_wf);
  while(1)
  {
	adc_value_CH0 = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);		//GUARDO DATO EN adc_value_CH0
	uart_send_uint16_ascii(adc_value_CH0);

  }

  return 0;
}

/**
 * @brief Configuramos Funcionalidad de cada pin.
 *
 * @return void
 */
void confPines(void)
{
  //EINT0 -> SELECCION FORMA DE ONDA
  PINSEL_CFG_Type pinEINT0; //EINT 0 EN P2.10
	pinEINT0.Portnum = PINSEL_PORT_2;
	pinEINT0.Pinnum = PINSEL_PIN_10;
	pinEINT0.Funcnum = PINSEL_FUNC_1; //EINT0
	pinEINT0.Pinmode = PINSEL_PINMODE_PULLUP;//PULL-UP
	pinEINT0.OpenDrain = PINSEL_PINMODE_NORMAL; //NORMAL, SIN OPEN DRAIN
  PINSEL_ConfigPin(&pinEINT0);


  //EINT1 -> SELECCION DE FRECUENCIA
  PINSEL_CFG_Type pinEINT1; //EINT 1 EN P2.11
	pinEINT1.Portnum = PINSEL_PORT_2; //SELECCION DE PUERTO
	pinEINT1.Pinnum = PINSEL_PIN_11;
	pinEINT1.Funcnum = PINSEL_FUNC_1; //EINT0
	pinEINT1.Pinmode = PINSEL_PINMODE_PULLUP;//PULL-UP
	pinEINT1.OpenDrain = PINSEL_PINMODE_NORMAL; //NORMAL, SIN OPEN DRAIN
  PINSEL_ConfigPin(&pinEINT1);

  //EINT2 -> AUMENTA AMPLITUD (AMPLIFICA)
  PINSEL_CFG_Type pinEINT2; //EINT 2 EN P2.12
	pinEINT2.Portnum = PINSEL_PORT_2;
	pinEINT2.Pinnum = PINSEL_PIN_12;
	pinEINT2.Funcnum = PINSEL_FUNC_1; //EINT0
	pinEINT2.Pinmode = PINSEL_PINMODE_PULLUP;//PULL-UP
	pinEINT2.OpenDrain = PINSEL_PINMODE_NORMAL; //NORMAL, SIN OPEN DRAIN
  PINSEL_ConfigPin(&pinEINT2);

  //EINT3 -> BAJA AMPLITUD (ATENUA)
  PINSEL_CFG_Type pinEINT3; //EINT 2 EN P2.13
	pinEINT3.Portnum = PINSEL_PORT_0;
	pinEINT3.Pinnum = PINSEL_PIN_13;
	pinEINT3.Funcnum = PINSEL_FUNC_1; //EINT0
	pinEINT3.Pinmode = PINSEL_PINMODE_PULLUP;//PULL-UP
	pinEINT3.OpenDrain = PINSEL_PINMODE_NORMAL; //NORMAL, SIN OPEN DRAIN
  PINSEL_ConfigPin(&pinEINT3);

  //P0.27-> INC (FLANCO DE BAJADA)
  PINSEL_CFG_Type pinINC_pote = {0};
	pinINC_pote.Portnum = PINSEL_PORT_0;
	pinINC_pote.Pinnum = PINSEL_PIN_27;
	pinINC_pote.Funcnum = PINSEL_FUNC_0; //GPIO
	pinINC_pote.Pinmode = PINSEL_PINMODE_TRISTATE;// ES SALIDA
	pinINC_pote.OpenDrain = PINSEL_PINMODE_NORMAL; //NORMAL, SIN OPEN DRAIN
  PINSEL_ConfigPin(&pinINC_pote);
  GPIO_SetDir(PINSEL_PORT_0, PIN_27, OUTPUT); //GPIO COMO SALIDA


   //P0.28-> U/D (UP/DOWN)
  PINSEL_CFG_Type pinUD_pote = {0};
	pinUD_pote.Portnum = PINSEL_PORT_0;
	pinUD_pote.Pinnum = PINSEL_PIN_28;
	pinUD_pote.Funcnum = PINSEL_FUNC_0; //GPIO
	pinUD_pote.Pinmode = PINSEL_PINMODE_TRISTATE;// ES SALIDA
	pinUD_pote.OpenDrain = PINSEL_PINMODE_NORMAL; //NORMAL, SIN OPEN DRAIN
  PINSEL_ConfigPin(&pinUD_pote);
  GPIO_SetDir(PINSEL_PORT_0, PIN_28, OUTPUT); //GPIO COMO SALIDA


  //CONFIGURMAOS PIN DAC P0.26
	PINSEL_CFG_Type pinDAC = {0};
	pinDAC.Portnum = PINSEL_PORT_0;
	pinDAC.Pinnum = PINSEL_PIN_26;
	pinDAC.Funcnum = PINSEL_FUNC_2;
	pinDAC.Pinmode = PINSEL_PINMODE_TRISTATE; //PIN FLOTANTE, SIN PULL-UP NI PULL-DOWN
	pinDAC.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&pinDAC); //CARGAMOS LOS PARAMETROS DE LA ESTRUCTURA CON ESTO CONFIGRUAMOS EL PIN DEL DAC

	//CONFIGURAMOS PIN ADC A0.0 P0.23
	PINSEL_CFG_Type pinADC; // P0.23 AD0.0 FUNCION_1
	pinADC.Portnum = PINSEL_PORT_0;
	pinADC.Pinnum = PINSEL_PIN_23;
	pinADC.Funcnum = PINSEL_FUNC_1;
	pinADC.Pinmode = PINSEL_PINMODE_TRISTATE; //PIN FLOTANTE, SIN PULL-UP NI PULL-DOWN
	pinADC.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&pinADC);

  //CONFIGURAMOS TX2 P0.10
  PINSEL_CFG_Type  pinTXD2 = {0};
  pinTXD2.Portnum = PINSEL_PORT_0;
  pinTXD2.Pinnum = PINSEL_PIN_10;
  pinTXD2.Funcnum = PINSEL_FUNC_1;
  pinTXD2.Pinmode = PINSEL_PINMODE_TRISTATE; //PIN FLOTANTE, SIN PULL-UP NI PULL-DOWN, EL PERIFERICO LUEGO MANEJA EL NIVEL
  pinTXD2.OpenDrain = PINSEL_PINMODE_NORMAL;
  PINSEL_ConfigPin(&pinTXD2);

  //CONFIGURAMOS RX2 P0.11
  PINSEL_CFG_Type  pinRXD2 = {0};
  pinRXD2.Portnum = PINSEL_PORT_0;
  pinRXD2.Pinnum = PINSEL_PIN_11;
  pinRXD2.Funcnum = PINSEL_FUNC_1;
  pinRXD2.Pinmode = PINSEL_PINMODE_PULLUP; //PIN CON PULL-UPM MANTIENE UN VALOR ALTO CUANDO NO HAY TRANSMISION PARA EVITAR LECTURAS ERRATICAS O RUIDO CUANDO LA LINEA ESTA LIBRE
  pinRXD2.OpenDrain = PINSEL_PINMODE_NORMAL;
  PINSEL_ConfigPin(&pinRXD2);

  //

}

void confIntExt(void)
{
  //LIMPIO FLAGS
  EXTI_Init();
  //EINT0:
  EXTI_InitTypeDef pinEINT;
  pinEINT.EXTI_Line = EXTI_EINT0; //EINT0
  pinEINT.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
  pinEINT.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
  EXTI_Config(&pinEINT);

  //EINT1
  pinEINT.EXTI_Line = EXTI_EINT1;
  EXTI_Config(&pinEINT);

  //EINT2
  pinEINT.EXTI_Line = EXTI_EINT2;
  EXTI_Config(&pinEINT);

  //EINT2
   pinEINT.EXTI_Line = EXTI_EINT3;
   EXTI_Config(&pinEINT);
  //HABILITACION INTERRUPCION
  NVIC_EnableIRQ(EINT0_IRQn);
  NVIC_EnableIRQ(EINT1_IRQn);
  NVIC_EnableIRQ(EINT2_IRQn);
  NVIC_EnableIRQ(EINT3_IRQn);

}

void confADC(void)
{
  //ADC:
  ADC_Init(LPC_ADC, FREC_ADC); //ADC SAMPLES A 200KHz
  ADC_BurstCmd(LPC_ADC, ENABLE);	//CONVERSION EN RAFAGA O BURST ->CONVIERTE DE FORMA CONTINUA -<ACABA UNA CONVERSION Y SIGUE CON LA OTRA

  //CHANNEL 0:
  ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);	//HABILITAMOS EL CANAL0
 // ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE); //HABILITAMOS INTERRUPCION PARA CANAL0

  //NVIC_EnableIRQ(ADC_IRQn); //HABILITAMOS INTERRUPCION DE ADC PARA EL CORE

}

void confDAC()
{
  uint32_t countDAC = DAC_frec[select_fs]; //NUMERO DE CUENTAS PARA EL CONTADOR INTERNO DE 16 BITS DEL DAC (valor maximo 65535)->  2,65mS

	DAC_CONVERTER_CFG_Type confDAC = {0};
	confDAC.CNT_ENA = ENABLE;		//HABILITA EL CONTADOR DEL DAC, FUNCIONA DE TRIGGER PARA EL DMA TAMBIEN
	confDAC.DMA_ENA = ENABLE;		//HABILITO DMA PARA DAC
	confDAC.DBLBUF_ENA = DISABLE;	//NO SE USA --> YA QUE USO DMA

	DAC_ConfigDAConverterControl(LPC_DAC, &confDAC);

	DAC_Init(LPC_DAC);			//BIAS 700uA 1uS

	DAC_SetDMATimeOut(LPC_DAC, countDAC); //CARGO LA CUENTAS AL CONTADOR DEDICADO DEL DAC
}

void confGPDMA(wf_t select){

	GPDMA_Init();
	NVIC_DisableIRQ(DMA_IRQn);

	GPDMA_Channel_CFG_Type  conf_CH0_DAC={0};

	  //ALTERNATIVA PARA CAMBIO DE FORMA DE ONDA :D
	    switch(select)
	  {
	    case CUADRADA:
	    	static GPDMA_LLI_Type conf_LLI0_DAC;
	    	conf_LLI0_DAC.SrcAddr = (uint32_t)dac_samples_0;
	    	conf_LLI0_DAC.DstAddr = (uint32_t)&(LPC_DAC->DACR);
	    	conf_LLI0_DAC.NextLLI = (uint32_t)&conf_LLI0_DAC;
	    	conf_LLI0_DAC.Control = ((WAVEFORM_SIZE<<0)
	    							|(2<<18)
	    							|(2<<21)
	    							|(1<<26))
	    							&~(1<<27);
	    	conf_CH0_DAC.SrcMemAddr    = (uint32_t)dac_samples_0; //cambio*
	    	conf_CH0_DAC.DMALLI        = (uint32_t)&conf_LLI0_DAC;


	  	break;

	    case TRIANGULAR:
	    	static GPDMA_LLI_Type conf_LLI1_DAC;
	    	conf_LLI1_DAC.SrcAddr = (uint32_t)dac_samples_1;
	    	conf_LLI1_DAC.DstAddr = (uint32_t)&(LPC_DAC->DACR);
	    	conf_LLI1_DAC.NextLLI = (uint32_t)&conf_LLI1_DAC;
	    	conf_LLI1_DAC.Control = ((WAVEFORM_SIZE<<0)
	    							|(2<<18)
	    							|(2<<21)
	    							|(1<<26))
	    							&~(1<<27);
	    	conf_CH0_DAC.SrcMemAddr    = (uint32_t)dac_samples_1; //cambio*
	    	conf_CH0_DAC.DMALLI        = (uint32_t)&conf_LLI1_DAC;



	  	break;

	    case SENOIDAL:
	    	static GPDMA_LLI_Type conf_LLI2_DAC;
	    	conf_LLI2_DAC.SrcAddr = (uint32_t)dac_samples_2;
	    	conf_LLI2_DAC.DstAddr = (uint32_t)&(LPC_DAC->DACR);
	    	conf_LLI2_DAC.NextLLI = (uint32_t)&conf_LLI2_DAC;
	    	conf_LLI2_DAC.Control = ((WAVEFORM_SIZE<<0)
	    							|(2<<18)
	    							|(2<<21)
	    							|(1<<26))
	    							&~(1<<27);
	    	conf_CH0_DAC.SrcMemAddr    = (uint32_t)dac_samples_2; //cambio*
	    	conf_CH0_DAC.DMALLI        = (uint32_t)&conf_LLI2_DAC;


	  	break;

	  	default:
	  	break;
	  }

	conf_CH0_DAC.ChannelNum    = 0;
	conf_CH0_DAC.TransferSize  = WAVEFORM_SIZE;
	conf_CH0_DAC.TransferWidth = 0;
	conf_CH0_DAC.DstMemAddr    = 0;
	conf_CH0_DAC.TransferType  = GPDMA_TRANSFERTYPE_M2P;
	conf_CH0_DAC.SrcConn       = 0;
	conf_CH0_DAC.DstConn       = GPDMA_CONN_DAC;


    GPDMA_Setup(&conf_CH0_DAC);
  //AQUI FINALIZA EL CAMBIO
	GPDMA_ChannelCmd(0,ENABLE);
}

void confUART(void)
{
  UART_CFG_Type cfgUART_ADC = {0};
  UART_ConfigStructInit(&cfgUART_ADC);
  UART_Init(LPC_UART2, &cfgUART_ADC);
  UART_TxCmd(LPC_UART2, ENABLE);

  UART_FIFO_CFG_Type cfgUART_ADC_FIFO = {0};
  UART_FIFOConfigStructInit(&cfgUART_ADC_FIFO);
  UART_FIFOConfig(LPC_UART2, &cfgUART_ADC_FIFO);
}


void ADC_IRQHandler()	//CADA VEZ QUE TERMINA LA CONVERSION DE UN CANAL ENTRA
{
	//if(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)) //TERMINO DONE DE AD0.0?
	adc_value_CH0 = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);		//GUARDO DATO EN adc_value_CH0
	//UART_Send(LPC_UART2, (uint8_t *) &adc_value_CH0, sizeof(adc_value_CH0), BLOCKING); // ENVIO DE DATOS DEL ADC AL UART
	//AL USAR UART_Send() nos da un Hardfoult Handler (hay que ver su NVIC)
	 //uart_send_uint16_ascii(adc_value_CH0);

}

//INTERRUPCION PARA SELECCION DE FORMA DE ONDA
void EINT0_IRQHandler(){

  select_wf = (select_wf + 1)%CANT_WF; //BUFFER CIRCULAR DE FORMAS DE ONDAS
  confGPDMA(select_wf);

  EXTI_ClearEXTIFlag(EXTI_EINT0);
}

//INTERRUPCION PARA SELECCION DE FRECUENCIAS
void EINT1_IRQHandler(){
 select_fs = (select_fs + 1) % FS_SIZE;  //BUFFFER CIRCULAR DE FRECUENCIAS
 confDAC();
  EXTI_ClearEXTIFlag(EXTI_EINT1);
}

//INTERRUPCION PARA INCREMENTO DE AMPLITUD EN POTENCIOMETRO --> U/D= 1  INC=  ---__
void EINT2_IRQHandler(){
  GPIO_SetValue(PORT_0, PIN_27);	//INC ALTO
  GPIO_SetValue(PORT_0, PIN_28); // U/D -> 1

  EXTI_ClearEXTIFlag(EXTI_EINT2);

  GPIO_ClearValue(PORT_0, PIN_27);	//INC BAJO

}

//INTERRUPCION PARA DECREMENTO DE AMPLITUD EN POTENCIOMETRO --> U/D= 0  INC=  ---__
void EINT3_IRQHandler(){

  GPIO_SetValue(PORT_0, PIN_27);	//INC ALTO
  GPIO_ClearValue(PORT_0, PIN_28); // U/D -> 0

  EXTI_ClearEXTIFlag(EXTI_EINT3);

  GPIO_ClearValue(PORT_0, PIN_27);	//INC BAJO

}


void gen_triangular()
{
  uint16_t index=0;
  //CUADRANTE 1:
  for(uint16_t i=0; i<CUADRANTE; i++)
  {
    triangular[index++] = DAC_OFFSET + (DAC_MAX - DAC_OFFSET) * ((float) i/ (CUADRANTE-1));
  }
  //CUADRANTE 2:
  for(uint16_t i=0; i<CUADRANTE; i++)
  {
    triangular[index++] = DAC_MAX - (DAC_MAX - DAC_OFFSET) * ((float) i/ (CUADRANTE-1));
  }
  //CUADRANTE 3:
  for(uint16_t i=0; i<CUADRANTE; i++)
  {
    triangular[index++] = DAC_OFFSET - (DAC_OFFSET - DAC_MIN) * ((float) i/ (CUADRANTE-1));
  }
  //CUADRANTE 4:
  for(uint16_t i=0; index<WAVEFORM_SIZE; i++)
  {
    triangular[index++] = DAC_MIN + (DAC_OFFSET - DAC_MIN) * ((float) i/ (CUADRANTE-1));
  }

}

void gen_cuadrada()
{
  for(uint16_t i=0; i<WAVEFORM_SIZE_HALF; i++)
  {
    cuadrada[i] = DAC_MAX;
  }

  for(uint16_t i=WAVEFORM_SIZE_HALF; i<WAVEFORM_SIZE; i++)
  {
    cuadrada[i] = DAC_MIN;
  }
}

void genWaveForms ()
{
  gen_cuadrada();
  gen_triangular();
}
void saveWaveForm(){
  volatile uint32_t *index = dac_samples_0; //INICIO DE DE GUARDADO DE FORMA DE ONDA CUADRADA

  		for(uint32_t i=0; i<WAVEFORM_SIZE; i++)
      {
        *index = cuadrada[i]<<6;
        index++;
      }

  	index = dac_samples_1; //INICIO DE DE GUARDADO DE FORMA DE ONDA TRIANGULAR

  		for(uint32_t i=0; i<WAVEFORM_SIZE; i++)
      {
        *index = triangular[i]<<6;
        index++;
      }

  	index = dac_samples_2; //INICIO DE DE GUARDADO DE FORMA DE ONDA SENOIDAL

  		for(uint32_t i=0; i<WAVEFORM_SIZE; i++)
      {
        *index = senoidal[i]<<6;
        index++;
      }
}

void uart_send_uint16_ascii(uint16_t value)
{
    char buffer[7];   // ahora necesitas espacio para CR+LF
    int i = 0;

    if(value >= 1000) buffer[i++] = '0' + (value / 1000) % 10;
    if(value >= 100)  buffer[i++] = '0' + (value / 100) % 10;
    if(value >= 10)   buffer[i++] = '0' + (value / 10) % 10;

    buffer[i++] = '0' + (value % 10);

    buffer[i++] = '\r';   // ← necesario para muchos osciloscopios
    buffer[i++] = '\n';   // ← salto de línea

    UART_Send(LPC_UART2, (uint8_t*)buffer, i, NONE_BLOCKING);
}
