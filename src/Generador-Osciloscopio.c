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

//DEFINICION DE MACROS Y CONSTANTES:
#define FREC_ADC 200000
//GENERAMOS LAS OPCIONES DE FRECUENCIA PARA EL TIMEOUT DEL DAC
typedef enum {
  FS_1 = 0,
  FS_10,
  FS_100,
  FS_1K,
  FS_10K,
  FS_SIZE	//TAMAÑO TOTAL
}fs_t;

static fs_t select_fs = FS_1; //INICIALIZO EN 1HZ, ESTA LUEGO SE LLAMARA EN DAC PARA CAMBIAR SU FREC

static const uint32_t DAC_frec[FS_SIZE] ={  //EL NUMERO LUEGO HAY QUE CALCULARLO PARA CADA FREC CORRESPONDIENTE
	65535,   //TICKS -> 1Hz
  6553,		//TICKS -> 10Hz
  655,		//TICKS -> 100Hz
  65,			//TICKS -> 1kHz
  6				//TICKS -> 10KHz
};

//VARIABLES GLOBALES:
uint16_t adc_value_CH0 = 0;

//PROTOTIPO DE FUNCIONES:
void confPines(void);
void confIntExt(void);
void confTimer(void);
void confADC(void);
void confDAC(void);

//FUNCION PRINCIPAL:
int main()
{
  confPines();
	confIntExt();
  confADC();
	confDAC();
  while(1)
  {
    
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

  
  //EINT1 -> SELECCION DE AMPLITUD
  PINSEL_CFG_Type pinEINT1; //EINT 1 EN P2.11
	pinEINT1.Portnum = PINSEL_PORT_2; //SELECCION DE PUERTO
	pinEINT1.Pinnum = PINSEL_PIN_11;
	pinEINT1.Funcnum = PINSEL_FUNC_1; //EINT0
	pinEINT1.Pinmode = PINSEL_PINMODE_PULLUP;//PULL-UP
	pinEINT1.OpenDrain = PINSEL_PINMODE_NORMAL; //NORMAL, SIN OPEN DRAIN
  PINSEL_ConfigPin(&pinEINT1);
  
  //EINT2 -> SELECCION DE BASE DE TIEMPO
  PINSEL_CFG_Type pinEINT2; //EINT 2 EN P2.12
	pinEINT2.Portnum = PINSEL_PORT_2;
	pinEINT2.Pinnum = PINSEL_PIN_12;
	pinEINT2.Funcnum = PINSEL_FUNC_1; //EINT0
	pinEINT2.Pinmode = PINSEL_PINMODE_PULLUP;//PULL-UP
	pinEINT2.OpenDrain = PINSEL_PINMODE_NORMAL; //NORMAL, SIN OPEN DRAIN
  PINSEL_ConfigPin(&pinEINT2);
  

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
  
}

void confIntExt(void)
{
  //LIMPIO FLAGS
  EXTI_Init();
  //EINT0:
  EXTI_InitTypedef pinEINT;
  pinEINT.EXTI_Line = EXTI_EINT0; //EINT0
  pinEINT.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
  pinEINT.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
  EXTI_Confg(&pinEINT);
  
  //EINT1
  pinEINT.EXTI_Line = EXTI_EINT1;
  EXTI_Confg(&pinEINT);
  
  //EINT2
  pinEINT.EXTI_Line = EXTI_EINT2;
  EXTI_Confg(&pinEINT);
  
  //HABILITACION INTERRUPCION
  NVIC_EnableIRQ(EINT0_IRQn);
  NVIC_EnableIRQ(EINT1_IRQn);
  NVIC_EnableIRQ(EINT2_IRQn);
}

void confADC(void)
{
  //ADC:
  ADC_Init(LPC_ADC, FREC_ADC); //ADC SAMPLES A 200KHz
  ADC_BurstCmd(LPC_ADC, ENABLE);	//CONVERSION EN RAFAGA O BURST ->CONVIERTE DE FORMA CONTINUA -<ACABA UNA CONVERSION Y SIGUE CON LA OTRA
  
  //CHANNEL 0:
  ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);	//HABILITAMOS EL CANAL0
  ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE); //HABILITAMOS INTERRUPCION PARA CANAL0  
  
	NVIC_EnableIRQ(ADC_IRQn); //HABILITAMOS INTERRUPCION DE ADC PARA EL CORE
}

void confDAC()
{
  uint32_t countDAC = DAC_frec[select_fs]; //NUMERO DE CUENTAS PARA EL CONTADOR INTERNO DE 16 BITS DEL DAC (valor maximo 65535)->  2,65mS

	DAC_CONVERTER_CFG_Type confDAC = {0};
	confDAC.CNT_ENA = ENABLE;		//HABILITA EL CONTADOR DEL DAC, FUNCIONA DE TRIGGER PARA EL DMA TAMBIEN
	confDAC.DMA_ENA = ENABLE;		//HABILITO DMA PARA DAC
	confDAC.DBLBUF_ENA = DISABLE;	//NO SE USA --> YA QUE USO DMA

	DAC_Init(LPC_DAC);			//BIAS 700uA 1uS

	DAC_ConfigDAConverterControl(LPC_DAC, &confDAC);

	DAC_Init(LPC_DAC);			//BIAS 700uA 1uS

	DAC_SetDMATimeOut(LPC_DAC, countDAC); //CARGO LA CUENTAS AL CONTADOR DEDICADO DEL DAC
}

void ADC_IRQHandler()	//CADA VEZ QUE TERMINA LA CONVERSION DE UN CANAL ENTRA
{
	if(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)) //TERMINO DONE DE AD0.0?
		adc_value_CH0 = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);		//GUARDO DATO EN adc_value_CH0
}

//INTERRUPCION PARA SELECCION DE LA FORMA DE ONDA
void EINT0_IRQHandler(){
  
  EXTI_ClearEXTIFlag(EXTI_EINT0);
}
//INTERRUPCION PARA SELECCION DE LA AMPLITUD
void EINT1_IRQHandler(){
  
  EXTI_ClearEXTIFlag(EXTI_EINT1);
}
//INTERRUPCION PARA SELECCION DE LA BASE DE TIEMPO
void EINT2_IRQHandler(){
  
  select_fs = (select_fs + 1) % FS_SIZE;  //BUFFFER CIRCULAR DE FRECUENCIAS
  confDAC();
  
  EXTI_ClearEXTIFlag(EXTI_EINT2);
}


