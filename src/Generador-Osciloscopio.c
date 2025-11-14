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
#define DAC_BUFFER_START 0X2007C000
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

//GENERAMOS LAS OPCIONES DE FORMA DE ONDA
typedef enum {
  CUADRADA = 0,
  TRIANGULAR,
  SENOIDAL,
  DIENTE_SIERRA,
  CANT_WF	//TAMAÑO TOTAL
}wf_t;

static wf_t select_wf = SENOIDAL; //INICIALIZO CON LA FORMA DE ONDA CUADRADA

//VARIABLES GLOBALES:
volatile uint32_t *dac_samples = (volatile uint32_t *)DAC_BUFFER_START;
uint16_t adc_value_CH0 = 0;

uint16_t triangular[WAVEFORM_SIZE];		//VECTORES CON FORMAS DE ONDAS
uint16_t cuadrada[WAVEFORM_SIZE];
uint16_t senoidal[WAVEFORM_SIZE] = {
		512, 520, 528, 536, 544, 552, 560, 568, 576, 584,
		592, 600, 608, 616, 624, 632, 640, 648, 656, 664,
		672, 680, 688, 696, 703, 711, 718, 726, 733, 740,
		748, 755, 762, 769, 776, 782, 789, 795, 801, 807,
		813, 819, 824, 830, 835, 840, 845, 850, 855, 859,
		864, 868, 872, 875, 879, 882, 885, 888, 891, 893,
		895, 897, 899, 901, 902, 903, 904, 904, 905, 904,
		904, 903, 902, 901, 899, 897, 895, 893, 891, 888,
		885, 882, 879, 875, 872, 868, 864, 859, 855, 850,
		845, 840, 835, 830, 824, 819, 813, 807, 801, 795,
		789, 782, 776, 769, 762, 755, 748, 740, 733, 726,
		718, 711, 703, 696, 688, 680, 672, 664, 656, 648,
		640, 632, 624, 616, 608, 600, 592, 584, 576, 568,
		560, 552, 544, 536, 528, 520, 512, 504, 496, 488,
		480, 472, 464, 456, 448, 440, 432, 424, 416, 408,
		400, 392, 384, 376, 369, 361, 353, 346, 338, 330,
		323, 315, 308, 300, 293, 286, 279, 272, 265, 259,
		252, 246, 240, 234, 229, 223, 218, 213, 207, 203,
		198, 193, 188, 184, 180, 176, 172, 168, 165, 161,
		158, 155, 152, 150, 147, 145, 143, 141, 139, 138,
		137, 136, 136, 135, 135, 135, 136, 136, 137, 138,
		139, 141, 143, 145, 147, 150, 152, 155, 158, 161,
		165, 168, 172, 176, 180, 184, 188, 193, 198, 203,
		207, 213, 218, 223, 229, 234, 240, 246, 252, 259,
		265, 272, 279, 286, 293, 300, 308, 315, 323, 330,
		338, 346, 353, 361, 369, 376, 384, 392, 400, 408,
		416, 424, 432, 440, 448, 456, 464, 472, 480, 488,
		496, 504, 512
		};


//PROTOTIPO DE FUNCIONES:
void confPines(void);
void confIntExt(void);
void confTimer(void);
void confADC(void);
void confDAC(void);
void confDAC_OFF(void);
void confGPDMA(void);
void saveWaveForm(wf_t select);
void gen_cuadrada(void);
void gen_triangular(void);
void genWaveForms(void);

//FUNCION PRINCIPAL:
int main()
{
  genWaveForms(); //GENERAMOS TODAS LAS FORMAS DE ONDAS
  saveWaveForm(select_wf); // LA QUE ESTA POR DEFECTO
  confPines();
  confIntExt();
  confADC();
  confDAC();
  confGPDMA();
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

  //NVIC_EnableIRQ(ADC_IRQn); //HABILITAMOS INTERRUPCION DE ADC PARA EL CORE
}

void confDAC()
{
  uint32_t countDAC = 65535; //NUMERO DE CUENTAS PARA EL CONTADOR INTERNO DE 16 BITS DEL DAC (valor maximo 65535)->  2,65mS

	DAC_CONVERTER_CFG_Type confDAC = {0};
	confDAC.CNT_ENA = ENABLE;		//HABILITA EL CONTADOR DEL DAC, FUNCIONA DE TRIGGER PARA EL DMA TAMBIEN
	confDAC.DMA_ENA = ENABLE;		//HABILITO DMA PARA DAC
	confDAC.DBLBUF_ENA = DISABLE;	//NO SE USA --> YA QUE USO DMA

	DAC_ConfigDAConverterControl(LPC_DAC, &confDAC);

	DAC_Init(LPC_DAC);			//BIAS 700uA 1uS

	DAC_SetDMATimeOut(LPC_DAC, countDAC); //CARGO LA CUENTAS AL CONTADOR DEDICADO DEL DAC
}

void confDAC_OFF()
{
	DAC_CONVERTER_CFG_Type confDAC = {0};
	confDAC.CNT_ENA = DISABLE;		//HABILITA EL CONTADOR DEL DAC, FUNCIONA DE TRIGGER PARA EL DMA TAMBIEN
	confDAC.DMA_ENA = DISABLE;		//HABILITO DMA PARA DAC
	confDAC.DBLBUF_ENA = DISABLE;	//NO SE USA --> YA QUE USO DMA

	DAC_ConfigDAConverterControl(LPC_DAC, &confDAC);
}

void confGPDMA(void){
	GPDMA_Init();
	NVIC_DisableIRQ(DMA_IRQn);
	GPDMA_LLI_Type conf_LLI0_DAC;
	conf_LLI0_DAC.SrcAddr = (uint32_t)dac_samples;
	conf_LLI0_DAC.DstAddr = (uint32_t)&(LPC_DAC->DACR);
	conf_LLI0_DAC.NextLLI = (uint32_t)&conf_LLI0_DAC;
	conf_LLI0_DAC.Control = ((WAVEFORM_SIZE<<0)
							|(2<<18)
							|(2<<21)
							|(1<<26))
							&~(1<<27);

	GPDMA_Channel_CFG_Type  conf_CH0_DAC={0};
	conf_CH0_DAC.ChannelNum    = 0;
	conf_CH0_DAC.TransferSize  = WAVEFORM_SIZE;
	conf_CH0_DAC.TransferWidth = 0;
	conf_CH0_DAC.SrcMemAddr    = (uint32_t)dac_samples;
	conf_CH0_DAC.DstMemAddr    = 0;
	conf_CH0_DAC.TransferType  = GPDMA_TRANSFERTYPE_M2P;
	conf_CH0_DAC.SrcConn       = 0;
	conf_CH0_DAC.DstConn       = GPDMA_CONN_DAC;
	conf_CH0_DAC.DMALLI        = (uint32_t)&conf_LLI0_DAC;
	GPDMA_Setup(&conf_CH0_DAC);
	GPDMA_ChannelCmd(0,ENABLE);
}


void ADC_IRQHandler()	//CADA VEZ QUE TERMINA LA CONVERSION DE UN CANAL ENTRA
{
	if(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)) //TERMINO DONE DE AD0.0?
	adc_value_CH0 = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);		//GUARDO DATO EN adc_value_CH0
}

void EINT0_IRQHandler(){

  GPDMA_ChannelCmd(0,DISABLE);
  confDAC_OFF();

  select_wf = (select_wf + 1)%CANT_WF; //BUFFER CIRCULAR DE FORMAS DE ONDAS
  saveWaveForm(select_wf);

  confDAC();
  confGPDMA();

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
void saveWaveForm(wf_t select)
{
  volatile uint32_t *index = dac_samples; //INICIO DE DE GUARDADO DE FORMA DE ONDA
  switch(select)
  {
    case CUADRADA:
  		for(uint32_t i=0; i<WAVEFORM_SIZE; i++)
      {
        *index = cuadrada[i]<<6;
        index++;
      }
  	break;

    case TRIANGULAR:
  		for(uint32_t i=0; i<WAVEFORM_SIZE; i++)
      {
        *index = triangular[i]<<6;
        index++;
      }
  	break;

    case SENOIDAL:
  		for(uint32_t i=0; i<WAVEFORM_SIZE; i++)
      {
        *index = senoidal[i]<<6;
        index++;
      }
  	break;

  	default:
  	break;
  }
}
