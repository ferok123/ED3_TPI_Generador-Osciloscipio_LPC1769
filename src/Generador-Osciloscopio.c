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

//VARIABLES GLOBALES:

//PROTOTIPO DE FUNCIONES:
void confPines(void);
void confTimer(void);
void confADC(void);
void confDAC(void);

//FUNCION PRINCIPAL:
int main()
{

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


  //EINT0 -> SELECCION DE AMPLITUD
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
