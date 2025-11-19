/****************************************************
 * @brief   Enciende el LED conectado al pin P0.22
 *          cuando se recibe el byte 0x01 por UART2
 *          y lo apaga para cualquier otro valor.
 *
 * @details Configura el pin P0.22 como salida digital
 *          y el periférico UART2 para comunicación
 *          serial a 9600 baudios.
 ****************************************************/

/* Inclusiones */
#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_uart.h"
#include <lpc17xx_gpio.h>

/* Defines */
#define OUTPUT   (uint8_t)1
#define INPUT    (uint8_t)0
#define PIN_22   (uint32_t)(1<<22)
#define PORT_0   (uint8_t)0

/* Variables globales */
uint8_t send_flag = 0;
uint8_t buffer[5] = {0x48, 0x4f, 0x4c, 0x41,0x0A};

/* Prototipos */
void cfgGPIO(void);
void cfgUART(void);

/* Función principal */
int main(void)
{
    cfgGPIO();
    cfgUART();

    while(1)
    {        if(send_flag == 1)
        {
            UART_Send(LPC_UART2, buffer, sizeof(buffer), BLOCKING);  //ENVIA LOS DATOS EN EL BUFFER (VECTOR)
            send_flag = 0;
        }
    }

    return 0;
}

/* Configuración de puertos */
void cfgGPIO(void)
{
    PINSEL_CFG_Type cfgPinLED;
    PINSEL_CFG_Type cfgPinTXD2;
    PINSEL_CFG_Type cfgPinRXD2;

    cfgPinLED.Portnum   = PINSEL_PORT_0;
    cfgPinLED.Pinnum    = PINSEL_PIN_22;
    cfgPinLED.Funcnum   = PINSEL_FUNC_0;
    cfgPinLED.OpenDrain = PINSEL_PINMODE_NORMAL;

    cfgPinTXD2.Portnum   = PINSEL_PORT_0;
    cfgPinTXD2.Pinnum    = PINSEL_PIN_10;
    cfgPinTXD2.Funcnum   = PINSEL_FUNC_1;
    cfgPinTXD2.OpenDrain = PINSEL_PINMODE_NORMAL;

    cfgPinRXD2.Portnum   = PINSEL_PORT_0;
    cfgPinRXD2.Pinnum    = PINSEL_PIN_11;
    cfgPinRXD2.Funcnum   = PINSEL_FUNC_1;
    cfgPinRXD2.OpenDrain = PINSEL_PINMODE_NORMAL;

    PINSEL_ConfigPin(&cfgPinLED);
    PINSEL_ConfigPin(&cfgPinTXD2);
    PINSEL_ConfigPin(&cfgPinRXD2);

    GPIO_SetDir(PORT_0, PIN_22, OUTPUT);
    GPIO_ClearValue(PORT_0, PIN_22);
}

/* Configuración UART */
void cfgUART(void)
{
    UART_CFG_Type cfgPinUART2;
    UART_FIFO_CFG_Type cfgUART2FIFO;

    UART_ConfigStructInit(&cfgPinUART2);
    UART_Init(LPC_UART2, &cfgPinUART2);

    UART_FIFOConfigStructInit(&cfgUART2FIFO);
    UART_FIFOConfig(LPC_UART2, &cfgUART2FIFO);

    UART_IntConfig(LPC_UART2, UART_INTCFG_RBR, ENABLE);
    UART_TxCmd(LPC_UART2, ENABLE);

    NVIC_EnableIRQ(UART2_IRQn);
}

/* Rutina de interrupción UART2 */
void UART2_IRQHandler(void)
{
    uint8_t receivedData;

    if(UART_GetLineStatus(LPC_UART2) & UART_LSR_RDR)
    {
        receivedData = UART_ReceiveByte(LPC_UART2);

        if(receivedData == 0x31)            //PREGUNTA POR CARACTER PRESIONADO EN ESTE CASO "1" CAMBIO FLAG 
        {
            GPIO_SetValue(PORT_0, PIN_22);
            send_flag = 1;
        }
        else
        {
            GPIO_ClearValue(PORT_0, PIN_22);
        }
    }
}
