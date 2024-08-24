/**
 *  CH32VX USB Serial Library
 *
 *  Copyright (c) 2024 Florian Korotschenko aka KingKoro
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  file         : ch32v_usb_serial.h
 *  description  : usb serial library main header
 *
 */

#ifndef __CH32V_USB_SERIAL_H
#define __CH32V_USB_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "stdio.h"
#include "string.h"
#include "debug.h"
#include "stdlib.h"
#include "stdatomic.h"
#include "stdarg.h"  // For handling variadic arguments


#if defined(CH32V10X)
    #include <hal/CH32V10X/USB-Driver/ch32v10x_usbfs_device.h>
    #include "ch32v10x_conf.h"
#elif defined(CH32V20X)
    #include "hal/CH32V20X/USB-Driver/usb_lib.h"
    #include "hal/CH32V20X/CONFIG/usb_desc.h"
    #include "hal/CH32V20X/CONFIG/usb_prop.h"
#elif defined(CH32V30X)
    #include "ch32v30x_conf.h"
#elif defined(CH32X035) || defined(CH32X033)
    #include <hal/CH32X03X/USB-Driver/ch32x035_usbfs_device.h>
    #include "ch32x035_conf.h"
#endif


// Specifics:
//      CH32V203:   Timer: TIM2,    USB-Port: USBD/USBFS
//      CH32X035:   Timer: TIM3,    USB-Port: USBFS
//      CH32V103:   Timer: TIM2,    USB-Port: USBFS
//      CH32V307:   Timer: TIM2,    USB-Port: USBFS/USBHS

// USB Port selection for Serial Printing
#define CDC_USBD            1   //Use USB-D-Port (V203: PA11 + PA12) USB-HS-Port (V307: PB6 + PB7)
#define CDC_USBFS           2   //Use USB-FS-Port (V203: PB6 + PB7) (X035: PC16 + PC17) (V103: PA11 + PA12) (V307: PA11 +  PA12)
// USB Write Mode selection
#define USB_TX_SYNC         1   //Synchronous USB TX Mode: Printf() returns only after usb bus available and data was sent 
                                //-> Can impact performance, when serial console is unconnected on host, every Printf() causes calling code to wait up to DEF_UARTx_USB_UP_TIMEOUT * 100uS
#define USB_TX_ASYNC        2   //Asynchronous USB TX Mode: Printf() returns immediatly, data is sent by periodically calling USB_Tx_runner() in endless while-loop
                                //-> Can cause dataloss, if USB_Tx_runner() is not called often enough to process USB_Tx_async_buffer, oldest data is omitted, generally higher memory consumption in async mode
// USB Receive Overflow handling
#define USB_RX_HALT         1   //If USB RX buffer soon to be full (e.g. not being processed), stop USB RX accepts
#define USB_RX_OVERWRITE    2   //If USB RX buffer soon to be full, do not care, wrap around buffer and overwrite (always fresh data in)
                                //-> WARNING: USB_RX_OVERWRITE can cause MCU to hang up if there is too much input traffic at once!

/* ++++++++++++++++++++ USER CONFIG AREA BEGIN ++++++++++++++++++++ */

#define USB_CDCPORT         CDC_USBFS            /* Select USB Port here (CDC_USBD or CDC_USBFS) */
#define USB_CDC_BAUDRATE    1000000              /* Select USB Port Baudrate here (For very high speeds >500KBaud/s, consider upping speed of timer TIM2 or any other interrupt, invoking USB_Tx_runner() ) */
#define USB_CDC_STOPBIT     0                   /* Select USB Port Stopbit here */
#define USB_CDC_PARITY      0                   /* Select USB Port Paritybit here */
#define USB_TX_MODE         USB_TX_ASYNC        /* Select USB Port Send Mode here */
#define USB_RX_OVERFLOW     USB_RX_HALT         /* Select USB Port Receive Overflow Mode here */   
#define GETCH_CLI_FEEDBACK                      /* Wether to display feedback into console when typing in */
#define USB_SCANF_BUF_SIZE  512                 /* Maximum USB RX buffer size for scanf() before matching */
//#define EXT_USB_TIM_HANDLER


/* ++++++++++++++++++++ USER CONFIG AREA END ++++++++++++++++++++ */

#ifndef USB_CDCPORT
    #define USB_CDCPORT CDC_USBD
#endif

#if(USB_CDCPORT == CDC_USBFS)
    #if defined(CH32V10X)
        #include "hal/CH32V10X/USB-Driver/ch32v10x_usbfs_device.h"
    #elif defined(CH32V20X)
        #include "hal/CH32V20X/USB-Driver/ch32v20x_usbfs_device.h"
        #include "ch32v20x_conf.h"
        typedef struct
    {
        __IO uint8_t  BASE_CTRL;
        __IO uint8_t  UDEV_CTRL;
        __IO uint8_t  INT_EN;
        __IO uint8_t  DEV_ADDR;
        __IO uint8_t  Reserve0;
        __IO uint8_t  MIS_ST;
        __IO uint8_t  INT_FG;
        __IO uint8_t  INT_ST;
        __IO uint32_t RX_LEN;
        __IO uint8_t  UEP4_1_MOD;
        __IO uint8_t  UEP2_3_MOD;
        __IO uint8_t  UEP5_6_MOD;
        __IO uint8_t  UEP7_MOD;
        __IO uint32_t UEP0_DMA;
        __IO uint32_t UEP1_DMA;
        __IO uint32_t UEP2_DMA;
        __IO uint32_t UEP3_DMA;
        __IO uint32_t UEP4_DMA;
        __IO uint32_t UEP5_DMA;
        __IO uint32_t UEP6_DMA;
        __IO uint32_t UEP7_DMA;
        __IO uint16_t UEP0_TX_LEN;
        __IO uint8_t  UEP0_TX_CTRL;
        __IO uint8_t  UEP0_RX_CTRL;
        __IO uint16_t UEP1_TX_LEN;
        __IO uint8_t  UEP1_TX_CTRL;
        __IO uint8_t  UEP1_RX_CTRL;
        __IO uint16_t UEP2_TX_LEN;
        __IO uint8_t  UEP2_TX_CTRL;
        __IO uint8_t  UEP2_RX_CTRL;
        __IO uint16_t UEP3_TX_LEN;
        __IO uint8_t  UEP3_TX_CTRL;
        __IO uint8_t  UEP3_RX_CTRL;
        __IO uint16_t UEP4_TX_LEN;
        __IO uint8_t  UEP4_TX_CTRL;
        __IO uint8_t  UEP4_RX_CTRL;
        __IO uint16_t UEP5_TX_LEN;
        __IO uint8_t  UEP5_TX_CTRL;
        __IO uint8_t  UEP5_RX_CTRL;
        __IO uint16_t UEP6_TX_LEN;
        __IO uint8_t  UEP6_TX_CTRL;
        __IO uint8_t  UEP6_RX_CTRL;
        __IO uint16_t UEP7_TX_LEN;
        __IO uint8_t  UEP7_TX_CTRL;
        __IO uint8_t  UEP7_RX_CTRL;
        __IO uint32_t Reserve1;
        __IO uint32_t OTG_CR;
        __IO uint32_t OTG_SR;
    } USBFSD_TypeDef;
        /* USBFS Registers */
        #define USBFSD                                  ((USBFSD_TypeDef *)USBFS_BASE)
        #define USBFS_IRQn  59              /* USBFS global Interrupt                               */
        #define USBFSWakeUp_IRQn = 60       /* USB Host/Device WakeUp Interrupt                     */
        
    #elif defined(CH32V30X)
        #include <hal/CH32V30X/USB-Driver/ch32v30x_usbfs_device.h>
    #elif defined(CH32X035) || defined(CH32X033)
        #include "hal/CH32X03X/USB-Driver/ch32x035_usbfs_device.h"
    #endif
#elif(USB_CDCPORT == CDC_USBD)
    #if defined(CH32V30X)
        #include <hal/CH32V30X/USB-Driver/ch32v30x_usbhs_device.h>
    #endif
#endif

/******************************************************************************/
/* Related macro definitions */
/* Serial buffer related definitions */
#define DEF_UARTx_RX_BUF_LEN       ( 4 * 512 )                                  /* Serial x receive buffer size */
#define DEF_UARTx_TX_BUF_LEN       ( 2 * 512 )                                  /* Serial x transmit buffer size */
#if(USB_CDCPORT == CDC_USBFS)
    #if defined(CH32X035) || defined(CH32X033) || defined(CH32V10X)
        #define DEF_USB_FS_PACK_LEN        DEF_USBD_FS_PACK_SIZE                        /* USB full speed mode packet size for serial x data */
        #define DEF_USB_HS_PACK_LEN        DEF_USBD_HS_PACK_SIZE                        /* USB high speed mode packet size for serial x data */
    #elif defined(CH32V30X)
        #define DEF_USB_FS_PACK_LEN        DEF_USBD_FS_PACK_SIZE                        /* USB full speed mode packet size for serial x data */
        #define DEF_USB_HS_PACK_LEN        DEF_USBD_HS_PACK_SIZE                        /* USB high speed mode packet size for serial x data */
    #elif defined(CH32V20X)
        #define DEF_USB_FS_PACK_LEN        64                                               /* USB full speed mode packet size for serial x data */
    #endif
#elif(USB_CDCPORT == CDC_USBD)
    #if defined(CH32V20X)
        #define DEF_USB_FS_PACK_LEN        64                                           /* USB full speed mode packet size for serial x data */
    #elif defined(CH32V30X)
        #define DEF_USB_FS_PACK_LEN        64                                           /* USB full speed mode packet size for serial x data */
        #define DEF_USB_HS_PACK_LEN        DEF_USBD_HS_PACK_SIZE                        /* USB high speed mode packet size for serial x data */
    #endif
#endif
#define DEF_UARTx_TX_BUF_NUM_MAX   ( DEF_UARTx_TX_BUF_LEN / DEF_USB_FS_PACK_LEN ) /* Serial x transmit buffer size */

/* Serial port receive timeout related macro definition */
#define DEF_UARTx_BAUDRATE         USB_CDC_BAUDRATE                             /* Default baud rate for serial port */
#define DEF_UARTx_STOPBIT          USB_CDC_STOPBIT                              /* Default stop bit for serial port */
#define DEF_UARTx_PARITY           USB_CDC_PARITY                               /* Default parity bit for serial port */
#define DEF_UARTx_DATABIT          8                                            /* Default data bit for serial port */
#define DEF_UARTx_RX_TIMEOUT       200                                           /* Serial port receive timeout, in 10uS */
#define DEF_UARTx_USB_UP_TIMEOUT   600000                                       /* Serial port receive upload timeout, in 10uS */

#if defined(CH32X035) || defined(CH32X033) || defined(CH32V10X) || defined(CH32V30X)
    typedef enum _RESULT
    {
    USB_SUCCESS = 0,    /* Process successfully */
    USB_ERROR,
    USB_UNSUPPORT,
    USB_NOT_READY       /* The process has not been finished, endpoint will be
                            NAK to further request */
    } RESULT;
#endif

/************************************************************/
/* Serial port X related structure definition */
typedef struct __attribute__((packed)) _UART_CTL
{
    uint8_t  Rx_TimeOut;                                                         /* Serial x data receive timeout */
    uint8_t  Rx_TimeOutMax;                                                      /* Serial x data receive timeout maximum */

    volatile uint16_t Tx_LoadNum;                                                /* Serial x data send buffer load number */
    volatile uint16_t Tx_DealNum;                                                /* Serial x data send buffer processing number */
    volatile uint16_t Tx_RemainNum;                                              /* Serial x data send buffer remaining unprocessed number */
    volatile uint16_t Tx_PackLen[ DEF_UARTx_TX_BUF_NUM_MAX ];                    /* The current packet length of the serial x data send buffer */
    volatile uint16_t Tx_PackPos[ DEF_UARTx_TX_BUF_NUM_MAX ];                    /* Current read position within the current packet */
    uint8_t  Recv1;

    uint8_t  USB_Up_IngFlag;                                                     /* Serial xUSB packet being uploaded flag */
    uint8_t  Recv2;
    uint16_t USB_Up_TimeOut;                                                     /* Serial xUSB packet upload timeout timer */
    uint8_t  USB_Down_StopFlag;                                                  /* Serial xUSB packet stop down flag */

    uint8_t  Com_Cfg[ 8 ];                                                       /* Serial x parameter configuration (default baud rate is 115200, 1 stop bit, no parity, 8 data bits) */
    uint8_t  Recv3;
    uint8_t  USB_Int_UpFlag;                                                     /* Serial x interrupt upload status */
    uint16_t USB_Int_UpTimeCount;                                                /* Serial x interrupt upload timing */
}UART_CTL, *PUART_CTL;

/***********************************************************************************************************************/
/* Constant, variable extents */
/* The following are serial port transmit and receive related variables and buffers */
extern volatile UART_CTL Uart;                                                    /* Serial x control related structure */
extern __attribute__ ((aligned(4))) uint8_t UART2_Tx_Buf[ DEF_UARTx_TX_BUF_LEN ]; /* Serial x transmit buffer */

/***********************************************************************************************************************/
/* Function extensibility */
extern uint8_t RCC_Configuration( void );
#if defined(CH32V20X) || defined(CH32V10X) || defined(CH32V30X)
    extern void TIM2_Init( void );
#elif defined(CH32X035) || defined(CH32X033)
    extern void TIM3_Init( void );
#endif
extern void UART2_ParaInit( uint8_t mode );                                       /* Serial port parameter initialization */
extern void UART2_USB_Init( void );
// ---------- Custom Extensions ----------

#if(USB_TX_MODE == USB_TX_ASYNC)
    #define USB_TX_ASYNC_BUFSIZE    1024
    extern void USB_Tx_runner();                                                                /* Function to send USB data asynchronously, call repeatidly in background */
    _Atomic extern int tx_semaphore;                                                            /* This shared variable locks USB_Tx_runner() (TIM2) interrupt out from sending during write operation into async send buffer _putchar() */
    extern void USB_Tx_flush();                                                                 /* Flush remaining tx buffer into USB port */
#endif

extern void USB_Serial_initialize();                                                            /* USB serial port initialization*/
extern void handle_USB_Error(uint8_t code);
extern void USB_disable_IRQ();
extern void USB_enable_IRQ();
extern void USB_restart_downlink();
extern uint8_t USBx_ENDP3_DataUp( uint8_t *pbuf, uint16_t len );
extern void USB_Rx_flush();

extern void _putchar(char character);
extern void USB_Rx_set_wait(uint8_t setval);
extern int getch();



#ifndef USB_TX_BUFF_SIZE
    #define USB_TX_BUFF_SIZE 1024
#endif
extern void _USB_Serial_print_buffer(char buffer[], uint16_t length, uint16_t start_index);
extern uint16_t USB_Serial_printf(char *format, ...);
extern uint16_t USB_Rx_readpacket(int16_t index, char * buf, uint8_t sanitize);
extern uint16_t USB_Rx_readfull(char * buf, uint16_t max_n, uint8_t sanitize);
extern int USB_Serial_scanf(uint8_t block, char *format, ...);


#ifdef __cplusplus
}
#endif

#endif

/***********************************************************************************************************************/