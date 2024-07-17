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
 *  file         : ch32v_usb_serial.c
 *  description  : usb serial library main code
 *
 */

#include "ch32v_usb_serial.h"

/*******************************************************************************/
/* Variable Definition */
/* Global */

uint8_t USB_Rx_wait = 0; // Store wether to wait for usb to receive input
char packet_buf[64];
int packet_left = 0;
int packet_pos = 0;

// Track temporary float string-buffer
uint16_t tmp_fbuf_pos = 0;                  // position in buffer
char tmp_fbuf[TMP_FBUF_SIZE];               // temporary float string-buffer = 8 slots x 16 chars

/* The following are serial port transmit and receive related variables and buffers */
volatile UART_CTL Uart;

__attribute__ ((aligned(4))) uint8_t  UART2_Tx_Buf[ DEF_UARTx_TX_BUF_LEN ];  /* Serial port 2 transmit data buffer */

extern uint8_t USBD_Endp3_Busy;

// Global buffer and tracking variables for async mode
#if(USB_TX_MODE == USB_TX_ASYNC)
    uint16_t USB_Tx_async_sendindex;                                                    /* Index of next Byte to send */
    uint16_t USB_Tx_async_writeindex;                                                   /* Index of next Byte to write */
    uint16_t USB_Tx_async_num_tosend;                                                   /* Number of Bytes, waiting to be send */
    __attribute__ ((aligned(4))) uint8_t USB_Tx_async_buffer[USB_TX_ASYNC_BUFSIZE];     /* Hold TX Data (FIFO-Style) before sending */
    _Atomic int tx_semaphore = 0;                                                       /* This shared variable locks USB_Tx_runner() (TIM2) interrupt out from sending during write operation into async send buffer _putchar() */
    void USB_Tx_runner();                                                               /* Declaration of asynchronous TX runner function */
#endif

#if (defined(CH32V10X) || defined(CH32V20X) || defined(CH32V30X)) && !defined(EXT_USB_TIM_HANDLER) && (USB_TX_MODE == USB_TX_ASYNC)
    void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#elif (defined(CH32X035) || defined(CH32X033)) && !defined(EXT_USB_TIM_HANDLER) && (USB_TX_MODE == USB_TX_ASYNC)
    void TIM3_IRQHandler( void )__attribute__((interrupt("WCH-Interrupt-fast")));
#endif

/*********************************************************************
 * @fn      RCC_Configuration
 *
 * @brief   Configures the different system clocks.
 *
 * @return  none
 */
uint8_t RCC_Configuration( void )
{
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );
    #if(USB_TX_MODE == USB_TX_ASYNC)
        #if defined(CH32V10X)
            RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );
        #elif defined(CH32V20X)
            RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );
        #elif defined(CH32V30X)
            RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );
        #elif defined(CH32X035) || defined(CH32X033)
            RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );
        #endif
    #endif
    
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );
    return 0;
}

#if (defined(CH32V20X) || defined(CH32V10X) || defined(CH32V30X)) && !defined(EXT_USB_TIM_HANDLER) && (USB_TX_MODE == USB_TX_ASYNC)
/*********************************************************************
 * @fn      TIM2_Init
 *
 * @brief   10us Timer
 *          144 * 10 * 13.8888 -----> 10uS
 *
 * @return  none
 */
void TIM2_Init( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};

    TIM_DeInit( TIM2 );

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 10 - 1; // TIM2 Period: Max value to count till interrupt -> 1 MHz / 10 = 100K -> 10us between interrupt
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1; // TIM2 Prescaler: Divide Systemclock by (itself/1M) -> Now 1MHz Tick-rate for timer
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );

    /* Clear TIM2 update pending flag */
    TIM_ClearFlag( TIM2, TIM_FLAG_Update );

    /* TIM IT enable */
    TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE );

    /* Enable Interrupt */
    NVIC_EnableIRQ( TIM2_IRQn );

    /* TIM2 enable counter */
    TIM_Cmd( TIM2, ENABLE );
}

/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   This function handles TIM2 exception.
 *
 * @return  none
 */
void TIM2_IRQHandler( void )
{
    /* uart timeout counts */
    Uart.Rx_TimeOut++;
    Uart.USB_Up_TimeOut++;
    // If USB TX async mode: When USB Tx ready, send queued tx data via this ISR
	#if(USB_TX_MODE == USB_TX_ASYNC)
        if(!tx_semaphore)
        {
            USB_Tx_runner();
        }
	#endif
    /* clear status */
    TIM2->INTFR = (uint16_t)~TIM_IT_Update;
}

#elif (defined(CH32X035) || defined(CH32X033)) && !defined(EXT_USB_TIM_HANDLER) && (USB_TX_MODE == USB_TX_ASYNC)
/*********************************************************************
 * @fn      TIM3_Init
 *
 * @brief   10us Timer
 *          144 * 10 * 13.8888 -----> 10uS
 *
 * @return  none
 */
void TIM3_Init( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};

    TIM_DeInit( TIM3 );

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 10 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure );

    /* Clear TIM3 update pending flag */
    TIM_ClearFlag( TIM3, TIM_FLAG_Update );

    /* TIM IT enable */
    TIM_ITConfig( TIM3, TIM_IT_Update, ENABLE );

    /* Enable Interrupt */
    NVIC_EnableIRQ( TIM3_IRQn );

    /* TIM3 enable counter */
    TIM_Cmd( TIM3, ENABLE );
}

/*********************************************************************
 * @fn      TIM3_IRQHandler
 *
 * @brief   This function handles TIM3 exception.
 *
 * @return  none
 */
void TIM3_IRQHandler( void )
{
    if( TIM_GetITStatus( TIM3, TIM_IT_Update ) != RESET )
    {
        /* Clear interrupt flag */
        TIM_ClearITPendingBit( TIM3, TIM_IT_Update );
        /* uart timeout counts */
        Uart.Rx_TimeOut++;
        Uart.USB_Up_TimeOut++;
        // If USB TX async mode: When USB Tx ready, send queued tx data via this ISR
        #if(USB_TX_MODE == USB_TX_ASYNC)
            if(!tx_semaphore)
            {
                USB_Tx_runner();
            }
        #endif
    }
}

#endif



/*********************************************************************
 * @fn      UART2_ParaInit
 *
 * @brief   Uart2 parameters initialization
 *          mode = 0 : Used in usb modify initialization
 *          mode = 1 : Used in default initializations
 * @return  none
 */
void UART2_ParaInit( uint8_t mode )
{
    uint8_t i;

    Uart.Rx_TimeOut = 0x00;
    Uart.Rx_TimeOutMax = 30;

    Uart.Tx_LoadNum = 0x00;
    Uart.Tx_DealNum = 0x00;
    Uart.Tx_RemainNum = 0x00;
    for( i = 0; i < DEF_UARTx_TX_BUF_NUM_MAX; i++ )
    {
        Uart.Tx_PackLen[ i ] = 0x00;
        Uart.Tx_PackPos[ i ] = 0x00;
    }

    Uart.USB_Up_IngFlag = 0x00;
    Uart.USB_Up_TimeOut = 0x00;
    Uart.USB_Down_StopFlag = 0x00;

    if( mode )
    {
        Uart.Com_Cfg[ 0 ] = (uint8_t)( DEF_UARTx_BAUDRATE );
        Uart.Com_Cfg[ 1 ] = (uint8_t)( DEF_UARTx_BAUDRATE >> 8 );
        Uart.Com_Cfg[ 2 ] = (uint8_t)( DEF_UARTx_BAUDRATE >> 16 );
        Uart.Com_Cfg[ 3 ] = (uint8_t)( DEF_UARTx_BAUDRATE >> 24 );
        Uart.Com_Cfg[ 4 ] = DEF_UARTx_STOPBIT;
        Uart.Com_Cfg[ 5 ] = DEF_UARTx_PARITY;
        Uart.Com_Cfg[ 6 ] = DEF_UARTx_DATABIT;
        Uart.Com_Cfg[ 7 ] = DEF_UARTx_RX_TIMEOUT;
    }
}

/*********************************************************************
 * @fn      UART2_USB_Init
 *
 * @brief   Uart2 initialization in usb interrupt
 *
 * @return  none
 */
void UART2_USB_Init( void )
{
    uint32_t baudrate;
    //uint8_t  stopbits;
    //uint8_t  parity;

    baudrate = ( uint32_t )( Uart.Com_Cfg[ 3 ] << 24 ) + ( uint32_t )( Uart.Com_Cfg[ 2 ] << 16 );
    baudrate += ( uint32_t )( Uart.Com_Cfg[ 1 ] << 8 ) + ( uint32_t )( Uart.Com_Cfg[ 0 ] );
    //stopbits = Uart.Com_Cfg[ 4 ];
    //parity = Uart.Com_Cfg[ 5 ];

    UART2_ParaInit( 0 );

    /* restart usb receive  */
    #if defined(CH32V10X)
        R16_UEP2_DMA = (uint16_t)(uint32_t)(uint8_t *)&UART2_Tx_Buf[ 0 ];
        R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
    #elif defined(CH32V20X)
        #if(USB_CDCPORT == CDC_USBFS)
            USBOTG_FS->UEP2_DMA = (uint32_t)(uint8_t *)&UART2_Tx_Buf[ 0 ];
            USBOTG_FS->UEP2_RX_CTRL &= ~USBFS_UEP_R_RES_MASK;
            USBOTG_FS->UEP2_RX_CTRL |= USBFS_UEP_R_RES_ACK;
        #endif
    #elif defined(CH32V30X)
        USBHSD->UEP2_RX_DMA = (uint32_t)(uint8_t *)&UART2_Tx_Buf[ 0 ];
        USBHSD->UEP2_RX_CTRL &= ~USBFS_UEP_R_RES_MASK;
        USBHSD->UEP2_RX_CTRL |= USBFS_UEP_R_RES_ACK;
    #elif defined(CH32X035) || defined(CH32X033)
        USBFSD->UEP2_DMA = (uint32_t)(uint8_t *)&UART2_Tx_Buf[ 0 ];
        USBFSD->UEP2_CTRL_H &= ~USBFS_UEP_R_RES_MASK;
        USBFSD->UEP2_CTRL_H |= USBFS_UEP_R_RES_ACK;
    #endif
}

/*********************************************************************
 * @fn      USB_Serial_initialize
 *
 * @brief   Initializes USB Serial interface with configured parameters (see ch32v_usb_serial.h).
 *
 * @return  none
 */
void USB_Serial_initialize()
{
    RCC_Configuration( );

    #if defined(CH32V10X) && (USB_TX_MODE == USB_TX_ASYNC)
        TIM2_Init( );
    #elif defined(CH32V20X) && (USB_TX_MODE == USB_TX_ASYNC)
        TIM2_Init( );
    #elif defined(CH32V30X) && (USB_TX_MODE == USB_TX_ASYNC)
        TIM2_Init( );
    #elif (defined(CH32X035) || defined(CH32X033)) && (USB_TX_MODE == USB_TX_ASYNC)
        TIM3_Init( );
    #endif
    
    UART2_ParaInit( 1 );
    #if(USB_CDCPORT == CDC_USBFS)
        /* USB20 device init */
        USBFS_RCC_Init( );
        #if defined(CH32V10X)
            USBFS_Device_Init( ENABLE , PWR_VDD_SupplyVoltage());
        #elif defined(CH32V20X)
            USBFS_Device_Init( ENABLE );
        #elif defined(CH32V30X)
            USBFS_Device_Init( ENABLE );
        #elif defined(CH32X035) || defined(CH32X033)
            USBFS_Device_Init( ENABLE , PWR_VDD_SupplyVoltage());
        #endif
    #else
        #if defined(CH32V20X)
            Set_USBConfig(); 
            USB_Init();	    
 	        USB_Interrupts_Config();
        #elif defined(CH32V30X)
            USBHS_Device_Init( ENABLE );
        #endif
    #endif
    // Initialize tracking variables for async tx buffer
    #if(USB_TX_MODE == USB_TX_ASYNC)
        USB_Tx_async_sendindex = 0;                                    
        USB_Tx_async_writeindex = 0;                                   
        USB_Tx_async_num_tosend = 0;
    #endif
}

/*********************************************************************
 * @fn      handle_USB_Error
 *
 * @brief   Handle USB Error code by printing error message to debug UART channel.
 * 
 * @param   code Error code to handle.
 *
 * @return  none
 */
void handle_USB_Error(uint8_t code)
{
    if(code == USB_ERROR) printf("[ERROR] USB_ERROR returned\r\n");
    else if(code == USB_UNSUPPORT) printf("[ERROR] USB_UNSUPPORT returned\r\n");
    else if(code == USB_NOT_READY) printf("[ERROR] USB_NOT_READY returned\r\n");
    else if(code != USB_SUCCESS) printf("[ERROR] Unknown USB Issue returned\r\n");
}

/*********************************************************************
 * @fn      USB_disable_IRQ
 *
 * @brief   Disable USB IRQ and allow for modification read/write operation from/to USB endpoint.
 *
 * @return  none
 */
void USB_disable_IRQ()
{
    // USB Interrupt deactivate
    #if(USB_CDCPORT == CDC_USBFS)
        #if defined(CH32V10X)
            NVIC_DisableIRQ( USBHD_IRQn );
        #elif defined(CH32X035) || defined(CH32X033)
            NVIC_DisableIRQ( USBFS_IRQn );
        #elif defined(CH32V20X)
            NVIC_DisableIRQ( USBFS_IRQn );
            NVIC_DisableIRQ( USBHD_IRQn );
        #elif defined(CH32V30X)
            NVIC_DisableIRQ( OTG_FS_IRQn );
            NVIC_DisableIRQ( OTG_FS_IRQn );
        #endif
    #else
        #if defined(CH32V20X)
            NVIC_DisableIRQ( USB_LP_CAN1_RX0_IRQn );
            NVIC_DisableIRQ( USB_HP_CAN1_TX_IRQn );
        #elif defined(CH32V30X)
            NVIC_DisableIRQ( USBHS_IRQn );
            NVIC_DisableIRQ( USBHS_IRQn );
        #endif
    #endif
}

/*********************************************************************
 * @fn      USB_enable_IRQ
 *
 * @brief   Enable USB IRQ and allow for USB endpoint to operate again.
 *
 * @return  none
 */
void USB_enable_IRQ()
{
    // USB Interrupt activate
    #if(USB_CDCPORT == CDC_USBFS)
        #if defined(CH32V10X)
            NVIC_EnableIRQ( USBHD_IRQn );
        #elif defined(CH32X035) || defined(CH32X033)
            NVIC_EnableIRQ( USBFS_IRQn );
        #elif defined(CH32V20X)
            NVIC_EnableIRQ( USBFS_IRQn );
            NVIC_EnableIRQ( USBHD_IRQn );
        #elif defined(CH32V30X)
            NVIC_EnableIRQ( OTG_FS_IRQn );
        #endif
    #else
        #if defined(CH32V20X)
            NVIC_EnableIRQ( USB_LP_CAN1_RX0_IRQn );
            NVIC_EnableIRQ( USB_HP_CAN1_TX_IRQn );
        #elif defined(CH32V30X)
            NVIC_EnableIRQ( USBHS_IRQn );
        #endif
    #endif  
}

/*********************************************************************
 * @fn      USB_restart_downlink
 *
 * @brief   If the current serial port has suspended the downlink, restart the driver downlink.
 *
 * @return  none
 */
void USB_restart_downlink()
{
    #if defined(CH32V10X)
        R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
    #elif defined(CH32V20X)
        #if(USB_CDCPORT == CDC_USBFS)
            USBFSD->UEP2_RX_CTRL &= ~USBFS_UEP_R_RES_MASK;
            USBFSD->UEP2_RX_CTRL |= USBFS_UEP_R_RES_ACK;
        #else
            SetEPRxValid( ENDP2 );
        #endif
    #elif defined(CH32V30X)
        USBHSD->UEP2_RX_CTRL &= ~USBFS_UEP_R_RES_MASK;
        USBHSD->UEP2_RX_CTRL |= USBFS_UEP_R_RES_ACK;
    #elif defined(CH32X035) || defined(CH32X033)
        USBFSD->UEP2_CTRL_H &= ~USBFS_UEP_R_RES_MASK;
        USBFSD->UEP2_CTRL_H |= USBFS_UEP_R_RES_ACK;
    #endif
    Uart.USB_Down_StopFlag = 0x00; // If buffer close to beeing 100% free and RX closed, open USB RX again
}

uint8_t USBx_ENDP3_DataUp( uint8_t *pbuf, uint16_t len )
{
    uint8_t success = USB_ERROR;
    #if(USB_CDCPORT == CDC_USBFS)
        USB_disable_IRQ();
        Uart.USB_Up_IngFlag = 0x01; // Paket upload flag, automatically de-asserted by EP3 IN-Callback on USB-D / Interrrupt on USB-FS
        Uart.USB_Up_TimeOut = 0x00; // Reset USB-TX Timeout (automatically counted up by TIM2 Interrupt)
        success = USBFS_Endp_DataUp( DEF_UEP3, pbuf, len, DEF_UEP_CPY_LOAD ); // Send data to USB Endpoint
        USB_enable_IRQ();
    #else
        #if defined(CH32V20X)
            USB_disable_IRQ();
            Uart.USB_Up_IngFlag = 0x01; // Paket upload flag, automatically de-asserted by EP3 IN-Callback on USB-D / Interrrupt on USB-FS
            Uart.USB_Up_TimeOut = 0x00; // Reset USB-TX Timeout (automatically counted up by TIM2 Interrupt)
            success = USBD_ENDPx_DataUp( ENDP3, pbuf, len); // Send data to USB Endpoint
            USB_enable_IRQ();
        #elif defined(CH32V30X)
            success = USB_SUCCESS;
            USB_disable_IRQ();
            Uart.USB_Up_IngFlag = 0x01; // Paket upload flag, automatically de-asserted by EP3 IN-Callback on USB-D / Interrrupt on USB-FS
            Uart.USB_Up_TimeOut = 0x00; // Reset USB-TX Timeout (automatically counted up by TIM2 Interrupt)
            USBHSD->UEP2_TX_DMA = (uint32_t)(uint8_t *)pbuf;
            USBHSD->UEP2_TX_LEN  = len;
            USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
            USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_ACK;
            USB_enable_IRQ();
        #endif
    #endif
    handle_USB_Error(success); // Handle possible errors in transmission
    return success;
}

/*********************************************************************
 * @fn      USB_Rx_flush
 *
 * @brief   Flush and reset USB RX buffer
 * 
 * @return  none
 */
void USB_Rx_flush()
{
    // USB Interrupt deactivate
    USB_disable_IRQ();

    // Reset PackLen to 0
    for(uint16_t i = 0; i < DEF_UARTx_TX_BUF_NUM_MAX; i++)
    {
        Uart.Tx_PackLen[ i ] = 0x00;
        Uart.Tx_PackPos[ i ] = 0x00;
    }
    // Reset Packet-tracking parameters
    Uart.Tx_LoadNum = 0x00;
    Uart.Tx_DealNum = 0x00;
    Uart.Tx_RemainNum = 0x00;

    // USB Interrupt activate
    USB_enable_IRQ();
}

#if(USB_TX_MODE == USB_TX_ASYNC)

/*********************************************************************
 * @fn      USB_Tx_runner
 *
 * @brief   If USB_TX_MODE is USB_TX_ASYNC, this function needs to be called repeatadly in a while loop to
 *          send queued data via USB. Does not wait for USB TX availability.
 *
 * @return  none
 */
void USB_Tx_runner()
{
    //printf("USB_Tx_runner() called ...\r\n");
    if(USB_Tx_async_num_tosend > 0)
    {
        // If USB Tx ready, send data, otherwise omit this call ...
        if(Uart.USB_Up_IngFlag == 0)
        {
            uint16_t send_this_run = DEF_USB_FS_PACK_LEN; // Number of bytes to send in this run
            // If possible, send 64-Byte Chunk at once
            if(USB_Tx_async_num_tosend < DEF_USB_FS_PACK_LEN)
            {
                send_this_run = USB_Tx_async_num_tosend;
            }
            // If not send only as many as available to send, or till wrap-around of buffer
            if((USB_Tx_async_sendindex + send_this_run) >= USB_TX_ASYNC_BUFSIZE)
            {
                send_this_run = USB_TX_ASYNC_BUFSIZE - USB_Tx_async_sendindex;
            }
            // Send full data via USB
            USBx_ENDP3_DataUp( &USB_Tx_async_buffer[USB_Tx_async_sendindex], send_this_run );
            // Update Send Index and Num to send
            USB_Tx_async_sendindex += send_this_run;
            USB_Tx_async_num_tosend -= send_this_run;
            // Debug
            //printf("USB_Tx_runner() sent %d Bytes from sendindex: %d ... \r\n", send_this_run, USB_Tx_async_sendindex);
            //printf("USB_Tx_runner() writeindex is %d , Num to send: %d \r\n", USB_Tx_async_writeindex, USB_Tx_async_num_tosend);
            // If sendindex wrap-around, reset to 0
            if(USB_Tx_async_sendindex >= USB_TX_ASYNC_BUFSIZE)
            {
                USB_Tx_async_sendindex = 0;
            }
        }
        else
        {
            // ... unless USB Tx timeout is reached, then reset busy-flag so next transmit can happen
            /* Set the upload success flag directly if the previous upload is not successful after the timeout */
            if( Uart.USB_Up_TimeOut >= DEF_UARTx_USB_UP_TIMEOUT )
            {
                Uart.USB_Up_IngFlag = 0x00;
                #if(USB_CDCPORT == CDC_USBFS)
                    #if defined(CH32V30X)
                        USBFS_Endp_Busy[ DEF_UEP3 ] = 0;
                    #else
                        USBFS_Endp_Busy[ DEF_UEP3 ] = 0;
                    #endif
                #else
                    #if defined(CH32V30X)
                        USBHS_Endp_Busy[ DEF_UEP2 ] = 0;
                    #else
                        USBD_Endp3_Busy = 0;
                    #endif
                #endif
            }
        }
        
    }
}

/*********************************************************************
 * @fn      USB_Tx_flush
 *
 * @brief   Flush and reset USB TX buffer
 * 
 * @return  none
 */
void USB_Tx_flush()
{
   USB_Tx_async_num_tosend = 0;
   USB_Tx_async_sendindex = USB_Tx_async_writeindex;
}

#endif

/*********************************************************************
 * @fn      _putchar
 *
 * @brief   Print a single character to USB TX
 * 
 * @param   character      Character to print
 *
 * @return  none
 */
void _putchar(char character)
{
    uint8_t unsigned_character = (uint8_t)(character);

    #if(USB_TX_MODE == USB_TX_ASYNC)
        // Disable Async USB_Tx_runner() during writing
        tx_semaphore = 1;
        // If asynchronous TX mode, do not wait for TX availability, only store into TX buffer
        // Write Data from buffer into USB_Tx_async_buffer
        // Wrap-around write index to 0 if end of Buffer reached
        if(USB_Tx_async_writeindex >= USB_TX_ASYNC_BUFSIZE)
        {
            USB_Tx_async_writeindex = 0;
        }
        // Write Byte and increment write index 
        USB_Tx_async_buffer[USB_Tx_async_writeindex] = unsigned_character;
        USB_Tx_async_writeindex++;
        // Update number of bytes to send
        USB_Tx_async_num_tosend = USB_Tx_async_writeindex - USB_Tx_async_sendindex;
        if(USB_Tx_async_num_tosend < 0)
        {
            USB_Tx_async_num_tosend = ( USB_TX_ASYNC_BUFSIZE - USB_Tx_async_sendindex ) + USB_Tx_async_writeindex;
        }
        // Enable Async USB_Tx_runner() again
        tx_semaphore = 0;
        
    #else
        // If synchronous TX mode, wait for TX availability or timeout, then send character
        while(Uart.USB_Up_IngFlag != 0)
        {
            // Wait for Endpoint to be ready for transmit
            /* Set the upload success flag directly if the previous upload is not successful after the timeout */
            if( Uart.USB_Up_TimeOut >= DEF_UARTx_USB_UP_TIMEOUT )
            {
                Uart.USB_Up_IngFlag = 0x00;
                #if(USB_CDCPORT == CDC_USBFS)
                    USBFS_Endp_Busy[ DEF_UEP3 ] = 0;
                #else
                    USBD_Endp3_Busy = 0;
                #endif
                break;
            }
        }
        USBx_ENDP3_DataUp( &unsigned_character, (uint16_t)1 );
    #endif
}

/*********************************************************************
 * @fn      USB_Rx_set_wait
 *
 * @brief   Set variable for getch() wait, if set to x > 0, getch() will wait if no data in USB RX buffer (useful for CLI terminal emulation)
 * 
 * @param   setval      Set wether to wait ( x > 0) or not wait (x = 0)
 *
 * @return  none
 */
void USB_Rx_set_wait(uint8_t setval)
{
    USB_Rx_wait = setval;
}

/*********************************************************************
 * @fn      _readch
 *
 * @brief   Internal read character function
 * 
 * @return  Received character as integer
 */
int _readch()
{
    int buf;

    if(Uart.Tx_PackPos[Uart.Tx_DealNum] >= DEF_USB_FS_PACK_LEN)
    {
        Uart.Tx_PackPos[Uart.Tx_DealNum] = 0;
    }
    if(Uart.Tx_DealNum >= 16)
    {
        Uart.Tx_DealNum = 0;
    }
    buf = (int)UART2_Tx_Buf[ ( (Uart.Tx_DealNum * DEF_USB_FS_PACK_LEN) + Uart.Tx_PackPos[Uart.Tx_DealNum]) ];
    Uart.Tx_PackLen[Uart.Tx_DealNum]--;         // Update Packet length
    Uart.Tx_PackPos[Uart.Tx_DealNum]++;         // Update Packet read position
    // If end of packet reached
    if(Uart.Tx_PackLen[Uart.Tx_DealNum] <= 0)
    {
        Uart.Tx_PackPos[Uart.Tx_DealNum] = 0;       // Reset Packet read position on leaving packet
        Uart.Tx_RemainNum--;                        // One less Load to process now
        Uart.Tx_DealNum++;                          // Increment number of next
        if( Uart.Tx_DealNum >= DEF_UARTx_TX_BUF_NUM_MAX )
        {
            Uart.Tx_DealNum = 0x00; // Wrap-around when limit of num of reads reached
        }     
    }
    return buf;
}

/*********************************************************************
 * @fn      getch
 *
 * @brief   Read a character from USB RX buffer
 * 
 * @return  Received character as integer
 */
int getch()
{
    int buf;

    if (USB_Rx_wait)
    {
        // --------- Do Wait ----------
        while ( Uart.Tx_PackLen[Uart.Tx_DealNum] <= 0 )
        {
        }
        // USB Interrupt deactivate
        USB_disable_IRQ();
        buf = _readch();
        #ifdef GETCH_CLI_FEEDBACK
            _putchar((char)buf); // Print read char into console as feedback to user (print out previous unsent tx data as well)
        #endif
    }
    else
    {
        // --------- Do not Wait ----------
        if(Uart.Tx_PackLen[Uart.Tx_DealNum] > 0)
        {
            // Packet is not empty, copy content into buf
            // USB Interrupt deactivate
            USB_disable_IRQ();
            buf = _readch();
            #ifdef GETCH_CLI_FEEDBACK
                _putchar((char)buf); // Print read char into console as feedback to user (print out previous unsent tx data as well)
            #endif
        }
        else
        {
            // If next packet has no data yet, do not wait and just return 0
            buf = 0;
        }  
    }

    #if(USB_RX_OVERFLOW == USB_RX_HALT)
        // If the current serial port has suspended the downlink, restart the driver downlink
        if( ( Uart.USB_Down_StopFlag == 0x01 ) && ( Uart.Tx_RemainNum < 2 ) )
        {
            USB_restart_downlink();
        }
    #endif

    // USB Interrupt activate
    USB_enable_IRQ();

    return buf;
}

/*********************************************************************
 * @fn      _USB_Serial_print_buffer
 *
 * @brief   Print buffer via USB serial (waits for USB TX availability in sync-mode), 
 *          if USB_TX_MODE is USB_TX_ASYNC, this only queues data to print and returns immediatly.
 * 
 * @param   buffer      Char Buffer to print.
 * @param   length      Number of chars from buffer to print.
 * @param   start_index Starting index in buffer to begin printing from.
 *
 * @return  none
 */
void _USB_Serial_print_buffer(char buffer[], uint16_t length, uint16_t start_index)
{
    #if(USB_TX_MODE == USB_TX_ASYNC)
        uint8_t * uint_buffer = (uint8_t*)buffer + start_index;
        // Write Data from buffer into USB_Tx_async_buffer
        for(uint16_t i = 0; i < length; i++)
        {
            // Wrap-around write index to 0 if end of Buffer reached
            if(USB_Tx_async_writeindex >= USB_TX_ASYNC_BUFSIZE)
            {
                USB_Tx_async_writeindex = 0;
            }
            // Write Byte and increment write index 
            USB_Tx_async_buffer[USB_Tx_async_writeindex] = uint_buffer[i];
            USB_Tx_async_writeindex++;
        }
        // Update number of bytes to send
        USB_Tx_async_num_tosend += length;
    #else
        uint8_t success = 0;
        uint8_t * uint_buffer = (uint8_t*)buffer + start_index;

        if(length > 0)
        {
            // USBD and USBFS Device Endpoint can only send 64-Bytes at once, split buffer up and send in 64-byte chunks
            uint16_t remaining_bytes = length;
            uint16_t index = 0;
            while(remaining_bytes > 0)
            {
                uint16_t local_start_index = index * DEF_USB_FS_PACK_LEN;
                uint16_t num_to_send = length - local_start_index;
                if (num_to_send > DEF_USB_FS_PACK_LEN)
                {
                    while(Uart.USB_Up_IngFlag != 0)
                    {
                        // Wait for Endpoint to be ready for transmit
                        /* Set the upload success flag directly if the previous upload is not successful after the timeout */
                        if( Uart.USB_Up_TimeOut >= DEF_UARTx_USB_UP_TIMEOUT )
                        {
                            Uart.USB_Up_IngFlag = 0x00;
                            #if(USB_CDCPORT == CDC_USBFS)
                                USBFS_Endp_Busy[ DEF_UEP3 ] = 0;
                            #else
                                USBD_Endp3_Busy = 0;
                            #endif
                            break;
                        }
                    }
                    // Send full 64-byte Package
                    success = USBx_ENDP3_DataUp( &uint_buffer[local_start_index], DEF_USB_FS_PACK_LEN );
                    remaining_bytes -= DEF_USB_FS_PACK_LEN;
                }
                else
                {
                    while(Uart.USB_Up_IngFlag != 0)
                    {
                        // Wait for Endpoint to be ready for transmit
                        /* Set the upload success flag directly if the previous upload is not successful after the timeout */
                        if( Uart.USB_Up_TimeOut >= DEF_UARTx_USB_UP_TIMEOUT )
                        {
                            Uart.USB_Up_IngFlag = 0x00;
                            #if(USB_CDCPORT == CDC_USBFS)
                                USBFS_Endp_Busy[ DEF_UEP3 ] = 0;
                            #else
                                USBD_Endp3_Busy = 0;
                            #endif
                            break;
                        }
                    }
                    // Send the remaining bytes
                    success = USBx_ENDP3_DataUp( &uint_buffer[local_start_index], num_to_send );
                    remaining_bytes -= num_to_send;
                }
                index++;
            }
        }
    #endif
}

/*********************************************************************
 * @fn      USB_Serial_printf
 *
 * @brief   Works like a regular printf function. Prints data to USB serial, requires USB_Tx_runner() if USB_TX_MODE is USB_TX_ASYNC.
 * 
 * @param   format Format String with specifiers. Additional arguments possible.
 *
 * @return  Number of characters printed.
 */
uint16_t USB_Serial_printf(char *format, ...)
{
    va_list args;
    char buffer[USB_TX_BUFF_SIZE];
    uint16_t count = 0;

    va_start(args, format);

    count += (uint16_t)vsnprintf(buffer, sizeof(buffer), format, args);     // convert to string and write into buffer
    _USB_Serial_print_buffer(buffer, count, 0);                             // write/queue string for USB TX

    va_end(args);
    return count;
}

/*********************************************************************
 * @fn      ftoa_s
 *
 * @brief   Convert a double variable into a string that represents it's value. Works similar to ftoa().
 *          WARNING: This function uses a ringbuffer, and can only convert up to TMP_FSTR_NUM doubles at once
 *                      (as per in a single printf() call). Each double must be smaller than TMP_FSTR_SIZE in number of characters to represent.
 * 
 * @param   val         Double value to convert to string.
 * @param   precision   The number of decimals after the '.' to include.
 *
 * @return  A string that represents val. (Held in ringbuffer only till overwritte by new values)
 */
char * ftoa_s(double val, int precision)
{
    char * fstr_ptr = &tmp_fbuf[tmp_fbuf_pos * TMP_FSTR_SIZE];
    ftoa(val, fstr_ptr, precision, 1);                 // Convert float to str and write into fbuf
    tmp_fbuf_pos++;
    if(tmp_fbuf_pos >= TMP_FSTR_NUM)                   // Increment ringbuffer write position
    {
        tmp_fbuf_pos = 0;
    }
    return fstr_ptr;
}

/*********************************************************************
 * @fn      USB_Rx_readpacket
 *
 * @brief   Read the content of one USB RX packet (max. 64 Bytes) and copy it into a specified buffer.
 * 
 * @param   index Specify the index of the packet ([0:15]) or a negative number to let Tx_DealNum decide.
 * @param   buf Pointer to the buffer to copy contents of the USB RX packet into.
 * @param   sanitize Whether to reset contents inside copied USB RX packet to zero.
 *
 * @return  Number of characters read.
 */
uint16_t USB_Rx_readpacket(int16_t index, char * buf, uint8_t sanitize)
{
    uint16_t rx_count = 0;              // Return how many bytes were read

    if(index < 0)
    {
        index = Uart.Tx_DealNum;        // If Packet index is negative, use current DealNum index
    }

    // USB Interrupt deactivate
    USB_disable_IRQ();

    if(Uart.Tx_PackLen[index] > 0)
    {
        // Packet is not empty, copy content into buf
        memcpy(buf, (char*)&UART2_Tx_Buf[ ( index * DEF_USB_FS_PACK_LEN ) ], Uart.Tx_PackLen[index]);
        rx_count = Uart.Tx_PackLen[index];
        // If needed, sanitize data in packet to zero
        if(sanitize)
        {
            for(uint16_t i = (index * DEF_USB_FS_PACK_LEN); i < ((index + 1) * DEF_USB_FS_PACK_LEN); i++)
            {
                UART2_Tx_Buf[ i ] = 0;
            }
        }
        Uart.Tx_PackLen[index] = 0;     // Reset Packet length
        Uart.Tx_RemainNum--;            // One less Load to process now
        Uart.Tx_DealNum++;              // Increment number of next
        if( Uart.Tx_DealNum >= DEF_UARTx_TX_BUF_NUM_MAX )
        {
            Uart.Tx_DealNum = 0x00; // Wrap-around when limit of num of reads reached
        }     
    }

    #if(USB_RX_OVERFLOW == USB_RX_HALT)
        /* If the current serial port has suspended the downlink, restart the driver downlink */
        if( ( Uart.USB_Down_StopFlag == 0x01 ) && ( Uart.Tx_RemainNum < 2 ) )
        {
            USB_restart_downlink();
        }
    #endif

    // USB Interrupt activate
    USB_enable_IRQ();

    return rx_count;
}

/*********************************************************************
 * @fn      USB_Rx_readfull
 *
 * @brief   Read and copy entire content of USB RX buffer, till eithe max_n number of bytes copied or no new RX data left.
 *          Data will still be received by the USB RX endpoint inbetween packet read-outs while this function is in action, 
 *          meaning that in high-traffic scenarios, it is more likely that the function will fill the specified buffer with max_n characters.
 * 
 * @param   buf Pointer to the buffer to copy contents of USB RX into.
 * @param   max_n Maximum number of characters/bytes to read.
 * @param   sanitize Whether to reset contents inside copied USB RX areas to zero.
 *
 * @return  Number of characters read.
 */
uint16_t USB_Rx_readfull(char * buf, uint16_t max_n, uint8_t sanitize)
{
    uint16_t rx_count = 0;              // Return how many bytes were read
    max_n -= DEF_USB_FS_PACK_LEN;       // Ensure Limits of buf are never overwritten
    while(Uart.Tx_RemainNum && rx_count < max_n)
    {
        // While Data on RX port, read it out until either all is processed or max_n of read bytes is reached
        rx_count += USB_Rx_readpacket(-1, (buf + rx_count), sanitize);
    }
    return rx_count;
}

/*********************************************************************
 * @fn      USB_Serial_scanf
 *
 * @brief   Similarly to scanf(), receive inputs from the USB serial port. Optionally call USB_Rx_flush() before invokation if needed.
 * 
 * @param   block Wether to flush receive buffer first and wait for new inputs by user (delimiter = '\n')
 * @param   format Format-string with specifiers for requested data-types.
 *
 * @return  Number of successfully parsed variables. Return EOF (-1) if receive buffer empty, return 0 if none parsed successfully.
 */
int USB_Serial_scanf(uint8_t block, char *format, ...)
{
    va_list args;                               // For declaration of list or arguments
    char rx_buf[USB_SCANF_BUF_SIZE];            // Buffer for received data
    uint16_t rx_buf_writeindex = 0;             // Receiver buffer write index

    // ---------- Buffer fill from USB RX ----------
    //If blocking enabled, first flush RX buffer and then wait till new data comes in
    if(block)
    {
        // Flush receive buffer
        //USB_Rx_flush();
        // Wait till delimiter char '\n' appears at ending of buffer or buffer limit is reached
        while(rx_buf[rx_buf_writeindex-1] != '\n')
        {
            while(!Uart.Tx_RemainNum)
            {
                // Wait for new data...
            }
            // Read new data
            uint16_t num_read_bytes = USB_Rx_readfull(&rx_buf[rx_buf_writeindex], (USB_SCANF_BUF_SIZE - rx_buf_writeindex), 0);
            rx_buf_writeindex += num_read_bytes;
            // Write read data into USB TX to give feedback to prompt
            #ifdef GETCH_CLI_FEEDBACK
                _USB_Serial_print_buffer(rx_buf, num_read_bytes, (rx_buf_writeindex - num_read_bytes));
            #endif
            // Buffer limit reached? -> leave while-loop
            if(rx_buf_writeindex >= USB_SCANF_BUF_SIZE)
            {
                break;
            }
        }
    }
    else
    {
        // If non-blocking, just read what's in receive buffer right now
        rx_buf_writeindex += USB_Rx_readfull(&rx_buf[0], (USB_SCANF_BUF_SIZE-1), 0);
    }
    // Append 0 to tell parser where end of input buf string is (override delimiter '\n')
    rx_buf[rx_buf_writeindex - 1] = 0;

    // ---------- Parsing ----------
    va_start(args, format);                     // Here you are starting your list from the format

    int num_parsed = vsscanf(rx_buf, format, args);

    va_end(args);
    return num_parsed;
}

/*********************************************************************
 * @fn      ratof
 *
 * @brief   Similarly to stdlib's atof() or strtod(), it converts a string of a FP32 double to a double variable. (But at much lower memory footprint)
 *          WARNING: This function skips all letters in a string, malformed float strings can result in unwanted behaviour.
 *          WARNING: Exponential floats are not accepted by this function (e.g. 10E-5 leads to 105).
 * 
 * @param   arr C String of represented double value
 *
 * @return  Double floating point value.
 */
double ratof(char *arr)
{
  double val = 0;
  int afterdot=0;
  double scale=1;
  int neg = 0; 

    // Handle negative floats
    if (*arr == '-') 
    {
        arr++;
        neg = 1;
    }
    while (*arr) 
    {
        if((*arr >= '0' && *arr <= '9') || *arr == '.') // Skip non-numbers and non-dots
        {
            if (afterdot)                               // Handle decimals
            {
                scale = scale/10;
                val = val + (*arr-'0')*scale;
            } 
            else 
            {
                if (*arr == '.')                        // Handle dot
                {
                    afterdot++;
                }
                else 
                {
                    val = val * 10.0 + (*arr - '0');    // Handle digits infront of dot
                }
            }
        }
        arr++;
    }
  if(neg) return -val;
  else    return  val;
}

/*********************************************************************
 * @fn      ratoff
 *
 * @brief   Similarly to stdlib's atoff(), it converts a string of a FP16 float to a float variable. (But at much lower memory footprint)
 *          WARNING: This function skips all letters in a string, malformed float strings can result in unwanted behaviour.
 *          WARNING: Exponential floats are not accepted by this function (e.g. 10E-5 leads to 105).
 * 
 * @param   arr C String of represented float value
 *
 * @return  Floating point value.
 */
float ratoff(char *arr)
{
  float val = 0;
  int afterdot=0;
  float scale=1;
  int neg = 0; 

    // Handle negative floats
    if (*arr == '-') 
    {
        arr++;
        neg = 1;
    }
    while (*arr) 
    {
        if((*arr >= '0' && *arr <= '9') || *arr == '.') // Skip non-numbers and non-dots
        {
            if (afterdot)                               // Handle decimals
            {
                scale = scale/10;
                val = val + (*arr-'0')*scale;
            } 
            else 
            {
                if (*arr == '.')                        // Handle dot
                {
                    afterdot++;
                }
                else 
                {
                    val = val * 10.0 + (*arr - '0');    // Handle digits infront of dot
                }
            }
        }
        arr++;
    }
  if(neg) return -val;
  else    return  val;
}
