/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_endp.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/08/08
 * Description        : Endpoint routines
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/ 
#include "hal/CH32V20X/USB-Driver/usb_lib.h"
#include "usb_desc.h"
#include "hal/CH32V20X/USB-Driver/usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "usb_prop.h"
#include "ch32v_usb_serial.h"

uint8_t USBD_Endp3_Busy;
uint16_t USB_Rx_Cnt=0; 

/*********************************************************************
 * @fn      EP2_IN_Callback
 *
 * @brief  Endpoint 1 IN.
 *
 * @return  none
 */
void EP1_IN_Callback (void)
{ 
	
}



/*********************************************************************
 * @fn      EP2_OUT_Callback
 *
 * @brief  Endpoint 2 OUT.
 *
 * @return  none
 */
void EP2_OUT_Callback (void)
{ 
	uint32_t len;
    len = GetEPRxCount( EP2_OUT & 0x7F ); // Amount of Bytes red via USB Endpoint 2
    PMAToUserBufferCopy( &UART2_Tx_Buf[ ( Uart.Tx_LoadNum * DEF_USB_FS_PACK_LEN ) ], GetEPRxAddr( EP2_OUT & 0x7F ), len ); // Copy data from USB RX to buffer
    Uart.Tx_PackLen[ Uart.Tx_LoadNum ] = len; // Buffer organized as 1kB with Loads of up to 64 Bytes each (16 Loads, LoadNum is index)
    Uart.Tx_LoadNum++;
    if( Uart.Tx_LoadNum >= DEF_UARTx_TX_BUF_NUM_MAX )
    {
        Uart.Tx_LoadNum = 0x00; // Wrap-around when limit of num of loads reached
    }
    Uart.Tx_RemainNum++; // One more Load to process now
	#if(USB_RX_OVERFLOW == USB_RX_HALT)
		if( Uart.Tx_RemainNum >= ( DEF_UARTx_TX_BUF_NUM_MAX - 2 ) )
		{
			Uart.USB_Down_StopFlag = 0x01; // If buffer soon to be overloaded, stop USB RX
		}
		else
		{
			// If not, signal everything is ok
			#if(USB_CDCPORT == CDC_USBFS)
            	USBFSD->UEP2_RX_CTRL &= ~USBFS_UEP_R_RES_MASK;
            	USBFSD->UEP2_RX_CTRL |= USBFS_UEP_R_RES_ACK;
        	#else
            	SetEPRxValid( ENDP2 );
        	#endif
		}
	#else
		// Always signal everything is ok
		#if(USB_CDCPORT == CDC_USBFS)
            USBFSD->UEP2_RX_CTRL &= ~USBFS_UEP_R_RES_MASK;
            USBFSD->UEP2_RX_CTRL |= USBFS_UEP_R_RES_ACK;
        #else
            SetEPRxValid( ENDP2 );
        #endif
	#endif
}
/*********************************************************************
 * @fn      EP3_IN_Callback
 *
 * @brief  Endpoint 3 IN.
 *
 * @return  none
 */
void EP3_IN_Callback (void)
{ 
	USBD_Endp3_Busy = 0;
	Uart.USB_Up_IngFlag = 0x00;
}

/*********************************************************************
 * @fn      USBD_ENDPx_DataUp
 *
 * @brief  USBD ENDPx DataUp Function
 * 
 * @param   endp - endpoint num.
 *          *pbuf - A pointer points to data.
 *          len - data length to transmit.
 * 
 * @return  data up status.
 */
uint8_t USBD_ENDPx_DataUp( uint8_t endp, uint8_t *pbuf, uint16_t len )
{
	if( endp == ENDP3 )
	{
		if (USBD_Endp3_Busy)
		{
			return USB_ERROR;
		}
		USB_SIL_Write( EP3_IN, pbuf, len );
		USBD_Endp3_Busy = 1;
		SetEPTxStatus( ENDP3, EP_TX_VALID );
	}
	else
	{
		return USB_ERROR;
	}
	return USB_SUCCESS;
}
