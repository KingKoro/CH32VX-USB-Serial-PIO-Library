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
 *  file         : main.c
 *  description  : example main code
 *
 */
#if defined(CH32V00X)
#include <ch32v00x.h>
#elif defined(CH32V10X)
#include <ch32v10x.h>
#elif defined(CH32V20X)
#include <ch32v20x.h>
#elif defined(CH32V30X)
#include <ch32v30x.h>
#elif defined(CH32X035) || defined(CH32X033)
#include <ch32x035.h>
#endif

#include "ch32v_usb_serial.h"
#include "ch32v_ftoa_atof.h"

#define TRUE 1
#define FALSE 0

int main()
{
    // ---------- Initialization Code ----------
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("CH32V USB CDC Test - Starting ...\r\n");
    // Setup Code USB Serial
    USB_Serial_initialize();
    // Variables for testing CDC
    uint32_t counter1s = 0;
    char testchar = 'A';
    char teststring[] = "STRING";
    int testint = 187;
    double testfloatA = 456321.7651234;
    double testfloatB = -3.14159;
    char testfloatC[16];
    uint32_t testhex = 0xFDCA8421;
    // ---------- Endless Loop Code ----------
    while ( 1 )
    {
        if(counter1s >= 2000000)
        {
            // Call roughly every ~second
            counter1s = 0;
            printf("1s Loop ...\r\n");
            USB_Serial_printf("This is a char: %c,\tThis is a string: %s,\tThis is an integer: %d,\tThis is a float: %s or %s,\tThis is a hex value (32bit): %x \tThis is a percent sign: %%\n", testchar, teststring, testint, ftoa_s(testfloatA, 6), ftoa_s(testfloatB, 5), testhex);
            USB_Serial_printf("This is negative pi with 2 decimals: %s and now 4 decimals: %s \n", ftoa_s(testfloatB, 2), ftoa_s(testfloatB, 4));
            
            int hours = 0;
            int minutes = 0;
            int seconds = 0;
            USB_Serial_printf("Please enter a time in format \"HH:MM:SS\" : ");
            int status_scanf = USB_Serial_scanf(TRUE, "%d:%d:%d", &hours, &minutes, &seconds);
            USB_Serial_printf("Received hours: %d !, statuscode: %d \n", hours, status_scanf);
            USB_Serial_printf("Received minutes: %d !, statuscode: %d \n", minutes, status_scanf);
            USB_Serial_printf("Received seconds: %d !, statuscode: %d \n", seconds, status_scanf);
            USB_Serial_printf("Please enter a floating point value: ");
            status_scanf = USB_Serial_scanf(TRUE, "%s", &testfloatC);
            double received_float = ratof(testfloatC);
            //double received_float = strtod(testfloatC, NULL);
            USB_Serial_printf("Received float: %s !, statuscode: %d \n", ftoa_s(received_float, 6), status_scanf);
            
        }
        // Call on every loop
        counter1s++;
        //USB_Tx_runner();
    }
}
