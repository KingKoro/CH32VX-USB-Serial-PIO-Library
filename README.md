# CH32VX USB Serial PIO Library
This repository contains a CH32Vx/CH32X0x PIO NoneOS library for serial communication over usb ports. It is supposed to work with CH32V10X, CH32V20X, CH32V30X, CH32X033 and CH32X035 MCUs and is based on the SimulateCDC drivers, provided in the [OpenWCH](https://github.com/openwch) examples.

# Warning
Currently this library does not yet work on CH32V30X MCUs.

# Installation
## Prequisites
You need to have PlatformIO VSCode Plugin with the [WCH CH32V](https://github.com/Community-PIO-CH32V/platform-ch32v) Platform installed.

## Setup
Simply clone this repository onto your computer and open the folder like a regular PlatformIO project. You can set the build target by choosing one from the ```platformio.ini``` file or adding your own one for the specific MCU model. Just take the predefined environments as an example.

Alternativly, you can also copy the library ```lib/CH32V_USB_SERIAL``` into any PIO project, modify your ```platformio.ini```, add the required ```ch32xxxx_it``` code for the timer interrupt handlers (recommended for async tx mode) and import the library into your ```main.c``` with: 
```c
#include "ch32v_usb_serial.h"
```

# Usage

Import ```ch32v_usb_serial.h```, then modify the ```platform.ini``` environment by specifying the serial monitor speed, flow control, build source filters and build flags. The build flags are important and tell the compiler which family of MCUs is the build target. The provided ```platform.ini``` gives a good example. E.g. ```build_flags = -DHAL=CH32V20X``` for selecting the CH32V20x family.

Afterwards, edit the user config section (line 82 to 92) in ```lib/CH32V_USB_SERIAL/ch32v_usb_serial.h``` to control the serial port settings. The following settings are available:

|       Option:       | Description:                                                           |
|:-----------------:|-----------------------------------------------------------------|
| USB_CDCPORT           | For MCUs with multiple USB-ports select which one to use (```CDC_USBD``` or ```CDC_USBFS```). For CH32V30x, USBD means USB-HS. For CH32X03x and CH32V10x, only USBFS is a valid option.       |
| USB_CDC_BAUDRATE         | Set the baudrate. For very high speeds above 500kBaud/s, consider upping the timer speed in TX asynchronous mode. The maximum speed tested is 1MBaud/s. Default = 1000000           |
| USB_CDC_STOPBIT       | Select USB Port Stopbit. Default = 0 |
| USB_CDC_PARITY     | Select USB Port Paritybit. Default = 0   |
| USB_TX_MODE | Select USB TX send mode (either ```USB_TX_SYNC``` for synchronous or ```USB_TX_ASYNC```for asynchronous). In synchronous mode, printf() will wait for usb availability and return after data was sent. In asynchronous mode, printf() returns immediately, and only writes into a send buffer, which needs to be periodically read from and actually written into the usb port. This can be achieved by either calling ```USB_Tx_runner()``` in an endless while-loop or using a timer interrupt routine to call ```USB_Tx_runner()``` periodically (default). Timer 2 is used for this purpose, except for CH32X03x, which uses Timer 3. These Timer handlers are declared and utilized within this library already, therefore, to avoid confusion and compatibility issues, it is advised to not use these Timers for anything else. If you wish to supply your own handler for Timer 2 or Timer 3 anyways, remove the comment on line 93 ```#define EXT_USB_TIM_HANDLER``` in ``ch32v_usb_serial.h`` and define the IRQHandler externally. Make sure to include the same code, invoking ``USB_Tx_runner()``, in the IRQHandler's body. See ``ch32v_usb_serial.c`` for reference.      |
| USB_RX_OVERFLOW           | Select USB Port receive overflow mode. Select either ```USB_RX_HALT``` (default) to stop accepting USB traffic if the receive buffer is almost full and not being processed fast enough, or ```USB_RX_OVERWRITE``` to ignore overflow and overwrite older data.    |
| GETCH_CLI_FEEDBACK         | Wether to display feedback into console when typing into. Comment out if not desired. Default = on                |
| TMP_FBUF_SIZE         | Buffer size to temporary store float strings after conversion in ftoa_s(). Default = 128 Bytes |
| TMP_FSTR_SIZE         | Length of individial strings of floats (num of floats stored at once = TMP_FBUF_SIZE / TMP_FSTR_SIZE). Default = 16 Bytes |
| TMP_FSTR_NUM          | TMP_FBUF_SIZE / TMP_FSTR_SIZE   default is 8 slots for temporary float strings (automatically overwritten on overflow) |
| USB_SCANF_BUF_SIZE    | Maximum USB RX buffer size for scanf() before matching. Default = 512 Bytes |
| EXT_USB_TIM_HANDLER   | Remove comment to supply the USB asynchronous TX Timer IRQHandler yourself. Useful if you need that timer to do something else as well. Default = Commented out |


Now you can compile and upload the project.

## Overview

The most important functions, exposed by the USB serial library, are:
```C
void USB_Serial_initialize()    /* USB serial port initialization */

void USB_Rx_flush()             /* Flush and reset USB RX buffer */

void _putchar(char character)   /* putchar() */
int getch()                     /* getchar() */

uint16_t USB_Serial_printf(char *format, ...)   /* printf() */
int USB_Serial_scanf(uint8_t block, char *format, ...)  /* scanf() */

char * ftoa_s(double val, int precision)    /* compact ftoa() */
double ratof(char *arr)     /* atof() double */
float ratoff(char *arr)     /* atof() float */
```

# Notes

## Floating Point values
As of right now, the standard library for CH32V MCUs does not handle string-to-float and float-to-string conversion correctly. Therefore this library contains an adapted ftoa implementation by [Anton B. Gusev](https://github.com/antongus/stm32tpl/blob/master/ftoa.c).
The library additionally contains an atof implementation.

When using ```USB_Serial_scanf()``` and ```USB_Serial_printf()```, floating point values have to be passed as strings and then converted to double or float. To achive this in a more compact manner, ```char * ftoa_s(double val, int precision)``` is an inline version of ftoa for use in the same line as the printf statement. For an example, look at the proivded ```src/main.c```.

## 64-Bit Integers
As of right now, the standard library for CH32V MCUs does not handle string-to-(U)INT64 and (U)INT64-to-string conversion correctly. Therefore a custom ``atoi()`` and ``itoa()`` function needs to be supplied.

## Supported MCUs
This library was only tested on the CH32V203C8T6-EVT-R0 (both USBD and USBFS interfaces) and CH32X035C8T6-EVT-R0. However, it should work for the CH32V10X family as well.

# Disclaimer

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
