/**
 *  CH32VX FTOA ATOF Library
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
 *  file         : ch32v_ftoa_atof.c
 *  description  : ftoa atof library main code
 *
 */

#include "ch32v_ftoa_atof.h"


// Track temporary float string-buffer
uint16_t tmp_fbuf_pos = 0;                  // position in buffer
char tmp_fbuf[TMP_FBUF_SIZE];               // temporary float string-buffer = 8 slots x 16 chars


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