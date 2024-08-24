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
 *  file         : ch32v_ftoa_atof.h
 *  description  : ftoa atof library main header
 *
 */

#ifndef CH32V_FTOA_ATOF_H_
#define CH32V_FTOA_ATOF_H_

#include "debug.h"
#include "ftoa.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* ++++++++++++++++++++ USER CONFIG AREA BEGIN ++++++++++++++++++++ */
#define TMP_FBUF_SIZE       128                 /* Buffer size to temporary store float strings after conversion in ftoa_s() */
#define TMP_FSTR_SIZE       16                  /* Length of individial strings of floats (num of floats stored at once = TMP_FBUF_SIZE / TMP_FSTR_SIZE) */
#define TMP_FSTR_NUM        TMP_FBUF_SIZE / TMP_FSTR_SIZE   /* default is 8 slots for temporary float strings (automatically overwritten on overflow) */
/* ++++++++++++++++++++ USER CONFIG AREA END ++++++++++++++++++++ */


extern char * ftoa_s(double val, int precision);
extern double ratof(char *arr);
extern float ratoff(char *arr);


#ifdef __cplusplus
}
#endif

#endif /* CH32V_FTOA_ATOF_H_ */