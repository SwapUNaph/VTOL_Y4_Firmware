/****************************************************************************
 * examples/micropython/pexec.h
 *
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ****************************************************************************/

#ifndef __APPS_INTERPRETERS_MICROPYTHON_PYEXEC_H
#define __APPS_INTERPRETERS_MICROPYTHON_PYEXEC_H 1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PYEXEC_FORCED_EXIT (0x100)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  PYEXEC_MODE_RAW_REPL,
  PYEXEC_MODE_FRIENDLY_REPL,
} pyexec_mode_kind_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern pyexec_mode_kind_t pyexec_mode_kind;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int pyexec_raw_repl(void);
int pyexec_friendly_repl(void);
int pyexec_file(const char *filename);

MP_DECLARE_CONST_FUN_OBJ(pyb_set_repl_info_obj);

#endif /* __APPS_INTERPRETERS_MICROPYTHON_PYEXEC_H */
