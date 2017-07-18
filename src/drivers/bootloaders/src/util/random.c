/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "random.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define RAND_MAX_32 ((1U << 31) - 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static uint32_t rseed;


/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: util_srand
 *
 * Description:
 *   This function seeds the random number generator
 *
 *
 * Input Parameters:
 *   seed    - The seed
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/
void util_srand(uint16_t seed)
{
	rseed = seed;
}


/****************************************************************************
 * Name: util_random
 *
 * Description:
 *   This function returns a random number between min and max
 *
 *
 * Input Parameters:
 *   min    - The minimum value the return value can be.
 *   max    - The maximum value the return value can be.
 *
 * Returned value:
 *   A random number
 *
 ****************************************************************************/

uint16_t util_random(uint16_t min, uint16_t max)
{
	uint16_t rand = (rseed = (rseed * 214013 + 2531011) & RAND_MAX_32) >> 16;
	return rand % (max - min) + min;
}
