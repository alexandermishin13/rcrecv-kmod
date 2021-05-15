/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2021 Alexander Mishin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _RCRECV_KMOD_H_
#define _RCRECV_KMOD_H_

struct rcrecv_code {
    unsigned long value;
    size_t bit_length;
    size_t proto;
    bool ready;
};

typedef struct {
    uint8_t high;
    uint8_t low;
} levels_ratio;

typedef struct {
    size_t pulse_length;
    levels_ratio sync_factor;
    levels_ratio zero;
    levels_ratio one;
    bool inverted;
} protocol;

static protocol proto[] = {
    { 350, {1,31},  {1,3},  {3,1},  false }, // protocol 1
    { 650, {1,10},  {1,2},  {2,1},  false }, // protocol 2
    { 100, {30,71}, {4,11}, {9,6},  false }, // protocol 3
    { 380, {1,6},   {1,3},  {3,1},  false }, // protocol 4
    { 500, {6,14},  {1,2},  {2,1},  false }, // protocol 5
    { 450, {23,1},  {1,2},  {2,1},  true },  // protocol 6 (HT6P20B)
    { 150, {2,62},  {1,6},  {6,1},  false }, // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
    { 200, {3,130}, {7,16}, {3,16}, false }, // protocol 8 Conrad RS-200 RX
    { 200, {130,7}, {16,7}, {16,3}, true },  // protocol 9 Conrad RS-200 TX
    { 365, {18,1},  {3,1},  {1,3},  true },  // protocol 10 (1ByOne Doorbell)
    { 270, {36,1},  {1,2},  {2,1},  true },  // protocol 11 (HT12E)
    { 320, {36,1},  {1,2},  {2,1},  true }   // protocol 12 (SM5212)
};

#define RCRECV_READ_CODE      _IOR('R', 10, unsigned long)
#define RCRECV_READ_CODE_INFO _IOR('R', 11, struct rcrecv_code)

#endif /* _RCRECV_KMOD_H_ */
