/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file modified from sf0x_parser.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Chuong Nguyen <chnguye7@asu.edu>
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 *
 * Declarations of parser for the Benewake TFmini laser rangefinder series
 */

#include "ri080_parser.h"

#include <stdlib.h>
#include <string.h>

// #define RI080_DEBUG

#ifdef RI080_DEBUG
#include <stdio.h>

const char *parser_state[] = {"0_UNSYNC",      "1_SYNC_1",         "2_SYNC_2",         "3_GOT_DIST_L",
                              "4_GOT_DIST_H",  "5_GOT_STRENGTH_L", "6_GOT_STRENGTH_H", "7_GOT_PRESERVED",
                              "8_GOT_QUALITY", "9_GOT_CHECKSUM"};
#endif

int ri080_parse(char c, char *parserbuf, unsigned *parserbuf_index, RI080_PARSE_STATE *state, float *dist) {
    int ret = -1;
    // char *end;

    switch (*state) {
        case RI080_PARSE_STATE::STATE1_HEAD:
            if (c == RI080_PROTOCOL_HEAD) {
                *state = RI080_PARSE_STATE::STATE2_DEVICE_ID;
                parserbuf[*parserbuf_index] = c;
                (*parserbuf_index)++;

            } else {
                *state = RI080_PARSE_STATE::STATE1_HEAD;
                *parserbuf_index = 0;
            }

            break;

        case RI080_PARSE_STATE::STATE2_DEVICE_ID:
            if (c == RI080_PROTOCOL_DEVICE_ID) {
                *state = RI080_PARSE_STATE::STATE3_SYS_ID;
                parserbuf[*parserbuf_index] = c;
                (*parserbuf_index)++;
            } else {
                *state = RI080_PARSE_STATE::STATE1_HEAD;
                *parserbuf_index = 0;
            }
            break;

        case RI080_PARSE_STATE::STATE3_SYS_ID:
            if (c == RI080_PROTOCOL_SYS_ID) {
                *state = RI080_PARSE_STATE::STATE4_MSG_ID;
                parserbuf[*parserbuf_index] = c;
                (*parserbuf_index)++;

            } else {
                *state = RI080_PARSE_STATE::STATE1_HEAD;
                *parserbuf_index = 0;
            }

            break;

        case RI080_PARSE_STATE::STATE4_MSG_ID:
            *state = RI080_PARSE_STATE::STATE5_NUM_INDEX;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;

            break;

        case RI080_PARSE_STATE::STATE5_NUM_INDEX:
            *state = RI080_PARSE_STATE::STATE6_PAYLOAD_LENS;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;

            break;

        case RI080_PARSE_STATE::STATE6_PAYLOAD_LENS:
            *state = RI080_PARSE_STATE::STATE7_PAYLOAD_DISTANS_L;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;

            break;

        case RI080_PARSE_STATE::STATE7_PAYLOAD_DISTANS_L:
            *state = RI080_PARSE_STATE::STATE8_PAYLOAD_DISTANS_H;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;

            break;

        case RI080_PARSE_STATE::STATE8_PAYLOAD_DISTANS_H:
            *state = RI080_PARSE_STATE::STATE9_PAYLOAD_STRENGTH;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;

            break;

        case RI080_PARSE_STATE::STATE9_PAYLOAD_STRENGTH:
            *state = RI080_PARSE_STATE::STATE10_PAYLOAD_RESERVED;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;

            break;

        case RI080_PARSE_STATE::STATE10_PAYLOAD_RESERVED:
            *state = RI080_PARSE_STATE::STATE11_PAYLOAD_CHECKSUM;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;

            break;

        case RI080_PARSE_STATE::STATE11_PAYLOAD_CHECKSUM:
            unsigned char cksm = 0;
            int distance_index_l = static_cast<int>(RI080_PARSE_STATE::STATE7_PAYLOAD_DISTANS_L);
            int distance_index_h = static_cast<int>(RI080_PARSE_STATE::STATE8_PAYLOAD_DISTANS_H);
            for (unsigned int i = 0; i < *parserbuf_index; i++) {
                cksm += parserbuf[i];
            }
            if (cksm == c) {
                *dist = (parserbuf[distance_index_l] + parserbuf[distance_index_h] * 256)/1000.0f;
		ret = 0;
            }
            *state = RI080_PARSE_STATE::STATE1_HEAD;
            *parserbuf_index = 0;
            break;
    }

#ifdef RI080_DEBUG
    printf("state: RI080_PARSE_STATE%s\n", parser_state[*state]);
#endif

    return ret;
}
