/**
 * Copyright (c) 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _THROUGHPUT_MEASURE_H__
#define _THROUGHPUT_MEASURE_H__

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_timer.h"
#include "app_util_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__
    #ifdef PACKED
        #undef PACKED
    #endif
    #define PACKED(TYPE) TYPE __attribute__ ((packed))
#endif


#define LIVE_ONE_SECOND_TIMEOUT_INTERVAL APP_TIMER_TICKS(1000)    /* Feed Time on the WatchDog, 10 seconds*/

typedef enum throughput_measure_evt_type_s
{
        eTHROUGHPUT_INIT_DONE,
        eTHROUGHPUT_LIVE,
        eTHROUGHPUT_RESET,
        eTHROUGHPUT_START,
        eTHROUGHPUT_STOP,
        eTHROUGHPUT_THRESHOLD,
        eTHROUGPPUT_SUMMARY,
} throughput_measure_evt_type_t;

typedef PACKED (struct throughput_measure_data_send_s
{
        uint32_t start_tick;
        uint32_t total_tick;
        uint32_t live_send_byte;
        uint32_t total_send_byte;
}) throughput_measure_data_send_t;

typedef PACKED (struct throughput_measure_data_receive_s
{
        uint32_t start_tick;
        uint32_t live_receive_byte;
        uint32_t total_tick;
        uint32_t total_receive_byte;
}) throughput_measure_data_receive_t;

typedef PACKED ( struct throughput_measure_evt_s {
        throughput_measure_data_send_t * p_send_data;
        throughput_measure_data_receive_t * p_receive_data;
        throughput_measure_evt_type_t evt_type;
}) throughput_measure_evt_t;

typedef void (*throughput_measure_evt_handler_t)(throughput_measure_evt_t * p_throughput_measure_evt);

typedef PACKED (struct throughput_measure_config_s
{
        uint8_t payload_length;
        uint16_t live_timer_interval;
        uint32_t receive_data_threshold;
        uint32_t send_data_threshold;
}) throughput_measure_config_t;

typedef PACKED (struct throughput_measure_init_s
{
        throughput_measure_config_t config;
        throughput_measure_evt_handler_t evt_handler;
}) throughput_measure_init_t;


void throughput_measure_reset_count(void);

void throughput_measure_receive_update(uint32_t received_byte);

void throughput_measure_send_update(uint32_t send_byte);

uint32_t throughput_measure_get_send_total(void);

void throughput_measure_reset_receive_count(void);

void throughput_measure_reset_send_count(void);

uint32_t throughput_measure_live_timer_start(void);

uint32_t throughput_measure_live_timer_stop(void);

uint32_t throughput_measure_init(throughput_measure_init_t *p_throughput_measure_init);

#endif //_THROUGHPUT_MEASURE_H__
