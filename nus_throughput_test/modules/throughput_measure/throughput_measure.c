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

#include "throughput_measure.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"





static throughput_measure_data_send_t m_throughput_measure_send_data;
static throughput_measure_data_receive_t m_throughput_measure_receive_data;
static throughput_measure_config_t m_throughput_measure_config;
static throughput_measure_evt_handler_t evt_handler;

static bool m_live_timer_is_running = false;
static bool m_live_timer_is_init = false;

APP_TIMER_DEF(m_live_update_timer_id);

/**
 *
 *
 *
 **/

static void throughput_measure_live_summary(void)
{
//        double time_ms_float = (m_throughput_measure_send_data.diff_tick) * 2000 / 32768;
        uint32_t time_ms = 1000;

        uint32_t bit_count = (m_throughput_measure_send_data.live_send_byte) * 8;
        float throughput_kbps = ((bit_count / (time_ms / 1000.f)) / 1000.f);

        NRF_LOG_DEBUG("Time: %u.%.2u seconds elapsed.", (time_ms / 1000), (time_ms % 1000));
        NRF_LOG_INFO("Throughput: " NRF_LOG_FLOAT_MARKER " Kbps.",
                     NRF_LOG_FLOAT(throughput_kbps));

        // reset the counter;
        m_throughput_measure_send_data.live_send_byte = 0;
}

static void throughput_measure_threshold_summary(void)
{
        ret_code_t ret_code;
        m_throughput_measure_send_data.total_tick = app_timer_cnt_get();
        double time_ms_float = (m_throughput_measure_send_data.total_tick - m_throughput_measure_send_data.start_tick) * 2000 / 32768;
        uint32_t time_ms = (uint32_t) time_ms_float;

        uint32_t bit_count = (m_throughput_measure_send_data.total_send_byte)*8;
        float throughput_kbps = ((bit_count / (time_ms / 1000.f)) / 1000.f);

        NRF_LOG_INFO("From = %08x, To = %08x, Diff = %08d", m_throughput_measure_send_data.start_tick, m_throughput_measure_send_data.total_tick, time_ms);

        //NRF_LOG_INFO("Meet the threshold %08x", m_throughput_measure_config.send_data_threshold);
        NRF_LOG_INFO("=============================");
        NRF_LOG_INFO("Time: %u.%.2u seconds elapsed.", (time_ms / 1000), (time_ms % 1000));
        NRF_LOG_INFO("Throughput: " NRF_LOG_FLOAT_MARKER " Kbps.",
                     NRF_LOG_FLOAT(throughput_kbps));
        NRF_LOG_INFO("=============================");
        NRF_LOG_INFO("Sent %u KBytes of ATT payload.", m_throughput_measure_send_data.total_send_byte/1024);
        NRF_LOG_INFO("STOP Timer!!");

        throughput_measure_evt_t evt;
        evt.evt_type = eTHROUGHPUT_THRESHOLD;
        evt.p_receive_data = &m_throughput_measure_receive_data;
        evt.p_send_data = &m_throughput_measure_send_data;
        evt_handler(&evt);
}

static void live_timeout_event_handler(void)
{
        ASSERT(evt_handler);
        throughput_measure_evt_t evt;
        evt.evt_type = eTHROUGHPUT_LIVE;
        evt.p_receive_data = &m_throughput_measure_receive_data;
        evt.p_send_data = &m_throughput_measure_send_data;
        evt_handler(&evt);

        throughput_measure_live_summary();
}

static void throughput_live_timer_start(void)
{
        ret_code_t err_code;
        if (!m_live_timer_is_running)
        {
                err_code = app_timer_start(m_live_update_timer_id, LIVE_ONE_SECOND_TIMEOUT_INTERVAL, NULL);
                APP_ERROR_CHECK(err_code);
                m_live_timer_is_running = true;
                m_throughput_measure_send_data.start_tick = app_timer_cnt_get();
        }
}

static void throughput_live_timer_stop(void)
{
        ret_code_t err_code;
        if (m_live_timer_is_running)
        {
                err_code = app_timer_stop(m_live_update_timer_id);
                APP_ERROR_CHECK(err_code);
                m_live_timer_is_running = false;
        }
}


static void throughput_live_timer_create(void)
{
        ret_code_t err_code;
        if (m_live_timer_is_init != true)
        {
                err_code = app_timer_create(&m_live_update_timer_id, APP_TIMER_MODE_REPEATED, live_timeout_event_handler);
                APP_ERROR_CHECK(err_code);
                m_live_timer_is_init = true;
        }
        else
        {
                APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
        }
}


uint32_t throughput_measure_get_send_total(void)
{
        return m_throughput_measure_send_data.total_send_byte;
}

uint32_t throughput_measure_get_receive_total(void)
{
        return m_throughput_measure_receive_data.total_receive_byte;
}

void throughput_measure_send_update(uint32_t send_byte)
{
        m_throughput_measure_send_data.total_send_byte += send_byte;
        m_throughput_measure_send_data.live_send_byte += send_byte;

        if (m_throughput_measure_send_data.total_send_byte >= m_throughput_measure_config.send_data_threshold)
        {
                ASSERT(evt_handler);
                throughput_measure_evt_t evt;
                evt.evt_type = eTHROUGHPUT_THRESHOLD;
                evt.p_receive_data = &m_throughput_measure_receive_data;
                evt.p_send_data = &m_throughput_measure_send_data;
                evt_handler(&evt);
                throughput_measure_live_timer_stop();
                throughput_measure_threshold_summary();
        }
}

void throughput_measure_receive_update(uint32_t received_byte)
{
        m_throughput_measure_receive_data.total_receive_byte += received_byte;
        m_throughput_measure_receive_data.live_receive_byte += received_byte;

        if (m_throughput_measure_receive_data.total_receive_byte >= m_throughput_measure_config.receive_data_threshold)
        {
                ASSERT(evt_handler);
                throughput_measure_evt_t evt;
                evt.evt_type = eTHROUGHPUT_THRESHOLD;
                evt.p_receive_data = &m_throughput_measure_receive_data;
                evt.p_send_data = &m_throughput_measure_send_data;
                evt_handler(&evt);
                throughput_measure_live_timer_stop();
                throughput_measure_threshold_summary();
        }
}

void throughput_measure_reset_receive_count(void)
{
        memset(&m_throughput_measure_receive_data, 0, sizeof(m_throughput_measure_receive_data));
}

void throughput_measure_reset_send_count(void)
{
        memset(&m_throughput_measure_send_data, 0, sizeof(m_throughput_measure_send_data));
}

uint32_t throughput_measure_live_timer_start(void)
{
        throughput_live_timer_start();
        throughput_measure_evt_t evt;
        evt.evt_type = eTHROUGHPUT_START;
        evt_handler(&evt);
}

uint32_t throughput_measure_live_timer_stop(void)
{
        throughput_live_timer_stop();
        throughput_measure_evt_t evt;
        evt.evt_type = eTHROUGHPUT_STOP;
        evt_handler(&evt);
}

uint32_t throughput_measure_init(throughput_measure_init_t *p_throughput_measure_init)
{
        //ASSERT(p_throughput_measure_init->evt_handler == NULL);
        evt_handler = p_throughput_measure_init->evt_handler;

        memset(&m_throughput_measure_receive_data, 0, sizeof(m_throughput_measure_receive_data));

        m_throughput_measure_config.payload_length = p_throughput_measure_init->config.payload_length;
        m_throughput_measure_config.live_timer_interval = p_throughput_measure_init->config.live_timer_interval;
        m_throughput_measure_config.receive_data_threshold = p_throughput_measure_init->config.receive_data_threshold;
        m_throughput_measure_config.send_data_threshold = p_throughput_measure_init->config.send_data_threshold;

        m_live_timer_is_running = false;

        /* Create the live timer */
        throughput_live_timer_create();
}
