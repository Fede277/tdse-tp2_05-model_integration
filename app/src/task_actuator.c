/*
 * Copyright (c) 2023 Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file   : task_actuator.c
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/
/* Project includes */
#include "main.h"

/* Demo includes */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes */
#include "board.h"
#include "app.h"
#include "task_actuator_attribute.h"
#include "task_actuator_interface.h"

/********************** macros and definitions *******************************/
#define G_TASK_ACT_CNT_INIT         0ul
#define G_TASK_ACT_TICK_CNT_INI     0ul

#define DEL_LED_XX_PUL              250ul
#define DEL_LED_XX_BLI              500ul
#define DEL_LED_XX_MIN              0ul

/********************** internal data declaration ****************************/
const task_actuator_cfg_t task_actuator_cfg_list[] = {
    { ID_LED_A, LED_A_PORT, LED_A_PIN, LED_A_ON, LED_A_OFF,
      DEL_LED_XX_BLI, DEL_LED_XX_PUL }
};
#define ACTUATOR_CFG_QTY (sizeof(task_actuator_cfg_list)/sizeof(task_actuator_cfg_t))

task_actuator_dta_t task_actuator_dta_list[] = {
    { DEL_LED_XX_MIN, ST_LED_XX_OFF, EV_LED_XX_NOT_BLINK, false }
};
#define ACTUATOR_DTA_QTY (sizeof(task_actuator_dta_list)/sizeof(task_actuator_dta_t))

/********************** internal functions declaration ***********************/
void task_actuator_statechart(void);

/********************** internal data definition *****************************/
const char *p_task_actuator  = "Task Actuator (Actuator Statechart)";
const char *p_task_actuator_ = "Non-Blocking & Update By Time Code";

/********************** external data declaration ****************************/
uint32_t g_task_actuator_cnt;
volatile uint32_t g_task_actuator_tick_cnt;

/********************** external functions definition ************************/
void task_actuator_init(void *parameters)
{
    uint32_t index;
    const task_actuator_cfg_t *p_task_actuator_cfg;
    task_actuator_dta_t *p_task_actuator_dta;
    task_actuator_st_t state;
    task_actuator_ev_t event;
    bool b_event;

    /* Print out: Task Initialized */
    LOGGER_INFO(" ");
    LOGGER_INFO("  %s is running - %s", GET_NAME(task_actuator_init), p_task_actuator);
    LOGGER_INFO("  %s is a %s", GET_NAME(task_actuator), p_task_actuator_);

    /* Init & Print out: Task execution counter */
    g_task_actuator_cnt = G_TASK_ACT_CNT_INIT;
    LOGGER_INFO("   %s = %lu", GET_NAME(g_task_actuator_cnt), g_task_actuator_cnt);

    for (index = 0; ACTUATOR_DTA_QTY > index; index++)
    {
        /* Update Task Actuator Configuration & Data Pointer */
        p_task_actuator_cfg = &task_actuator_cfg_list[index];
        p_task_actuator_dta = &task_actuator_dta_list[index];

        /* Init & Print out: Index & Task execution FSM */
        state = ST_LED_XX_OFF;
        p_task_actuator_dta->state = state;

        event = EV_LED_XX_OFF;
        p_task_actuator_dta->event = event;

        b_event = false;
        p_task_actuator_dta->flag = b_event;

        LOGGER_INFO(" ");
        LOGGER_INFO("   %s = %lu   %s = %lu   %s = %lu   %s = %s",
                    GET_NAME(index), index,
                    GET_NAME(state), (uint32_t)state,
                    GET_NAME(event), (uint32_t)event,
                    GET_NAME(b_event), (b_event ? "true" : "false"));

        HAL_GPIO_WritePin(p_task_actuator_cfg->gpio_port,
                          p_task_actuator_cfg->pin,
                          p_task_actuator_cfg->led_off);
    }
}

void task_actuator_update(void *parameters)
{
    bool b_time_update_required = false;

    /* Protect shared resource */
    __asm("CPSID i"); /* disable interrupts*/
    if (G_TASK_ACT_TICK_CNT_INI < g_task_actuator_tick_cnt)
    {
        /* Update Tick Counter */
        g_task_actuator_tick_cnt--;
        b_time_update_required = true;
    }
    __asm("CPSIE i"); /* enable interrupts */

    while (b_time_update_required)
    {
        /* Update Task Counter */
        g_task_actuator_cnt++;

        /* Run Task Statechart */
        task_actuator_statechart();

        /* Protect shared resource */
        __asm("CPSID i"); /* disable interrupts */
        if (G_TASK_ACT_TICK_CNT_INI < g_task_actuator_tick_cnt)
        {
            /* Update Tick Counter */
            g_task_actuator_tick_cnt--;
            b_time_update_required = true;
        }
        else
        {
            b_time_update_required = false;
        }
        __asm("CPSIE i"); /* enable interrupts */
    }
}

void task_actuator_statechart(void)
{
    uint32_t index;
    const task_actuator_cfg_t *cfg;
    task_actuator_dta_t *dta;

    for (index = 0; ACTUATOR_DTA_QTY > index; index++)
    {
        cfg = &task_actuator_cfg_list[index];
        dta = &task_actuator_dta_list[index];

        switch (dta->state)
        {
            /* ===================== OFF ===================== */
            case ST_LED_XX_OFF:
                if (dta->flag)
                {
                    dta->flag = false;

                    if (dta->event == EV_LED_XX_ON) {
                        /* OFF -> ON */
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                        dta->state = ST_LED_XX_ON;
                    }
                    else if (dta->event == EV_LED_XX_BLINK) {
                        /* OFF -> BLINK_ON (entry: ON, tick = tick_blink) */
                        dta->tick = cfg->tick_blink;
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                        dta->state = ST_LED_XX_BLINK_ON;
                    }
                    else if (dta->event == EV_LED_XX_PULSE) {
                        /* OFF -> PULSE (entry: ON, tick = tick_pulse) */
                        dta->tick = cfg->tick_pulse;
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                        dta->state = ST_LED_XX_PULSE;
                    }
                    else if (dta->event == EV_LED_XX_OFF) {
                        /* idempotente: permanecer en OFF */
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_off);
                        dta->state = ST_LED_XX_OFF;
                    }
                }
                break;

            /* ===================== ON ====================== */
            case ST_LED_XX_ON:
                if (dta->flag)
                {
                    dta->flag = false;

                    if (dta->event == EV_LED_XX_OFF) {
                        /* ON -> OFF */
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_off);
                        dta->state = ST_LED_XX_OFF;
                    }
                    else if (dta->event == EV_LED_XX_BLINK) {
                        /* ON -> BLINK_ON (entry: ON, tick = tick_blink) */
                        dta->tick = cfg->tick_blink;
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                        dta->state = ST_LED_XX_BLINK_ON;
                    }
                    else if (dta->event == EV_LED_XX_PULSE) {
                        /* ON -> PULSE (entry: ON, tick = tick_pulse) */
                        dta->tick = cfg->tick_pulse;
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                        dta->state = ST_LED_XX_PULSE;
                    }
                    else if (dta->event == EV_LED_XX_ON) {
                        /* idempotente: permanecer en ON */
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                        dta->state = ST_LED_XX_ON;
                    }
                }
                break;

            /* ================ BLINK (fase ON) =============== */
            case ST_LED_XX_BLINK_ON:
                if (dta->flag)
                {
                    dta->flag = false;

                    if (dta->event == EV_LED_XX_OFF) {
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_off);
                        dta->state = ST_LED_XX_OFF;
                        break;
                    }
                    if (dta->event == EV_LED_XX_ON) {
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                        dta->state = ST_LED_XX_ON;
                        break;
                    }
                }

                if (dta->tick > DEL_LED_XX_MIN) {
                    dta->tick--;
                } else {
                    /* toggle -> ir a fase OFF */
                    HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_off);
                    dta->tick  = cfg->tick_blink;
                    dta->state = ST_LED_XX_BLINK_OFF;
                }
                break;

            /* =============== BLINK (fase OFF) =============== */
            case ST_LED_XX_BLINK_OFF:
                if (dta->flag)
                {
                    dta->flag = false;

                    if (dta->event == EV_LED_XX_OFF) {
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_off);
                        dta->state = ST_LED_XX_OFF;
                        break;
                    }
                    if (dta->event == EV_LED_XX_ON) {
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                        dta->state = ST_LED_XX_ON;
                        break;
                    }
                }

                if (dta->tick > DEL_LED_XX_MIN) {
                    dta->tick--;
                } else {
                    /* toggle -> ir a fase ON */
                    HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                    dta->tick  = cfg->tick_blink;
                    dta->state = ST_LED_XX_BLINK_ON;
                }
                break;

            /* ==================== PULSE ===================== */
            case ST_LED_XX_PULSE:
                /* permitir abortar el pulso con ON/OFF/BLINK */
                if (dta->flag)
                {
                    dta->flag = false;

                    if (dta->event == EV_LED_XX_OFF) {
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_off);
                        dta->state = ST_LED_XX_OFF;
                        break;
                    }
                    if (dta->event == EV_LED_XX_ON) {
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                        dta->state = ST_LED_XX_ON;
                        break;
                    }
                    if (dta->event == EV_LED_XX_BLINK) {
                        dta->tick = cfg->tick_blink;
                        HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_on);
                        dta->state = ST_LED_XX_BLINK_ON;
                        break;
                    }
                }

                /* duración del pulso */
                if (dta->tick > DEL_LED_XX_MIN) {
                    dta->tick--;
                } else {
                    HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_off);
                    dta->state = ST_LED_XX_OFF;
                }
                break;

            /* =============== DEFAULT / RESET ================= */
            default:
                dta->tick  = DEL_LED_XX_MIN;
                dta->state = ST_LED_XX_OFF;
                dta->event = EV_LED_XX_OFF;
                dta->flag  = false;
                HAL_GPIO_WritePin(cfg->gpio_port, cfg->pin, cfg->led_off);
                break;
        }
    }
}

/********************** end of file *******************************************/

