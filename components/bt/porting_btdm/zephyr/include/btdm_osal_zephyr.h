/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _BTDM_OSAL_ZEPHYR_H_
#define _BTDM_OSAL_ZEPHYR_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "esp_timer.h"
#include "btdm_osal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Zephyr counterparts of the btdm_osal_*_freertos objects. Callouts always
 * use esp_timer, as ESP-IDF does with CONFIG_BT_LE_USE_ESP_TIMER, so a
 * btdm_osal time unit is one millisecond.
 */

struct btdm_osal_event_zephyr {
    void *fifo_reserved;            /* k_queue link, must be first */
    struct btdm_osal_event *parent; /* the object handed out by eventq_get() */
    bool queued;
    btdm_osal_event_fn *fn;
    void *arg;
};

struct btdm_osal_eventq_zephyr {
    struct k_queue q;
};

struct btdm_osal_callout_zephyr {
    esp_timer_handle_t handle;
    struct btdm_osal_eventq *evq;
    struct btdm_osal_event ev;
};

struct btdm_osal_mutex_zephyr {
    struct k_mutex handle;
};

struct btdm_osal_sem_zephyr {
    struct k_sem handle;
};

#ifdef __cplusplus
}
#endif

#endif /* _BTDM_OSAL_ZEPHYR_H_ */
