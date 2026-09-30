/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Zephyr implementation of the BTDM controller OSAL (btdm_osal.h).
 *
 * This mirrors porting_btdm/controller/btdm_common/src/btdm_osal_freertos.c
 * from ESP-IDF v6.1: OSAL objects are carved from btdm mempools (whose
 * implementation lives in libbtdm_common.a), so an object pointer handed in by
 * the controller is only reused when it really comes from the pool, and
 * callouts are esp_timer based (one OSAL tick is one millisecond).
 */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "btdm_osal_zephyr.h"
#include "btdm_user_cfg.h"
#include "btdm_mempool.h"

#include "esp_attr.h"
#include "esp_mac.h"
#include "esp_intr_alloc.h"
#include "esp_random.h"
#include "esp_rom_sys.h"

LOG_MODULE_REGISTER(btdm_osal, CONFIG_LOG_DEFAULT_LEVEL);

#define BTDM_OSAL_ASSERT(con)                                                                      \
    do {                                                                                           \
        if (!(con)) {                                                                              \
            esp_rom_printf("assertion:%s\n", #con);                                                \
            esp_rom_printf("line:%d,function:%s\n", __LINE__, __func__);                           \
            assert(0);                                                                             \
        }                                                                                          \
    } while (0)

#define BTDM_OSAL_TIME_FOREVER (0xFFFFFFFFUL)

#if !defined(CONFIG_ESP32_BT_CONTROLLER_TASK_PRIO)
#define CONFIG_ESP32_BT_CONTROLLER_TASK_PRIO 2
#endif

/* The controller creates a single task of CONFIG_BT_CTRL_TASK_STACK_SIZE
 * bytes; keep one spare slot.
 */
#define BTDM_OSAL_TASK_NUM 2
static K_KERNEL_STACK_ARRAY_DEFINE(s_btdm_osal_task_stacks, BTDM_OSAL_TASK_NUM,
                                   CONFIG_BT_CTRL_TASK_STACK_SIZE);
static struct k_thread s_btdm_osal_task_threads[BTDM_OSAL_TASK_NUM];
static bool s_btdm_osal_task_used[BTDM_OSAL_TASK_NUM];

struct btdm_mempool s_btdm_osal_ev_pool;
static btdm_membuf_t *s_btdm_osal_ev_buf = NULL;

struct btdm_mempool s_btdm_osal_evq_pool;
static btdm_membuf_t *s_btdm_osal_evq_buf = NULL;

struct btdm_mempool s_btdm_osal_co_pool;
static btdm_membuf_t *s_btdm_osal_co_buf = NULL;

struct btdm_mempool s_btdm_osal_sem_pool;
static btdm_membuf_t *s_btdm_osal_sem_buf = NULL;

struct btdm_mempool s_btdm_osal_mutex_pool;
static btdm_membuf_t *s_btdm_osal_mutex_buf = NULL;

static volatile uint8_t s_btdm_osal_intr_nest = 0;
static unsigned int s_btdm_osal_irq_key;

static inline bool IRAM_ATTR in_isr(void)
{
    return k_is_in_isr();
}

static inline k_timeout_t btdm_osal_timeout(btdm_osal_time_t tmo)
{
    if (tmo == BTDM_OSAL_TIME_FOREVER) {
        return K_FOREVER;
    }
    return K_MSEC(tmo);
}

/*
 ***************************************************************************************************
 * BTDM OSAL Event Queue
 ***************************************************************************************************
 */
void
wr_btdm_osal_eventq_init(struct btdm_osal_eventq *evq)
{
    struct btdm_osal_eventq_zephyr *eventq = NULL;

    if (!btdm_memblock_from(&s_btdm_osal_evq_pool, evq->eventq)) {
        evq->eventq = btdm_memblock_get(&s_btdm_osal_evq_pool);
        eventq = (struct btdm_osal_eventq_zephyr *)evq->eventq;
        BTDM_OSAL_ASSERT(eventq);
        memset(eventq, 0, sizeof(*eventq));
        k_queue_init(&eventq->q);
    } else {
        eventq = (struct btdm_osal_eventq_zephyr *)evq->eventq;
        while (k_queue_get(&eventq->q, K_NO_WAIT) != NULL) {
        }
    }
}

void
wr_btdm_osal_eventq_deinit(struct btdm_osal_eventq *evq)
{
    struct btdm_osal_eventq_zephyr *eventq = (struct btdm_osal_eventq_zephyr *)evq->eventq;

    BTDM_OSAL_ASSERT(eventq);
    k_queue_cancel_wait(&eventq->q);
    btdm_memblock_put(&s_btdm_osal_evq_pool, eventq);
    evq->eventq = NULL;
}

struct btdm_osal_event *IRAM_ATTR
wr_btdm_osal_eventq_get(struct btdm_osal_eventq *evq, btdm_osal_time_t tmo)
{
    struct btdm_osal_eventq_zephyr *eventq = (struct btdm_osal_eventq_zephyr *)evq->eventq;
    struct btdm_osal_event_zephyr *event;

    if (in_isr()) {
        BTDM_OSAL_ASSERT(tmo == 0);
        event = k_queue_get(&eventq->q, K_NO_WAIT);
    } else {
        event = k_queue_get(&eventq->q, btdm_osal_timeout(tmo));
    }

    if (!event) {
        return NULL;
    }

    event->queued = false;
    return event->parent;
}

void IRAM_ATTR
wr_btdm_osal_eventq_put(struct btdm_osal_eventq *evq, struct btdm_osal_event *ev)
{
    struct btdm_osal_eventq_zephyr *eventq = (struct btdm_osal_eventq_zephyr *)evq->eventq;
    struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;
    unsigned int key = irq_lock();

    if (event->queued) {
        irq_unlock(key);
        return;
    }
    event->queued = true;
    k_queue_append(&eventq->q, event);
    irq_unlock(key);
}

void IRAM_ATTR
wr_btdm_osal_eventq_put_to_front(struct btdm_osal_eventq *evq, struct btdm_osal_event *ev)
{
    struct btdm_osal_eventq_zephyr *eventq = (struct btdm_osal_eventq_zephyr *)evq->eventq;
    struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;
    unsigned int key = irq_lock();

    if (event->queued) {
        irq_unlock(key);
        return;
    }
    event->queued = true;
    k_queue_prepend(&eventq->q, event);
    irq_unlock(key);
}

void IRAM_ATTR
wr_btdm_osal_eventq_remove(struct btdm_osal_eventq *evq, struct btdm_osal_event *ev)
{
    struct btdm_osal_eventq_zephyr *eventq = (struct btdm_osal_eventq_zephyr *)evq->eventq;
    struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;
    unsigned int key = irq_lock();

    if (event->queued) {
        (void)k_queue_remove(&eventq->q, event);
        event->queued = false;
    }
    irq_unlock(key);
}

bool IRAM_ATTR
wr_btdm_osal_eventq_is_empty(struct btdm_osal_eventq *evq)
{
    struct btdm_osal_eventq_zephyr *eventq = (struct btdm_osal_eventq_zephyr *)evq->eventq;

    return k_queue_is_empty(&eventq->q);
}

/*
 ***************************************************************************************************
 * BTDM OSAL Event
 ***************************************************************************************************
 */
void IRAM_ATTR
wr_btdm_osal_event_run(struct btdm_osal_event *ev)
{
    struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

    event->fn(ev);
}

void IRAM_ATTR
wr_btdm_osal_event_init(struct btdm_osal_event *ev, btdm_osal_event_fn *fn, void *arg)
{
    struct btdm_osal_event_zephyr *event = NULL;

    if (!btdm_memblock_from(&s_btdm_osal_ev_pool, ev->event)) {
        ev->event = btdm_memblock_get(&s_btdm_osal_ev_pool);
    }
    event = (struct btdm_osal_event_zephyr *)ev->event;
    BTDM_OSAL_ASSERT(event);

    memset(event, 0, sizeof(*event));
    event->parent = ev;
    event->fn = fn;
    event->arg = arg;
}

void IRAM_ATTR
wr_btdm_osal_event_deinit(struct btdm_osal_event *ev)
{
    if (!ev->event) {
        return;
    }

    btdm_memblock_put(&s_btdm_osal_ev_pool, ev->event);
    ev->event = NULL;
}

void IRAM_ATTR
wr_btdm_osal_event_reset(struct btdm_osal_event *ev)
{
    struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

    BTDM_OSAL_ASSERT(event);
    event->queued = false;
}

bool IRAM_ATTR
wr_btdm_osal_event_is_queued(struct btdm_osal_event *ev)
{
    struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

    return event->queued;
}

void *IRAM_ATTR
wr_btdm_osal_event_get_arg(struct btdm_osal_event *ev)
{
    struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

    return event->arg;
}

void IRAM_ATTR
wr_btdm_osal_event_set_arg(struct btdm_osal_event *ev, void *arg)
{
    struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

    event->arg = arg;
}

/*
 ***************************************************************************************************
 * BTDM OSAL Mutex (recursive, like the FreeRTOS recursive mutex it replaces)
 ***************************************************************************************************
 */
btdm_osal_error_t
wr_btdm_osal_mutex_init(struct btdm_osal_mutex *mu)
{
    struct btdm_osal_mutex_zephyr *mutex = NULL;

    if (!btdm_memblock_from(&s_btdm_osal_mutex_pool, mu->mutex)) {
        mu->mutex = btdm_memblock_get(&s_btdm_osal_mutex_pool);
        mutex = (struct btdm_osal_mutex_zephyr *)mu->mutex;
        if (!mutex) {
            return BTDM_OSAL_INVALID_PARM;
        }
        memset(mutex, 0, sizeof(*mutex));
        k_mutex_init(&mutex->handle);
    }

    return BTDM_OSAL_OK;
}

btdm_osal_error_t
wr_btdm_osal_mutex_deinit(struct btdm_osal_mutex *mu)
{
    struct btdm_osal_mutex_zephyr *mutex = (struct btdm_osal_mutex_zephyr *)mu->mutex;

    if (!mutex) {
        return BTDM_OSAL_INVALID_PARM;
    }

    btdm_memblock_put(&s_btdm_osal_mutex_pool, mutex);
    mu->mutex = NULL;

    return BTDM_OSAL_OK;
}

btdm_osal_error_t IRAM_ATTR
wr_btdm_osal_mutex_pend(struct btdm_osal_mutex *mu, btdm_osal_time_t timeout)
{
    struct btdm_osal_mutex_zephyr *mutex = (struct btdm_osal_mutex_zephyr *)mu->mutex;

    if (!mutex) {
        return BTDM_OSAL_INVALID_PARM;
    }

    if (in_isr()) {
        BTDM_OSAL_ASSERT(0);
        return BTDM_OSAL_ERR_IN_ISR;
    }

    return k_mutex_lock(&mutex->handle, btdm_osal_timeout(timeout)) == 0 ? BTDM_OSAL_OK
                                                                          : BTDM_OSAL_TIMEOUT;
}

btdm_osal_error_t IRAM_ATTR
wr_btdm_osal_mutex_release(struct btdm_osal_mutex *mu)
{
    struct btdm_osal_mutex_zephyr *mutex = (struct btdm_osal_mutex_zephyr *)mu->mutex;

    if (!mutex) {
        return BTDM_OSAL_INVALID_PARM;
    }

    if (in_isr()) {
        BTDM_OSAL_ASSERT(0);
        return BTDM_OSAL_ERR_IN_ISR;
    }

    if (k_mutex_unlock(&mutex->handle) != 0) {
        return BTDM_OSAL_BAD_MUTEX;
    }

    return BTDM_OSAL_OK;
}

/*
 ***************************************************************************************************
 * BTDM OSAL Sem
 ***************************************************************************************************
 */
btdm_osal_error_t
wr_btdm_osal_sem_init(struct btdm_osal_sem *sem, uint16_t tokens)
{
    struct btdm_osal_sem_zephyr *semaphore = NULL;

    if (!btdm_memblock_from(&s_btdm_osal_sem_pool, sem->sem)) {
        sem->sem = btdm_memblock_get(&s_btdm_osal_sem_pool);
        semaphore = (struct btdm_osal_sem_zephyr *)sem->sem;
        if (!semaphore) {
            return BTDM_OSAL_INVALID_PARM;
        }
        memset(semaphore, 0, sizeof(*semaphore));
        k_sem_init(&semaphore->handle, tokens, 128);
    }

    return BTDM_OSAL_OK;
}

btdm_osal_error_t
wr_btdm_osal_sem_deinit(struct btdm_osal_sem *sem)
{
    struct btdm_osal_sem_zephyr *semaphore = (struct btdm_osal_sem_zephyr *)sem->sem;

    if (!semaphore) {
        return BTDM_OSAL_INVALID_PARM;
    }

    btdm_memblock_put(&s_btdm_osal_sem_pool, semaphore);
    sem->sem = NULL;

    return BTDM_OSAL_OK;
}

btdm_osal_error_t IRAM_ATTR
wr_btdm_osal_sem_pend(struct btdm_osal_sem *sem, btdm_osal_time_t timeout)
{
    struct btdm_osal_sem_zephyr *semaphore = (struct btdm_osal_sem_zephyr *)sem->sem;
    int rc;

    if (!semaphore) {
        return BTDM_OSAL_INVALID_PARM;
    }

    if (in_isr()) {
        BTDM_OSAL_ASSERT(timeout == 0);
        rc = k_sem_take(&semaphore->handle, K_NO_WAIT);
    } else {
        rc = k_sem_take(&semaphore->handle, btdm_osal_timeout(timeout));
    }

    return rc == 0 ? BTDM_OSAL_OK : BTDM_OSAL_TIMEOUT;
}

btdm_osal_error_t IRAM_ATTR
wr_btdm_osal_sem_release(struct btdm_osal_sem *sem)
{
    struct btdm_osal_sem_zephyr *semaphore = (struct btdm_osal_sem_zephyr *)sem->sem;

    if (!semaphore) {
        return BTDM_OSAL_INVALID_PARM;
    }

    k_sem_give(&semaphore->handle);
    return BTDM_OSAL_OK;
}

uint16_t IRAM_ATTR
wr_btdm_osal_sem_get_count(struct btdm_osal_sem *sem)
{
    struct btdm_osal_sem_zephyr *semaphore = (struct btdm_osal_sem_zephyr *)sem->sem;

    return k_sem_count_get(&semaphore->handle);
}

/*
 ***************************************************************************************************
 * BTDM OSAL Callout (esp_timer based)
 ***************************************************************************************************
 */
static void IRAM_ATTR
btdm_osal_event_fn_wrapper(void *arg)
{
    struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)arg;

    BTDM_OSAL_ASSERT(callout);

    if (callout->evq) {
        btdm_osal_eventq_put(callout->evq, &callout->ev);
    } else {
        struct btdm_osal_event_zephyr *event =
            (struct btdm_osal_event_zephyr *)callout->ev.event;
        event->fn(&callout->ev);
    }
}

static btdm_osal_error_t IRAM_ATTR
esp_err_to_btdm_osal_error(esp_err_t err)
{
    switch (err) {
    case ESP_ERR_INVALID_ARG:
        return BTDM_OSAL_INVALID_PARM;
    case ESP_ERR_INVALID_STATE:
        return BTDM_OSAL_EINVAL;
    case ESP_OK:
        return BTDM_OSAL_OK;
    default:
        return BTDM_OSAL_ERROR;
    }
}

int
wr_btdm_osal_callout_init(struct btdm_osal_callout *co, struct btdm_osal_eventq *evq,
                          btdm_osal_event_fn *ev_cb, void *ev_arg)
{
    struct btdm_osal_callout_zephyr *callout = NULL;

    if (!btdm_memblock_from(&s_btdm_osal_co_pool, co->co)) {
        co->co = btdm_memblock_get(&s_btdm_osal_co_pool);
        callout = (struct btdm_osal_callout_zephyr *)co->co;
        BTDM_OSAL_ASSERT(callout);

        memset(callout, 0, sizeof(*callout));
        btdm_osal_event_init(&callout->ev, ev_cb, ev_arg);
        callout->evq = evq;

        esp_timer_create_args_t create_args = {
            .callback = btdm_osal_event_fn_wrapper,
            .arg = callout,
            .name = "btdm_timer",
        };

        if (esp_timer_create(&create_args, &callout->handle) != ESP_OK) {
            btdm_osal_event_deinit(&callout->ev);
            btdm_memblock_put(&s_btdm_osal_co_pool, callout);
            co->co = NULL;
            return -1;
        }
    } else {
        callout = (struct btdm_osal_callout_zephyr *)co->co;
        BTDM_OSAL_ASSERT(callout);
        callout->evq = evq;
        btdm_osal_event_init(&callout->ev, ev_cb, ev_arg);
    }

    return 0;
}

void
wr_btdm_osal_callout_deinit(struct btdm_osal_callout *co)
{
    struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

    if (!callout || !callout->handle) {
        return;
    }

    btdm_osal_event_deinit(&callout->ev);
    (void)esp_timer_stop(callout->handle);
    if (esp_timer_delete(callout->handle) != ESP_OK) {
        LOG_WRN("Timer not deleted");
    }
    btdm_memblock_put(&s_btdm_osal_co_pool, callout);
    co->co = NULL;
    memset(co, 0, sizeof(struct btdm_osal_callout));
}

btdm_osal_error_t IRAM_ATTR
wr_btdm_osal_callout_reset(struct btdm_osal_callout *co, btdm_osal_time_t ticks)
{
    struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

    (void)esp_timer_stop(callout->handle);

    return esp_err_to_btdm_osal_error(esp_timer_start_once(callout->handle,
                                                           (uint64_t)ticks * 1000));
}

void
wr_btdm_osal_callout_mem_reset(struct btdm_osal_callout *co)
{
    struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

    btdm_osal_event_reset(&callout->ev);
}

void IRAM_ATTR
wr_btdm_osal_callout_stop(struct btdm_osal_callout *co)
{
    struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

    if (!callout) {
        return;
    }

    (void)esp_timer_stop(callout->handle);
}

bool IRAM_ATTR
wr_btdm_osal_callout_is_active(struct btdm_osal_callout *co)
{
    struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

    return esp_timer_is_active(callout->handle);
}

btdm_osal_time_t IRAM_ATTR
wr_btdm_osal_callout_get_ticks(struct btdm_osal_callout *co)
{
    struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;
    uint64_t expiry = 0;
    btdm_osal_time_t exp = 0;

    if (esp_timer_get_expiry_time(callout->handle, &expiry) != ESP_OK) {
        return 0;
    }

    btdm_osal_time_ms_to_ticks((uint32_t)(expiry / 1000), &exp);
    return exp;
}

btdm_osal_time_t IRAM_ATTR
wr_btdm_osal_callout_remaining_ticks(struct btdm_osal_callout *co, btdm_osal_time_t now)
{
    btdm_osal_time_t exp = wr_btdm_osal_callout_get_ticks(co);

    return exp > now ? exp - now : 0;
}

void IRAM_ATTR
wr_btdm_osal_callout_set_arg(struct btdm_osal_callout *co, void *arg)
{
    struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;
    struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)callout->ev.event;

    event->arg = arg;
}

/*
 ***************************************************************************************************
 * BTDM OSAL Time (milliseconds)
 ***************************************************************************************************
 */
uint32_t IRAM_ATTR
wr_btdm_osal_time_get(void)
{
    return esp_timer_get_time() / 1000;
}

btdm_osal_error_t IRAM_ATTR
wr_btdm_osal_time_ms_to_ticks(uint32_t ms, btdm_osal_time_t *out_ticks)
{
    *out_ticks = ms;
    return BTDM_OSAL_OK;
}

btdm_osal_error_t IRAM_ATTR
wr_btdm_osal_time_ticks_to_ms(btdm_osal_time_t ticks, uint32_t *out_ms)
{
    *out_ms = ticks;
    return BTDM_OSAL_OK;
}

btdm_osal_time_t IRAM_ATTR
wr_btdm_osal_time_ms_to_ticks32(uint32_t ms)
{
    return ms;
}

uint32_t IRAM_ATTR
wr_btdm_osal_time_ticks_to_ms32(btdm_osal_time_t ticks)
{
    return ticks;
}

uint32_t IRAM_ATTR
wr_btdm_osal_get_time_forever(void)
{
    return BTDM_OSAL_TIME_FOREVER;
}

/*
 ***************************************************************************************************
 * BTDM OSAL Hw Critical
 ***************************************************************************************************
 */
uint32_t IRAM_ATTR
wr_btdm_osal_hw_enter_critical(void)
{
    unsigned int key = irq_lock();

    if (s_btdm_osal_intr_nest++ == 0) {
        s_btdm_osal_irq_key = key;
    }
    return 0;
}

uint8_t IRAM_ATTR
wr_btdm_osal_hw_is_in_critical(void)
{
    return s_btdm_osal_intr_nest;
}

void IRAM_ATTR
wr_btdm_osal_hw_exit_critical(uint32_t ctx)
{
    ARG_UNUSED(ctx);

    BTDM_OSAL_ASSERT(s_btdm_osal_intr_nest > 0);
    if (--s_btdm_osal_intr_nest == 0) {
        irq_unlock(s_btdm_osal_irq_key);
    }
}

/*
 ***************************************************************************************************
 * BTDM OSAL Task
 ***************************************************************************************************
 */
int
wr_btdm_osal_task_create(btdm_osal_task_fn *fn, const char *name, uint32_t stack_size, void *arg,
                         uint32_t priority, btdm_osal_task_handle_t *task_handle, uint32_t core_id)
{
    ARG_UNUSED(priority);
    ARG_UNUSED(core_id);

    if (stack_size > K_KERNEL_STACK_SIZEOF(s_btdm_osal_task_stacks[0])) {
        LOG_ERR("task %s: stack %u > %u", name, stack_size,
                (unsigned int)K_KERNEL_STACK_SIZEOF(s_btdm_osal_task_stacks[0]));
        return BTDM_OSAL_ENOMEM;
    }

    for (int i = 0; i < BTDM_OSAL_TASK_NUM; i++) {
        if (s_btdm_osal_task_used[i]) {
            continue;
        }

        k_tid_t tid = k_thread_create(&s_btdm_osal_task_threads[i], s_btdm_osal_task_stacks[i],
                                      K_KERNEL_STACK_SIZEOF(s_btdm_osal_task_stacks[i]),
                                      (k_thread_entry_t)fn, arg, NULL, NULL,
                                      K_PRIO_COOP(CONFIG_ESP32_BT_CONTROLLER_TASK_PRIO), 0,
                                      K_NO_WAIT);
        if (tid == NULL) {
            return BTDM_OSAL_ERROR;
        }
        s_btdm_osal_task_used[i] = true;
        k_thread_name_set(tid, name);
        if (task_handle) {
            *task_handle = (btdm_osal_task_handle_t)tid;
        }
        return BTDM_OSAL_OK;
    }

    LOG_ERR("task %s: no free task slot", name);
    return BTDM_OSAL_ENOMEM;
}

void
wr_btdm_osal_task_delete(btdm_osal_task_handle_t task_handle)
{
    for (int i = 0; i < BTDM_OSAL_TASK_NUM; i++) {
        if (s_btdm_osal_task_used[i] && (k_tid_t)task_handle == &s_btdm_osal_task_threads[i]) {
            k_thread_abort((k_tid_t)task_handle);
            s_btdm_osal_task_used[i] = false;
            return;
        }
    }
}

/*
 ***************************************************************************************************
 * BTDM OSAL Interrupt
 ***************************************************************************************************
 */
int
wr_btdm_osal_intr_alloc(int src, int flags, btdm_osal_intr_fn *fn, void *arg,
                        btdm_osal_intr_handle_t *intr_handle)
{
    int intr_flags = ESP_INTR_FLAG_LEVEL3;

    if (!flags) {
        intr_flags |= ESP_INTR_FLAG_IRAM;
    }

    return esp_intr_alloc(src, intr_flags, (intr_handler_t)fn, arg, (intr_handle_t *)intr_handle);
}

int
wr_btdm_osal_intr_free(btdm_osal_intr_handle_t intr_handle)
{
    return esp_intr_free((intr_handle_t)intr_handle);
}

/*
 ***************************************************************************************************
 * BTDM OSAL Malloc (system heap, internal RAM)
 ***************************************************************************************************
 */
void *
wr_btdm_osal_malloc(uint32_t size, btdm_osal_malloc_flag_t flags)
{
    ARG_UNUSED(flags);
    return k_malloc(size);
}

void
wr_btdm_osal_free(void *ptr)
{
    k_free(ptr);
}

#if !CONFIG_BT_CTRL_MULTI_LINK_ENABLED
void *
wr_btdm_osal_mmgmt_block_malloc(uint32_t size)
{
    uint32_t *addr = btdm_osal_malloc(size + 4, BTDM_OSAL_MALLOC_F_INTERNAL);

    if (!addr) {
        return NULL;
    }
    *addr = (5 << 29) | ((size + 4) >> 2);
    return (void *)((uint32_t)addr + 4);
}

void
wr_btdm_osal_mmgmt_block_free(void *ptr)
{
    btdm_osal_free((uint8_t *)ptr - 4);
}

void
wr_btdm_osal_mmgmt_block_copy(void *dst, const void *src, uint16_t size)
{
    extern void r_ble_lll_mmgmt_block_copy(void *addr0, void *addr1, uint16_t size);
    r_ble_lll_mmgmt_block_copy(dst, (void *)src, size);
}
#endif /* !CONFIG_BT_CTRL_MULTI_LINK_ENABLED */

/*
 ***************************************************************************************************
 * BTDM OSAL Efuse / Rand
 ***************************************************************************************************
 */
int
wr_btdm_osal_read_efuse_mac(uint8_t *mac)
{
    return esp_read_mac(mac, ESP_MAC_BT) == ESP_OK ? 0 : -1;
}

void
wr_btdm_osal_srand(uint32_t seed)
{
    srand(seed);
}

int
wr_btdm_osal_rand(void)
{
    return (int)esp_random();
}

/*
 ***************************************************************************************************
 * BTDM OSAL Initialization
 ***************************************************************************************************
 */
static int
btdm_osal_pool_init(struct btdm_mempool *pool, btdm_membuf_t **buf, uint16_t count,
                    uint32_t block_size, const char *name)
{
    if (!count) {
        return 0;
    }

    *buf = btdm_osal_malloc(BTDM_MEMPOOL_SIZE(count, block_size) * sizeof(btdm_membuf_t),
                            BTDM_OSAL_MALLOC_F_INTERNAL);
    if (!*buf) {
        return -1;
    }

    return btdm_mempool_init(pool, count, block_size, *buf, name) ? -2 : 0;
}

int
btdm_osal_elem_mempool_init(btdm_osal_elem_num_t *elem_num)
{
    if (btdm_osal_pool_init(&s_btdm_osal_ev_pool, &s_btdm_osal_ev_buf, elem_num->evt_count,
                            sizeof(struct btdm_osal_event_zephyr), "s_btdm_osal_ev_pool")) {
        return -1;
    }
    if (btdm_osal_pool_init(&s_btdm_osal_evq_pool, &s_btdm_osal_evq_buf, elem_num->evtq_count,
                            sizeof(struct btdm_osal_eventq_zephyr), "s_btdm_osal_evq_pool")) {
        return -3;
    }
    if (btdm_osal_pool_init(&s_btdm_osal_co_pool, &s_btdm_osal_co_buf, elem_num->co_count,
                            sizeof(struct btdm_osal_callout_zephyr), "s_btdm_osal_co_pool")) {
        return -5;
    }
    if (btdm_osal_pool_init(&s_btdm_osal_sem_pool, &s_btdm_osal_sem_buf, elem_num->sem_count,
                            sizeof(struct btdm_osal_sem_zephyr), "s_btdm_osal_sem_pool")) {
        return -7;
    }
    if (btdm_osal_pool_init(&s_btdm_osal_mutex_pool, &s_btdm_osal_mutex_buf,
                            elem_num->mutex_count, sizeof(struct btdm_osal_mutex_zephyr),
                            "s_btdm_osal_mutex_pool")) {
        return -9;
    }

    return 0;
}

static void
btdm_osal_pool_deinit(struct btdm_mempool *pool, btdm_membuf_t **buf)
{
    if (*buf) {
        BTDM_OSAL_ASSERT(pool->mp_num_free == pool->mp_num_blocks);
        btdm_osal_free(*buf);
        *buf = NULL;
    }
}

void
btdm_osal_elem_mempool_deinit(void)
{
    btdm_osal_pool_deinit(&s_btdm_osal_ev_pool, &s_btdm_osal_ev_buf);
    btdm_osal_pool_deinit(&s_btdm_osal_evq_pool, &s_btdm_osal_evq_buf);
    btdm_osal_pool_deinit(&s_btdm_osal_co_pool, &s_btdm_osal_co_buf);
    btdm_osal_pool_deinit(&s_btdm_osal_sem_pool, &s_btdm_osal_sem_buf);
    btdm_osal_pool_deinit(&s_btdm_osal_mutex_pool, &s_btdm_osal_mutex_buf);
}
