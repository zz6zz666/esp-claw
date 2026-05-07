/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "lua_module_delay.h"

#include <stdint.h>
#include <stdbool.h>

#include "cap_lua.h"
#include "esp_rom_sys.h"
#include "lauxlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LUA_MODULE_DELAY_US_MAX_BLOCKING 1000000U

#ifdef __linux__
#include <time.h>
#elif _WIN32
#include <windows.h>
#endif

/* ---- frame_sync -------------------------------------------------------
 * Frame pacing: ensure exactly `target_ms` elapses between consecutive
 * calls, regardless of how much time the caller spent rendering.
 *
 *    frame_sync(50)  -- first call anchors t0, returns immediately
 *    -- ... render 14ms ...
 *    frame_sync(50)  -- waits 36ms, total gap from last call = 50ms
 *
 * Uses a simple static timestamp — Lua is single-threaded so this is safe.
 * ----------------------------------------------------------------------*/

#if defined(__linux__) || defined(_WIN32)

static uint64_t frame_sync_now_us(void)
{
#if defined(__linux__)
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
#elif defined(_WIN32)
    static LARGE_INTEGER freq = {0};
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (uint64_t)(now.QuadPart * 1000000ULL / freq.QuadPart);
#endif
}

static uint64_t s_frame_sync_last_us = 0;

static int lua_module_delay_frame_sync(lua_State *L)
{
    lua_Integer target_ms = luaL_checkinteger(L, 1);
    if (target_ms < 0) target_ms = 0;

    uint64_t now_us = frame_sync_now_us();
    uint64_t target_us = (uint64_t)target_ms * 1000ULL;

    if (s_frame_sync_last_us != 0 && target_us > 0) {
        uint64_t elapsed_us = now_us - s_frame_sync_last_us;
        if (elapsed_us < target_us) {
            uint64_t remain_us = target_us - elapsed_us;
#if defined(__linux__)
            struct timespec req, rem;
            req.tv_sec  = (time_t)(remain_us / 1000000);
            req.tv_nsec = (long)((remain_us % 1000000) * 1000);
            while (nanosleep(&req, &rem) == -1 && errno == EINTR) {
                req = rem;
            }
#elif defined(_WIN32)
            LARGE_INTEGER freq_win, start_win, end_win;
            QueryPerformanceFrequency(&freq_win);
            QueryPerformanceCounter(&start_win);
            uint64_t target_ticks = remain_us * freq_win.QuadPart / 1000000ULL;
            for (;;) {
                QueryPerformanceCounter(&end_win);
                if ((uint64_t)(end_win.QuadPart - start_win.QuadPart) >= target_ticks)
                    break;
            }
#endif
            now_us = frame_sync_now_us();
        }
    }

    s_frame_sync_last_us = now_us;
    return 0;
}

#else /* no high-res clock available — passthrough to delay_ms */

static int lua_module_delay_frame_sync(lua_State *L)
{
    return lua_module_delay_sleep_ms(L);
}

#endif

static int lua_module_delay_sleep_ms(lua_State *L)
{
    lua_Integer ms = luaL_checkinteger(L, 1);

    if (ms < 0) {
        ms = 0;
    }

#ifdef __linux__
    /* Use nanosleep for precise millisecond delay on Linux */
    struct timespec req, rem;
    req.tv_sec = (time_t)(ms / 1000);
    req.tv_nsec = (long)((ms % 1000) * 1000000);
    while (nanosleep(&req, &rem) == -1 && errno == EINTR) {
        req = rem;
    }
#elif _WIN32
    /* Use QueryPerformanceCounter for high-precision timing on Windows */
    LARGE_INTEGER freq, start, end;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&start);
    uint64_t target_ticks = (uint64_t)ms * freq.QuadPart / 1000;
    do {
        QueryPerformanceCounter(&end);
    } while ((end.QuadPart - start.QuadPart) < target_ticks);
#else
    /* Fallback to vTaskDelay for embedded targets */
    vTaskDelay(pdMS_TO_TICKS((uint32_t)ms));
#endif
    return 0;
}

static int lua_module_delay_sleep_us(lua_State *L)
{
    lua_Integer us = luaL_checkinteger(L, 1);

    if (us < 0) {
        us = 0;
    }

    if ((uint64_t)us > LUA_MODULE_DELAY_US_MAX_BLOCKING) {
        return luaL_error(L, "delay_us supports 0..%u only; use delay_ms for longer waits",
                          LUA_MODULE_DELAY_US_MAX_BLOCKING);
    }

    /* Microsecond delay is a busy-wait, so keep it for short hardware timings only. */
    esp_rom_delay_us((uint32_t)us);
    return 0;
}

int luaopen_delay(lua_State *L)
{
    lua_newtable(L);
    lua_pushcfunction(L, lua_module_delay_sleep_ms);
    lua_setfield(L, -2, "delay_ms");
    lua_pushcfunction(L, lua_module_delay_sleep_us);
    lua_setfield(L, -2, "delay_us");
    lua_pushcfunction(L, lua_module_delay_frame_sync);
    lua_setfield(L, -2, "frame_sync");
    return 1;
}

esp_err_t lua_module_delay_register(void)
{
    return cap_lua_register_module("delay", luaopen_delay);
}
