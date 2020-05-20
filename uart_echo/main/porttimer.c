#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "driver/timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DBG_TAG "porttimer.c"

#define MODBUS_TIMER_INDEX              (0) 
#define MODBUS_TIMER_GROUP              (1)

#define MB_US50_FREQ            (20000) // 20kHz 1/20000 = 50mks
#define MB_DISCR_TIME_US        (50)    // 50uS = one discreet for timer

#define MB_TIMER_PRESCALLER     ((TIMER_BASE_CLK / MB_US50_FREQ) - 1);
#define MB_TIMER_SCALE          (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define MB_TIMER_DIVIDER        ((TIMER_BASE_CLK / 1000000UL) * MB_DISCR_TIME_US - 1) // divider for 50uS
#define MB_TIMER_WITH_RELOAD    (1)

static const uint16_t usTimerIndex =      MODBUS_TIMER_INDEX; // Modbus Timer index used by stack
static const uint16_t usTimerGroupIndex = MODBUS_TIMER_GROUP; // Modbus Timer group index used by stack
static uint16_t usTim1Timerout50us_;

extern portBASE_TYPE uart_addTimeoutEventFromIsr(void);
void vMBPortTimersDisable(void);
/* ----------------------- Start implementation -----------------------------*/
static void IRAM_ATTR vTimerGroupIsr(void *param)
{
    portBASE_TYPE hPTaskAwoken = 0;
    
    timer_intr_t timer_intr = timer_group_intr_get_in_isr(usTimerGroupIndex);
    if ((timer_intr & TIMER_INTR_T0) == 0)
    {
        return;
    }
    
    assert((int)param == usTimerIndex);
    // Retrieve the counter value from the timer that reported the interrupt
    timer_group_intr_clr_in_isr(usTimerGroupIndex, usTimerIndex);
    hPTaskAwoken = uart_addTimeoutEventFromIsr();
    timer_pause(usTimerGroupIndex, usTimerIndex);
    timer_group_enable_alarm_in_isr(usTimerGroupIndex, usTimerIndex);
}

int8_t  mbrtuTmr_init(uint32_t intercharTime)
{
    uint16_t usTim1Timerout50us = intercharTime / 50;

    esp_err_t xErr = 0;
    timer_config_t config;
    config.alarm_en = TIMER_ALARM_EN;
    config.auto_reload = MB_TIMER_WITH_RELOAD;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = MB_TIMER_PRESCALLER;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;
    // Configure timer
    xErr |= timer_init(usTimerGroupIndex, usTimerIndex, &config);
    xErr |= timer_pause(usTimerGroupIndex, usTimerIndex);
    xErr |= timer_set_counter_value(usTimerGroupIndex, usTimerIndex, 0x00000000ULL);
    xErr |= timer_set_alarm_value(usTimerGroupIndex, usTimerIndex, (uint32_t)(usTim1Timerout50us));
    usTim1Timerout50us_ = usTim1Timerout50us;
    // Register ISR for timer
    xErr |= timer_isr_register(usTimerGroupIndex, usTimerIndex, vTimerGroupIsr, (void*)(uint32_t)usTimerIndex, ESP_INTR_FLAG_IRAM, NULL);
	ESP_LOGI(DBG_TAG, "%s modbus timer init %d", __func__, xErr);
    return true;
}

void  mbrtuTmr_startIntercharTimer(void)
{
#if 1
    timer_pause(usTimerGroupIndex, usTimerIndex);
    timer_set_counter_value(usTimerGroupIndex, usTimerIndex, 0ULL);
    timer_set_alarm_value(usTimerGroupIndex, usTimerIndex, (uint32_t)(usTim1Timerout50us_));
    timer_set_alarm(usTimerGroupIndex, usTimerIndex, 1);
    timer_enable_intr(usTimerGroupIndex, usTimerIndex);
    timer_start(usTimerGroupIndex, usTimerIndex);
#endif
}

void vMBPortTimersDisable(void)
{
    timer_pause(usTimerGroupIndex, usTimerIndex);
    timer_set_counter_value(usTimerGroupIndex, usTimerIndex, 0ULL);
    timer_disable_intr(usTimerGroupIndex, usTimerIndex);
}

void vMBPortTimerClose(void)
{
    timer_pause(usTimerGroupIndex, usTimerIndex);
    timer_disable_intr(usTimerGroupIndex, usTimerIndex);
}

