#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "usb_link.h"

/* This test links the generated startup routine to a scheduler stub that records
   actual thread entries. On the Cortex-M port, creating the first thread masks TIM6. */
typedef uint32_t StaticTask_t;
typedef uint32_t StackType_t;
typedef void (*osThreadId)(void const *);
#define configMINIMAL_STACK_SIZE 128U
#define osThreadDef(name, entry, priority, instances, stack) osThreadId name = entry
#define osThread(name) (name)
#define __weak

static unsigned thread_count, usb_init_count;
static osThreadId threads[3];
static uint8_t usb_ready;

static osThreadId osThreadCreate(osThreadId entry, void *argument)
{
    assert(argument == NULL && thread_count < 3U);
    threads[thread_count++] = entry;
    return entry;
}

uint8_t UsbLink_Init(void)
{
    assert(thread_count == 0U); /* HAL_Delay still needs the unmasked TIM6 interrupt. */
    ++usb_init_count;
    return usb_ready;
}

void UsbLink_Task(void const *argument) { (void)argument; }
void test_delay_ms(uint32_t ms) { (void)ms; }

/* Cube-generated weak thread defaults intentionally ignore their argument. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "../../project/Core/Src/freertos.c"
#pragma GCC diagnostic pop

int main(void)
{
    for (unsigned ready = 0; ready <= 1U; ++ready)
    {
        thread_count = usb_init_count = 0U;
        usb_ready = ready;
        MX_FREERTOS_Init();
        assert(threads[0] == test_task && testHandle == test_task);
        assert(threads[1] == INS_task && imuTaskHandle == INS_task);
#if UART6_MANUAL_ENABLED
        assert(usb_init_count == 0U && thread_count == 2U);
#else
        assert(usb_init_count == 1U && thread_count == 2U + ready);
        if (ready) assert(threads[2] == UsbLink_Task);
#endif
    }
    puts("PASS: RTOS startup preserves the HAL tick and starts chassis/INS independently of USB");
    return 0;
}
