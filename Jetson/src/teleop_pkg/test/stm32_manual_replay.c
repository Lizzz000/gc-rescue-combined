/* 把上位机类型 2 手动速度按时间戳回放进 STM32 真实的 manual_link.c + manual_control.c，
 * 输出手动控制状态的每次变化，供 test_manual_link.py 判断 SIGKILL 后何时进入制动。
 * 底盘只做最小桩：使能/失能、设速即运动、停车后经 STOP_DELAY_MS 才算静止；不代表实物。
 * 输入（stdin，每行）：<ms> <valid> <vx> <vy> <wz>，ms 为相对毫秒且非降序；可选末行 end <ms>。
 * 输出（stdout，每行）：<ms> <状态名>。参数：argv[1] = 任务周期 ms（取自 chassis_task.c）。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "manual_link.h"
#include "manual_control.h"
#include "chassis.h"
#include "task.h"

#define STOP_DELAY_MS 100U   /* 桩：停车后到“静止”的时间，保证制动态可被观察 */

uint32_t test_tick;
static uint8_t enabled, moving;
static uint32_t stop_at;

void test_enter_critical(void) {}
void test_exit_critical(void) {}
void vTaskDelay(TickType_t ticks) { (void)ticks; }
void vTaskDelayUntil(TickType_t *wake, TickType_t ticks) { *wake += ticks; }

uint8_t Chassis_Is_Enabled(void) { return enabled; }
uint8_t Chassis_Enable(void) { enabled = 1U; return 1U; }
void Chassis_Disable(void) { enabled = 0U; moving = 0U; }
void Chassis_Stop(void) { if (moving && stop_at == 0U) stop_at = test_tick + STOP_DELAY_MS; }
uint8_t Chassis_Is_Stopped(void)
{
    if (moving && stop_at != 0U && test_tick >= stop_at) { moving = 0U; stop_at = 0U; }
    return !moving;
}
uint8_t Chassis_Motors_Ready(void) { return enabled; }
uint8_t Chassis_Set_Velocity(float vx, float vy, float wz)
{
    if (vx != 0.0f || vy != 0.0f || wz != 0.0f) { moving = 1U; stop_at = 0U; }
    return enabled;
}

static const char *Name(Manual_Control_State_t s)
{
    static const char *names[] = {"DISARMED", "PREPARING", "READY", "RUNNING", "BRAKING"};
    return (unsigned)s < 5U ? names[s] : "?";
}

int main(int argc, char **argv)
{
    const uint32_t period = argc > 1 ? (uint32_t)strtoul(argv[1], NULL, 10) : 5U;
    char line[128];
    uint32_t frame_ms = 0U, end_ms = 0U;
    int valid = 0, vx = 0, vy = 0, wz = 0, pending = 0;
    Manual_CAN_Init();
    Manual_Control_Init();
    Manual_Control_State_t last = manual_control_state;
    printf("0 %s\n", Name(last));
    for (test_tick = 1U;; ++test_tick) {
        /* 投递所有时间戳 ≤ 当前毫秒的帧 */
        for (;;) {
            if (!pending) {
                if (!fgets(line, sizeof line, stdin)) { pending = -1; break; }
                if (sscanf(line, "end %u", &end_ms) == 1) { pending = -1; break; }
                if (sscanf(line, "%u %d %d %d %d", &frame_ms, &valid, &vx, &vy, &wz) != 5) continue;
                pending = 1;
            }
            if (pending != 1 || frame_ms > test_tick) break;
            const int16_t v[3] = {(int16_t)vx, (int16_t)vy, (int16_t)wz};
            uint8_t data[7] = {(uint8_t)valid};
            memcpy(&data[1], v, sizeof v);   /* 小端主机 */
            Manual_CAN_Receive(data, 7U);
            pending = 0;
        }
        if (test_tick % period == 0U) {
            Manual_Control_Loop();
            if (manual_control_state != last) {
                last = manual_control_state;
                printf("%u %s\n", test_tick, Name(last));
            }
        }
        if (pending == -1 && test_tick >= end_ms) break;
    }
    return 0;
}
