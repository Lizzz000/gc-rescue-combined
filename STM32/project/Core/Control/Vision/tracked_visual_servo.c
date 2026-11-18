#include "tracked_visual_servo.h"
#include <stddef.h>

void Tracked_Visual_Servo_Update(Visual_Servo_t *servo, const Visual_Servo_Config_t *config,
                                 const AdaptiveKalman *tracker, const float raw[2], const float goal[2],
                                 const float tolerance[2], float seconds)
{
    Visual_Servo_Estimate_t estimate;
    const Visual_Servo_Estimate_t *input = NULL;
    if (tracker != NULL && tracker->initialized)
    {
        estimate = (Visual_Servo_Estimate_t){
            .position = {tracker->position[0], tracker->position[1]},
            .velocity = {tracker->velocity[0], tracker->velocity[1]},
            .accepted_mask = tracker->accepted_mask,
        };
        input = &estimate;
    }
    Visual_Servo_Update(servo, config, input, raw, goal, tolerance, seconds);
}
