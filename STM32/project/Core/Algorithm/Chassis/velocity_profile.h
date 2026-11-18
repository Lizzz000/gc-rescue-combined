#ifndef VELOCITY_PROFILE_H
#define VELOCITY_PROFILE_H

/* Body-frame vx, vy (m/s), wz (rad/s). Each controller owns its profile. */
typedef struct
{
    float ramp[3];
    float output[3];
} Velocity_Profile_t;

void Velocity_Profile_Seed(Velocity_Profile_t *profile, float vx, float vy, float wz);
/* Positive acceleration limits, smoothing time >= 0 and elapsed time >= 0, in SI units. */
void Velocity_Profile_Step(Velocity_Profile_t *profile, const float target[3], float linear_accel,
                           float angular_accel, float smoothing_s, float dt);

#endif
