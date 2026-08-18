#include "foc.h"
#include "arm_math.h"
#include "motor_runtime_param.h"
#include <stdbool.h>
#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define rad60 deg2rad(60)
#define SQRT3 1.73205080756887729353

arm_pid_instance_f32 pid_position;
arm_pid_instance_f32 pid_speed;
arm_pid_instance_f32 pid_torque_d;
arm_pid_instance_f32 pid_torque_q;
volatile motor_control_context_t motor_control_context;
motor_pid_param_t motor_pid_position;
motor_pid_param_t motor_pid_speed;
motor_pid_param_t motor_pid_torque_d;
motor_pid_param_t motor_pid_torque_q;

static void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w)
{
    d = min(d, 1);
    d = max(d, -1);
    q = min(q, 1);
    q = max(q, -1);
    const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
    const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};
    float sin_phi = arm_sin_f32(phi);
    float cos_phi = arm_cos_f32(phi);
    float alpha = 0;
    float beta = 0;
    arm_inv_park_f32(d, q, &alpha, &beta, sin_phi, cos_phi);

    bool A = beta > 0;
    bool B = fabs(beta) > SQRT3 * fabs(alpha);
    bool C = alpha > 0;

    int K = 4 * A + 2 * B + C;
    int sector = K_to_sector[K];

    float t_m = arm_sin_f32(sector * rad60) * alpha - arm_cos_f32(sector * rad60) * beta;
    float t_n = beta * arm_cos_f32(sector * rad60 - rad60) - alpha * arm_sin_f32(sector * rad60 - rad60);
    float t_0 = 1 - t_m - t_n;

    *d_u = t_m * v[sector - 1][0] + t_n * v[sector % 6][0] + t_0 / 2;
    *d_v = t_m * v[sector - 1][1] + t_n * v[sector % 6][1] + t_0 / 2;
    *d_w = t_m * v[sector - 1][2] + t_n * v[sector % 6][2] + t_0 / 2;
}

void set_pwm_duty(float d_u, float d_v, float d_w) __attribute__((weak));
void set_pwm_duty(float d_u, float d_v, float d_w)
// __attribute__((weak)) void set_pwm_duty(float d_u, float d_v, float d_w)
{
    while (1)
        ;
}

void foc_forward(float d, float q, float rotor_rad)
{
    float d_u = 0;
    float d_v = 0;
    float d_w = 0;
    svpwm(rotor_rad, d, q, &d_u, &d_v, &d_w);
    set_pwm_duty(d_u, d_v, d_w);
}

static float position_loop(float rad)
{
    float diff = cycle_diff(rad - motor_logic_angle, position_cycle);
    return arm_pid_f32(&pid_position, diff);
}

static float speed_loop(float speed_rad)
{
    float diff = speed_rad - motor_speed;
    return arm_pid_f32(&pid_speed, diff);
}

static float torque_d_loop(float d)
{
    float diff = d - motor_i_d / MAX_CURRENT;
    float out_unlimited = arm_pid_f32(&pid_torque_d, diff);
    float out_limited = 0;
    out_limited = min(out_unlimited, 1);
    out_limited = max(out_limited, -1);

    float error_integral_windup = out_limited - out_unlimited; //  积分饱和误差
    float Kt = 0.7f;                                           //  后积分增益 (Anti-windup Gain)
    pid_torque_d.state[0] -= (pid_torque_d.Ki * Kt * error_integral_windup);

    return out_limited;
}

static float torque_q_loop(float q)
{
    float diff = q - motor_i_q / MAX_CURRENT;
    float out_unlimited = arm_pid_f32(&pid_torque_q, diff);
    float out_limited = 0;
    out_limited = min(out_unlimited, 1);
    out_limited = max(out_limited, -1);

    float Kt = 0.7f;                                           //  后积分增益 (Anti-windup Gain)
    float error_integral_windup = out_limited - out_unlimited; //  积分饱和误差
    pid_torque_q.state[0] -= (pid_torque_q.Ki * Kt * error_integral_windup);

    return out_limited;
}

void lib_position_control(float rad)
{
    // 直接位置模式不经过电流环，没有Id/Iq目标
    motor_target_i_d = 0;
    motor_target_i_q = 0;
    float d = 0;
    float q = position_loop(rad);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_speed_control(float speed)
{
    // 直接速度模式不经过电流环，没有Id/Iq目标
    motor_target_i_d = 0;
    motor_target_i_q = 0;
    float d = 0;
    float q = speed_loop(speed);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_torque_control(float torque_norm_d, float torque_norm_q)
{
    // 对外显示安培值；电流PI内部仍使用-MAX_CURRENT~MAX_CURRENT的归一化值
    motor_target_i_d = torque_norm_d * MAX_CURRENT;
    motor_target_i_q = torque_norm_q * MAX_CURRENT;
    float d = torque_d_loop(torque_norm_d);
    float q = torque_q_loop(torque_norm_q);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_speed_torque_control(float speed_rad, float max_torque_norm)
{
    float torque_norm = speed_loop(speed_rad);
    torque_norm = min(fabs(torque_norm), max_torque_norm) * (torque_norm > 0 ? 1 : -1);
    lib_torque_control(0, torque_norm);
}

void lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm)
{
    float speed_rad = position_loop(position_rad);
    speed_rad = min(fabs(speed_rad), max_speed_rad) * (speed_rad > 0 ? 1 : -1);
    lib_speed_torque_control(speed_rad, max_torque_norm);
}

void set_motor_pid(
    float position_p, float position_i, float position_d,
    float speed_p, float speed_i, float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d)
{
    set_position_pid(position_p, position_i, position_d);
    set_speed_pid(speed_p, speed_i, speed_d);
    set_torque_d_pid(torque_d_p, torque_d_i, torque_d_d);
    set_torque_q_pid(torque_q_p, torque_q_i, torque_q_d);
}

static void set_pid_param(arm_pid_instance_f32 *pid, motor_pid_param_t *param,
                          float p, float i, float d)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    param->p = p;
    param->i = i;
    param->d = d;
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    // 在线修改PID时清除旧的误差和输出状态，避免切换参数后突然冲击
    arm_pid_init_f32(pid, true);

    if (!primask)
        __enable_irq();
}

void reset_motor_pid_states(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    arm_pid_reset_f32(&pid_position);
    arm_pid_reset_f32(&pid_speed);
    arm_pid_reset_f32(&pid_torque_d);
    arm_pid_reset_f32(&pid_torque_q);
    if (!primask)
        __enable_irq();
}

void set_position_pid(float p, float i, float d)
{
    set_pid_param(&pid_position, &motor_pid_position, p, i, d);
}

void set_speed_pid(float p, float i, float d)
{
    set_pid_param(&pid_speed, &motor_pid_speed, p, i, d);
}

void set_torque_d_pid(float p, float i, float d)
{
    set_pid_param(&pid_torque_d, &motor_pid_torque_d, p, i, d);
}

void set_torque_q_pid(float p, float i, float d)
{
    set_pid_param(&pid_torque_q, &motor_pid_torque_q, p, i, d);
}

float cycle_diff(float diff, float cycle)
{
    if (diff > (cycle / 2))
        diff -= cycle;
    else if (diff < (-cycle / 2))
        diff += cycle;
    return diff;
}
