#ifndef __FOC_TEST_H__
#define __FOC_TEST_H__

#include <stdint.h>
#include <stdbool.h>
#include "bsp_encoder.h"


//float类型遵循IEEE754单精度标准：1位符号位+8位指数位+23位尾数位
#define SQRT_3                      1.73205080757f
#define _1_DIV_SQRT_3               0.57735026919f
#define SQRT3_DIV_2                 0.86602540378f
#define PI_DIV_3                    1.04719755120f      /*60° 弧度制*/



/* 三相电流/电压瞬时值结构体 */
typedef struct {
    float a;
    float b;
    float c;
} PhaseCurrents_t;

/* 静止两相坐标系 α-β */
typedef struct {
    float alpha;
    float beta;
} Clarke_ab_t;

/* 旋转坐标系 d-q */
typedef struct {
    float d;
    float q;
} Park_dq_t;

typedef struct
{
    PhaseCurrents_t     Uabc;
    PhaseCurrents_t     Tabc;
    Clarke_ab_t         Ualpha_beta;
    Park_dq_t           Uqd;
    
    float               angle_el;
    float               bus_Voltage;
    uint32_t            wave_period;
    
}FOC_PWM_t;

typedef struct
{
    float target_freq;
    float current_freq;

}FOC_VF_t;


extern Bsp_encoder_t   *g_enc;



Clarke_ab_t FOC_Clarke(PhaseCurrents_t *pParam);

Park_dq_t FOC_Park(Clarke_ab_t *pParam, float angle_el);

PhaseCurrents_t FOC_InvClarke(Clarke_ab_t *pParam);

Clarke_ab_t FOC_InvPark(Park_dq_t *pParam, float angle_el);


void FOC_PWM_Init(void);

void FOC_Run_SVPWM(FOC_PWM_t *pFOC_PWM);











#endif

