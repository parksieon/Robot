/**
 * @file trajectory_generator.c
 * @brief 사인파 궤적 생성 라이브러리 구현
 * @date 2025-11-07
 */

#include "trajectory_generator.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ========================================================================
 * 궤적 생성기 초기화
 * ======================================================================== */
void TrajectoryGenerator_Init(TrajectoryGenerator_t* gen, 
                               float amplitude, 
                               float frequency, 
                               float offset, 
                               float dt)
{
    if (gen == NULL) return;
    
    gen->params.amplitude = amplitude;
    gen->params.frequency = frequency;
    gen->params.offset = offset;
    gen->params.phase = 0.0f;
    
    gen->time = 0.0f;
    gen->dt = dt;
    gen->initialized = true;
}

/* ========================================================================
 * 궤적 생성기 리셋
 * ======================================================================== */
void TrajectoryGenerator_Reset(TrajectoryGenerator_t* gen)
{
    if (gen == NULL) return;
    
    gen->time = 0.0f;
}

/* ========================================================================
 * 목표 위치 계산
 * θ_ref(t) = A·sin(2πf·t + φ) + offset
 * ======================================================================== */
float TrajectoryGenerator_GetPosition(TrajectoryGenerator_t* gen)
{
    if (gen == NULL || !gen->initialized) return 0.0f;
    
    float omega = 2.0f * M_PI * gen->params.frequency;  // 각속도 [rad/s]
    float angle = omega * gen->time + gen->params.phase;
    
    float position = gen->params.amplitude * sinf(angle) + gen->params.offset;
    
    return position;  // [deg]
}

/* ========================================================================
 * 목표 속도 계산 (1차 미분)
 * dθ/dt = A·ω·cos(2πf·t + φ)
 * ======================================================================== */
float TrajectoryGenerator_GetVelocity(TrajectoryGenerator_t* gen)
{
    if (gen == NULL || !gen->initialized) return 0.0f;
    
    float omega = 2.0f * M_PI * gen->params.frequency;  // [rad/s]
    float angle = omega * gen->time + gen->params.phase;
    
    // dθ/dt를 deg/s로 반환 (라디안을 각도로 변환)
    float velocity = gen->params.amplitude * omega * cosf(angle) * (180.0f / M_PI);
    
    return velocity;  // [deg/s]
}

/* ========================================================================
 * 목표 가속도 계산 (2차 미분)
 * d²θ/dt² = -A·ω²·sin(2πf·t + φ)
 * ======================================================================== */
float TrajectoryGenerator_GetAcceleration(TrajectoryGenerator_t* gen)
{
    if (gen == NULL || !gen->initialized) return 0.0f;
    
    float omega = 2.0f * M_PI * gen->params.frequency;  // [rad/s]
    float angle = omega * gen->time + gen->params.phase;
    
    // d²θ/dt²를 deg/s²로 반환
    float acceleration = -gen->params.amplitude * omega * omega * sinf(angle) * (180.0f / M_PI);
    
    return acceleration;  // [deg/s²]
}

/* ========================================================================
 * 시간 업데이트
 * ======================================================================== */
void TrajectoryGenerator_Update(TrajectoryGenerator_t* gen)
{
    if (gen == NULL || !gen->initialized) return;
    
    gen->time += gen->dt;
}

/* ========================================================================
 * 파라미터 동적 변경
 * ======================================================================== */
void TrajectoryGenerator_SetParams(TrajectoryGenerator_t* gen,
                                    float amplitude,
                                    float frequency,
                                    float offset)
{
    if (gen == NULL) return;
    
    gen->params.amplitude = amplitude;
    gen->params.frequency = frequency;
    gen->params.offset = offset;
}
