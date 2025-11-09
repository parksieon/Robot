/**
 * @file trajectory_generator.h
 * @brief 사인파 궤적 생성 라이브러리
 * @date 2025-11-07
 * 
 * PD 제어를 위한 사인파 기반 목표 궤적을 생성합니다.
 */

#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <math.h>
#include <stdbool.h>

/* ========================================================================
 * 사인파 궤적 파라미터 구조체
 * ======================================================================== */
typedef struct {
    float amplitude;      // 진폭 [deg]
    float frequency;      // 주파수 [Hz]
    float offset;         // 오프셋 (중심값) [deg]
    float phase;          // 위상 [rad] (선택적)
} SineWaveParams_t;

/* ========================================================================
 * 궤적 생성기 구조체
 * ======================================================================== */
typedef struct {
    SineWaveParams_t params;    // 사인파 파라미터
    float time;                 // 현재 시간 [s]
    float dt;                   // 샘플링 시간 [s]
    bool initialized;           // 초기화 여부
} TrajectoryGenerator_t;

/* ========================================================================
 * 함수 선언
 * ======================================================================== */

/**
 * @brief 궤적 생성기 초기화
 * @param gen 궤적 생성기 포인터
 * @param amplitude 진폭 [deg]
 * @param frequency 주파수 [Hz]
 * @param offset 오프셋 [deg]
 * @param dt 샘플링 시간 [s] (예: 0.001 for 1kHz)
 */
void TrajectoryGenerator_Init(TrajectoryGenerator_t* gen, 
                               float amplitude, 
                               float frequency, 
                               float offset, 
                               float dt);

/**
 * @brief 궤적 생성기 리셋 (시간 카운터를 0으로)
 * @param gen 궤적 생성기 포인터
 */
void TrajectoryGenerator_Reset(TrajectoryGenerator_t* gen);

/**
 * @brief 현재 시간에서의 목표 위치 계산
 * @param gen 궤적 생성기 포인터
 * @return 목표 위치 [deg]
 */
float TrajectoryGenerator_GetPosition(TrajectoryGenerator_t* gen);

/**
 * @brief 현재 시간에서의 목표 속도 계산 (미분값)
 * @param gen 궤적 생성기 포인터
 * @return 목표 속도 [deg/s]
 */
float TrajectoryGenerator_GetVelocity(TrajectoryGenerator_t* gen);

/**
 * @brief 현재 시간에서의 목표 가속도 계산 (2차 미분값)
 * @param gen 궤적 생성기 포인터
 * @return 목표 가속도 [deg/s^2]
 */
float TrajectoryGenerator_GetAcceleration(TrajectoryGenerator_t* gen);

/**
 * @brief 시간 업데이트 (다음 샘플로 이동)
 * @param gen 궤적 생성기 포인터
 */
void TrajectoryGenerator_Update(TrajectoryGenerator_t* gen);

/**
 * @brief 파라미터 동적 변경
 * @param gen 궤적 생성기 포인터
 * @param amplitude 새 진폭 [deg]
 * @param frequency 새 주파수 [Hz]
 * @param offset 새 오프셋 [deg]
 */
void TrajectoryGenerator_SetParams(TrajectoryGenerator_t* gen,
                                    float amplitude,
                                    float frequency,
                                    float offset);

#endif /* TRAJECTORY_GENERATOR_H */
