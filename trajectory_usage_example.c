/**
 * ========================================================================
 * trajectory_generator 라이브러리 사용 예시
 * ========================================================================
 * 
 * algorithm_ctrl.c 파일에 추가할 내용
 * ========================================================================
 */

/* 
 * 1. 파일 상단에 헤더 포함 추가
 */
#include "trajectory_generator.h"

/* 
 * 2. 전역 변수 선언 (변수 선언 섹션에 추가)
 */
static TrajectoryGenerator_t trajectory_gen;  // 궤적 생성기 인스턴스

/* 
 * 3. USER_DEFINED_CTRL 섹션에서 사용
 */
} else if (controlMode == USER_DEFINED_CTRL) {   // 6 - User Defined Control
    // ========================================================================
    // USER DEFINED CONTROL: PD Controller with Sine Wave Trajectory
    // ========================================================================
    
    // 1) 모드 진입 시 초기화
    if (user_first) {
        // 궤적 생성기 초기화
        // 파라미터: 진폭 15°, 주파수 1Hz, 오프셋 15°, 샘플링 1ms
        TrajectoryGenerator_Init(&trajectory_gen, 15.0f, 1.0f, 15.0f, 0.001f);
        
        Lerror_prev = 0.0f;
        user_first = false;
    }

    // 2) 사인파 목표 궤적 생성
    float theta_ref = TrajectoryGenerator_GetPosition(&trajectory_gen);      // 목표 위치 [deg]
    float theta_ref_dot = TrajectoryGenerator_GetVelocity(&trajectory_gen);  // 목표 속도 [deg/s]

    // 3) 측정값 & 오차 계산
    Ldeg = robotDataObj_LH.thighTheta_act;        // 현재 각도 [deg]
    Lerror = theta_ref - Ldeg;                     // 오차 = 목표 - 현재

    // 4) 오차 미분 계산
    float Lerror_diff = (Lerror - Lerror_prev) / dT;  // de/dt

    // 5) PD 제어 입력 계산
    float Pterm = Kp * Lerror;           // 비례항
    float Dterm = Kd * Lerror_diff;      // 미분항

    // 6) 제어 출력
    float u = Pterm + Dterm;

    // 7) 상태 업데이트
    Lerror_prev = Lerror;
    TrajectoryGenerator_Update(&trajectory_gen);  // 시간 진행

    // 8) 제어 입력 출력
    UserDefinedCtrl_LH.control_input = u;

    // 9) 다른 제어 모드 초기화
    posCtrl_RH.control_input = 0.0f;
    posCtrl_LH.control_input = 0.0f;
    gravCompDataObj_RH.control_input = 0.0f;
    gravCompDataObj_LH.control_input = 0.0f;
    impedanceCtrl_RH.control_input = 0.0f;
    impedanceCtrl_LH.control_input = 0.0f;
    StepCurr_RH.control_input = 0.0f;
    StepCurr_LH.control_input = 0.0f;
    UserDefinedCtrl_RH.control_input = 0.0f;
    f_vector_input_LH = 0.0f;
    f_vector_input_RH = 0.0f;

    // 10) 디버깅 변수
    free_var1 = theta_ref;        // 목표 각도 [deg]
    free_var2 = Ldeg;             // 현재 각도 [deg]
    free_var3 = Lerror;           // 오차 [deg]
    free_var4 = u;                // 제어 출력 [A]
    free_var5 = theta_ref_dot;    // 목표 속도 [deg/s]
}

/* 
 * ========================================================================
 * 파라미터 실시간 변경 예시
 * ========================================================================
 */

// 주파수를 0.5Hz로 변경 (더 느리게)
TrajectoryGenerator_SetParams(&trajectory_gen, 15.0f, 0.5f, 15.0f);

// 범위를 10~40도로 변경 (진폭 15, 오프셋 25)
TrajectoryGenerator_SetParams(&trajectory_gen, 15.0f, 1.0f, 25.0f);

// 시간 리셋 (처음부터 다시)
TrajectoryGenerator_Reset(&trajectory_gen);

/* 
 * ========================================================================
 * 빌드 설정
 * ========================================================================
 * 
 * STM32 프로젝트의 Makefile 또는 프로젝트 설정에 추가:
 * 
 * 1. 소스 파일 추가:
 *    - trajectory_generator.c
 * 
 * 2. 헤더 경로 추가:
 *    - trajectory_generator.h가 있는 디렉토리
 * 
 * 3. math 라이브러리 링크:
 *    -lm (이미 포함되어 있을 가능성 높음)
 * 
 * ========================================================================
 */
