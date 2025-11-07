#include "algorithm_ctrl.h"

/* ------------------- Default Variables ------------------ */
TaskObj_t algorithmCtrlTask;

uint8_t CM_connect_signal = 0;
uint8_t CM_disconnect_signal = 0;

uint8_t motionMap_selection = 99;
uint8_t startPvector_decoding = 0;

ControlMode controlMode = USER_DEFINED_CTRL;

float EMG_Rawsignal = 0.0f;
float EMG_R1_Rawsignal = 0.0f;
float EMG_L1_Rawsignal = 0.0f;

uint16_t fsr1_R1 = 0;
uint16_t fsr2_R2 = 0;
uint16_t fsr1_L1 = 0;
uint16_t fsr2_L2 = 0;

float free_var1 = 0.0f;
float free_var2 = 0.0f;
float free_var3 = 0.0f;
float free_var4 = 0.0f;
float free_var5 = 0.0f;

float Lerror = 0.0f;
float Ldeg = 0.0f;
float Kp = 0.0f;
float count =0.0f;
float Ki =0.0f;
float sum =0.0f;

RobotData_t robotDataObj_RH;
RobotData_t robotDataObj_LH;
GravComp gravCompDataObj_RH;
GravComp gravCompDataObj_LH;
ImpedanceCtrl impedanceCtrl_RH;
ImpedanceCtrl impedanceCtrl_LH;
StepCurr StepCurr_RH;
StepCurr StepCurr_LH;
UserDefinedCtrl UserDefinedCtrl_RH;
UserDefinedCtrl UserDefinedCtrl_LH;

/* For Code Time Check */
static uint32_t STUDENTcodeStartTick = 0;
static uint32_t STUDENTcodeEndTick = 0;
static uint32_t algorithmCtrlLoopCnt;
static float algorithmCtrlTimeElap;

bool isFirstPos_RH = true;
bool isFirstPos_LH = true;
bool isFirstImp_RH = true;
bool isFirstImp_LH = true;

bool pVectorTrig_RH = false;
bool pVectorTrig_LH = false;
bool fVectorTrig_RH = false;
bool fVectorTrig_LH = false;
uint8_t MotionMap_ID_RH = 0;
uint8_t MotionMap_ID_LH = 0;

float RightHipFlexionTorque = 0.0f;
float RightHipExtensionTorque = 0.0f;
float LeftHipFlexionTorque = 0.0f;
float LeftHipExtensionTorque = 0.0f;

static uint32_t lastUpdateCnt = 0;
static uint8_t currentAmp = 0;
static bool done = false;

uint8_t ackSignal = 0;

float f_vector_input_RH = 0.0f;
float f_vector_input_LH = 0.0f;

float assist_level = 1.0f;

/*---------- (1) START of STUDENT CODE (Declare the functions to use) -----------*/

/* 수중 저항(허벅지/힙 관절) 파라미터 */
typedef struct {
  float rho;        // 물 밀도 [kg/m^3]
  float Cd;         // 항력 계수(의복/장비 포함)
  float L;          // 유효 길이 [m]
  float D;          // 유효 직경 [m]
  float b_visc;     // 점성 계수(저속)
  float alpha_diff; // 각속도 1차 LPF 비율 (0.0~1.0)
  float dead_lo;    // 슈미트 데드밴드(복귀) [rad/s]
  float dead_hi;    // 슈미트 데드밴드(이탈) [rad/s]
  float i_cap;      // 개별 전류 soft-cap [A] (최종 ±9A 포화는 기존 코드)
  float sign_map;   // 관절/모터 방향 보정 (+1 또는 -1)
} HydroParam;

/* 초기값(현장 튜닝 권장) */
static HydroParam gR = {1000.0f, 1.0f, 0.40f, 0.10f, 0.05f, 0.35f, 0.08f, 0.12f, 2.0f, +1.0f};
static HydroParam gL = {1000.0f, 1.0f, 0.40f, 0.10f, 0.05f, 0.35f, 0.08f, 0.12f, 2.0f, +1.0f};

/* 내부 상태(속도/데드밴드/램프/슬루·저크 제한) */
static uint8_t  hydro_init = 0;
static float    th_prev_R = 0.f, th_prev_L = 0.f;
static float    w_est_R   = 0.f, w_est_L   = 0.f;
static uint8_t  zero_R    = 1,   zero_L    = 1;   // 슈미트 “제로 상태”
static float    ramp      = 0.f;                   // 0→1 부드러운 시작
static float    i_prev_R  = 0.f, i_prev_L  = 0.f;  // 슬루 제한용
static float    di_prev_R = 0.f, di_prev_L = 0.f;  // 저크 제한용

/* 한계(부드러움 최우선) */
static float DI_LIMIT_A_S   = 3.0f;   // dI/dt 제한 [A/s]
static float JERK_LIMIT_A_S2 = 60.0f; // d^2I/dt^2 제한 [A/s^2]
static float RAMP_RATE_S     = 2.5f;  // 0→1 램프 시간(초) ~2.5s

/* 유틸 */
static inline float clampf(float x, float lo, float hi){ return (x<lo)?lo:((x>hi)?hi:x); }
static inline float sabsf (float x){ return (x>=0)?x:-x; }

/* 연속형 soft-saturation: i = cap * tanh(i_raw / cap)  (미분 연속 → 떨림 감소) */
static inline float softsat_tanh(float i_raw, float capA){
  float z = i_raw / (capA + 1e-6f);
  /* tanhf 사용(하드 클램프 대신 곡면으로 부드럽게) */
  return capA * tanhf(z);
}

/* 수중 저항 전류 생성: τ = −(b·ω + kq·ω|ω|)  →  i = τ / (GEAR_RATIO·Kτ) */
static inline float hydro_drag_current(const HydroParam* hp, float w_rad_s){
  /* kq = (1/8)·ρ·Cd·D·L^4  (단순 원봉 가정) */
  float L2 = hp->L * hp->L;
  float kq = 0.125f * hp->rho * hp->Cd * hp->D * (L2 * L2);
  float tau = -(hp->b_visc * w_rad_s + kq * w_rad_s * sabsf(w_rad_s));            // [N·m]
  float i_raw = tau / (GEAR_RATIO * MOTOR_TORQUE_CONSTANT);                        // [A]
  return hp->sign_map * softsat_tanh(i_raw, hp->i_cap);                             // 부호 보정 + soft-sat
}

/*---------- (1) END of STUDENT CODE (Declare the functions to use) -------------*/

/*-----------------중력제어-----------------------*/
float thetaL_deg =0.0f;
float thetaL_rad = 0.0f;
float thetaR_deg =0.0f;
float thetaR_rad = 0.0f;
float torque_R = 0.0f;
float torque_L = 0.0f;
float K_R = 0.0f;
float K_L = 0.0f;
float count_R = -1.0f;
float count_L = 0.0f;
int control_M = 0;

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

// Updata Data
static void UpdateRobotData(RobotData_t* robotDataObj, StudentsData_t* StudentsDataObj);
static void UpdateExtensionBoardData(Extpack_Data_t* ExtPackDataObj);

// PIF Vector Trigger
static void PvectorTrigger(P_Vector_Decoder* pvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft);
static void InitFvectorMaxTorque(F_Vector_Decoder* fvectorObj, uint8_t isLeft);
static void FvectorTrigger(F_Vector_Decoder* fvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft);

// Position Controller
static void InitPositionControl(PIDObject* posCtrl);
static void InitPosCtrlHoming(PIDObject* posCtrl, bool* triggerSignal, RobotData_t* robotDataObj, P_Vector_Decoder* pvectorObj);
static void PositionCtrl_Sample(RobotData_t* robotDataObj, PIDObject* posCtrl);

// Gravity Compensator
static void InitGravityCompensation(GravComp* gravComp);
static void GravityCompensation_Sample(GravComp* gravComp, RobotData_t* robotDataObj);

// Impedance Controller
static void InitImpedanceSetting(ImpedanceCtrl* impedanceCtrl);
static void ImpedanceControl_Sample(ImpedanceCtrl* impedanceCtrl, RobotData_t* robotDataObj, PIDObject* posCtrl, bool* isFirstImp);
static void error_filter2(ImpedanceCtrl *impedanceCtrl);

// Input Saturation
static void ControlInputSaturation(RobotData_t *robotDataObj, PIDObject *posCtrl, GravComp *gravComp, ImpedanceCtrl *impedanceCtrl, StepCurr *StepCurr, UserDefinedCtrl *UserDefinedCtrl, float f_vector_input);
/* -------------------------------------------------------- */

/*---------- (1) START of STUDENT CODE (Declare the functions to use) -----------*/

/*---------- (1) END of STUDENT CODE (Declare the functions to use) -------------*/

DOP_COMMON_SDO_CB(algorithmCtrlTask)

void InitAlgorithmCtrl(void)
{
    InitTask(&algorithmCtrlTask);

	InitFvectorMaxTorque(&fvectorObj_RH, RH_MOTOR);
	InitFvectorMaxTorque(&fvectorObj_LH, LH_MOTOR);

	/* State Definition */
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,		StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_ID_STUDENTS);

	// PDO
	/* For PDO setting */

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_ID_STUDENTS)

	/* Timer Callback Allocation */
	if (IOIF_StartTimIT(IOIF_TIM3) > 0) {
		//TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM3, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunAlgorithmCtrl, NULL);
}

void RunAlgorithmCtrl(void* params)
{
	/* Loop Start Time Check */
	STUDENTcodeStartTick = DWT->CYCCNT;

	ackSignal = !ackSignal; // 0과 1 토글

	/* Run Device */
	RunTask(&algorithmCtrlTask);

	/* Elapsed Time Check */
	STUDENTcodeEndTick = DWT->CYCCNT;
	if (STUDENTcodeEndTick < STUDENTcodeStartTick) {
		algorithmCtrlTimeElap = ((4294967295 - STUDENTcodeStartTick) + STUDENTcodeEndTick) / 480;	// in microsecond (Roll-over)
	}
	else {
		algorithmCtrlTimeElap = (DWT->CYCCNT - STUDENTcodeStartTick) / 480;							// in microsecond
	}
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{
	StateTransition(&algorithmCtrlTask.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Run(void)
{
	StateTransition(&algorithmCtrlTask.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent(void)
{
	EntRoutines(&algorithmCtrlTask.routine);

	InitPositionControl(&posCtrl_RH);
	InitPositionControl(&posCtrl_LH);

	InitGravityCompensation(&gravCompDataObj_RH);
	InitGravityCompensation(&gravCompDataObj_LH);

	InitImpedanceSetting(&impedanceCtrl_RH);
	InitImpedanceSetting(&impedanceCtrl_LH);

	algorithmCtrlLoopCnt = 0;
}

static void StateEnable_Run(void)
{
	if (CM_connect_signal == 1) {
		Send_ExtensionBoardEnable();
		CM_connect_signal = 0;
	}

	RunRoutines(&algorithmCtrlTask.routine);

	/*---------------------------- Data gathering (DO NOT CHANGE THIS) ----------------------------*/
	UpdateRobotData(&robotDataObj_RH, &StudentsDataObj_RH);
	UpdateRobotData(&robotDataObj_LH, &StudentsDataObj_LH);
	UpdateExtensionBoardData(&ExtPackDataObj);
	/*---------------------------------------------------------------------------------------------*/

	/*------------------------- Control Sample Code ------------------------*/
	if ((SUIT_State_curr >= 3 && SUIT_State_curr <= 17) || (SUIT_State_curr >= 33 && SUIT_State_curr <= 45) ||
		(SUIT_State_curr >= 66 && SUIT_State_curr <= 75)) {
		if (controlMode == POSITION_CTRL) {	// 1 - Position Control
			// Init Position For Safety
			if (isFirstPos_RH == true) {
				InitPosCtrlHoming(&posCtrl_RH, &pVectorTrig_RH, &robotDataObj_RH, &pvectorObj_RH);
				isFirstPos_RH = false;
			} else {
				PvectorTrigger(&pvectorObj_RH, &MotionMap_File, &robotDataObj_RH, &pVectorTrig_RH, &MotionMap_ID_RH, RH_MOTOR);
				PositionCtrl_Sample(&robotDataObj_RH, &posCtrl_RH);
			}
			if (isFirstPos_LH == true) {
				InitPosCtrlHoming(&posCtrl_LH, &pVectorTrig_LH, &robotDataObj_LH, &pvectorObj_LH);
				isFirstPos_LH = false;
			} else {
				PvectorTrigger(&pvectorObj_LH, &MotionMap_File, &robotDataObj_LH, &pVectorTrig_LH, &MotionMap_ID_LH, LH_MOTOR);
				PositionCtrl_Sample(&robotDataObj_LH, &posCtrl_LH);
			}
			
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		} else if (controlMode == GRAVITY_COMPENSATION) {	// 2 - Gravity Compensation
			GravityCompensation_Sample(&gravCompDataObj_RH, &robotDataObj_RH);
			GravityCompensation_Sample(&gravCompDataObj_LH, &robotDataObj_LH);
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		} else if (controlMode == IMPEDANCE_CTRL) {   // 3 - Impedance Control
			ImpedanceControl_Sample(&impedanceCtrl_RH, &robotDataObj_RH, &posCtrl_RH, &isFirstImp_RH);
			ImpedanceControl_Sample(&impedanceCtrl_LH, &robotDataObj_LH, &posCtrl_LH, &isFirstImp_LH);
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
		} else if (controlMode == TORQUE_CTRL) {   // 4 - Torque Control
			FvectorTrigger(&fvectorObj_RH, &MotionMap_File, &robotDataObj_RH, &fVectorTrig_RH, &MotionMap_ID_RH, RH_MOTOR);
			FvectorTrigger(&fvectorObj_LH, &MotionMap_File, &robotDataObj_LH, &fVectorTrig_LH, &MotionMap_ID_LH, LH_MOTOR);
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
		} else if (controlMode == STEP_CURRENT_CTRL) {   // 5 - Step Current Control
		    // 매 루프에서 전류 입력 지정 (초기화 방지)
		    posCtrl_RH.control_input = 0.0f;
		    posCtrl_LH.control_input = 0.0f;
		    gravCompDataObj_RH.control_input = 0.0f;
		    gravCompDataObj_LH.control_input = 0.0f;
		    impedanceCtrl_RH.control_input = 0.0f;
		    impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
		    UserDefinedCtrl_RH.control_input = 0.0f;
		    UserDefinedCtrl_LH.control_input = 0.0f;

		    StepCurr_RH.control_input = (float)currentAmp;

		    if (!done && (algorithmCtrlLoopCnt - lastUpdateCnt) >= 3000)
		    {
		        if (currentAmp < 4) {
		            currentAmp++;
		            lastUpdateCnt = algorithmCtrlLoopCnt;
		        }
		        else {
		            currentAmp = 0;     // 3초 유지 후 0A로
		            done = true;        // 이후에는 다시 증가하지 않음
		            lastUpdateCnt = algorithmCtrlLoopCnt;
		        }
		    }

		} else if (controlMode == USER_DEFINED_CTRL) {
			/* 다른 컨트롤 0 (겹침 방지) */
			    posCtrl_RH.control_input = 0.f;  posCtrl_LH.control_input = 0.f;
			    gravCompDataObj_RH.control_input = 0.f;  gravCompDataObj_LH.control_input = 0.f;
			    impedanceCtrl_RH.control_input = 0.f;    impedanceCtrl_LH.control_input = 0.f;
			    StepCurr_RH.control_input = 0.f;         StepCurr_LH.control_input = 0.f;
			    f_vector_input_RH = 0.f;                 f_vector_input_LH = 0.f;

			    /* 1) 초기화(첫 루프) */
			    float th_R = robotDataObj_RH.thighTheta_act * (float)M_PI / 180.0f;  // [rad]
			    float th_L = robotDataObj_LH.thighTheta_act * (float)M_PI / 180.0f;  // [rad]
			    if (!hydro_init){
			        th_prev_R = th_R; th_prev_L = th_L;
			        w_est_R = 0.f;    w_est_L = 0.f;
			        zero_R = 1;       zero_L = 1;
			        ramp = 0.f;
			        i_prev_R = i_prev_L = 0.f;
			        di_prev_R = di_prev_L = 0.f;
			        hydro_init = 1;
			    }

			    /* 2) 속도 추정(저역 미분) */
			    float w_raw_R = (th_R - th_prev_R) / DT;
			    float w_raw_L = (th_L - th_prev_L) / DT;
			    th_prev_R = th_R; th_prev_L = th_L;
			    w_est_R = gR.alpha_diff * w_raw_R + (1.f - gR.alpha_diff) * w_est_R;
			    w_est_L = gL.alpha_diff * w_raw_L + (1.f - gL.alpha_diff) * w_est_L;

			    /* 3) 슈미트 데드밴드(0 근처 헌팅 억제, 히스테리시스) */
			    float wR = w_est_R, wL = w_est_L;
			    if (zero_R){                 // 제로 상태
			        if (sabsf(wR) > gR.dead_hi) zero_R = 0;
			        else                        wR = 0.f;
			    } else {                      // 활성 상태
			        if (sabsf(wR) < gR.dead_lo){ zero_R = 1; wR = 0.f; }
			    }
			    if (zero_L){
			        if (sabsf(wL) > gL.dead_hi) zero_L = 0;
			        else                        wL = 0.f;
			    } else {
			        if (sabsf(wL) < gL.dead_lo){ zero_L = 1; wL = 0.f; }
			    }

			    /* 4) 수중 저항 전류(연속형 soft-sat) */
			    float iR_tgt = hydro_drag_current(&gR, wR);
			    float iL_tgt = hydro_drag_current(&gL, wL);

			    /* 5) 부드러운 램프-업(전원 투입 직후 서서히 ↑) */
			    if (ramp < 1.f){
			        ramp += (DT / RAMP_RATE_S);             // RAMP_RATE_S초 동안 0→1
			        if (ramp > 1.f) ramp = 1.f;
			    }
			    iR_tgt *= ramp; iL_tgt *= ramp;

			    /* 6) Slew + Jerk 제한 (dI/dt, d²I/dt² 모두 제한) */
			    float di_max = DI_LIMIT_A_S * DT;
			    float dj_max = JERK_LIMIT_A_S2 * DT;        // 허용 저크 * dt

			    /* 먼저 slew로 1차 제한 */
			    float iR_slew = i_prev_R + clampf(iR_tgt - i_prev_R, -di_max, di_max);
			    float iL_slew = i_prev_L + clampf(iL_tgt - i_prev_L, -di_max, di_max);

			    /* 다음 frame의 변화량(di)을 계산 후 저크로 2차 제한 */
			    float di_R = iR_slew - i_prev_R;
			    float di_L = iL_slew - i_prev_L;
			    di_R = clampf(di_R, di_prev_R - dj_max, di_prev_R + dj_max);
			    di_L = clampf(di_L, di_prev_L - dj_max, di_prev_L + dj_max);

			    float iR_cmd = i_prev_R + di_R;
			    float iL_cmd = i_prev_L + di_L;

			    i_prev_R = iR_cmd; di_prev_R = di_R;
			    i_prev_L = iL_cmd; di_prev_L = di_L;

			    /* 7) 사용자 정의 전류 출력 (최종 합산/assist_level/±9A 포화는 기존 함수) */
			    UserDefinedCtrl_RH.control_input = iR_cmd;
			    UserDefinedCtrl_LH.control_input = iL_cmd;
		}
        else {
			// default : SUIT H10 Assist Mode
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		}
	}
	/*---------------------- END of Control Sample Code ---------------------*/

	/*--------------------- (2) START of STUDENT CODE (Write your code in this section) --------------------*/

	/*------------------------------------ (2) END of STUDENT CODE-----------------------------------------*/
	// Control Input Saturation (Do not Delete)
	ControlInputSaturation(&robotDataObj_RH, &posCtrl_RH, &gravCompDataObj_RH, &impedanceCtrl_RH, &StepCurr_RH , &UserDefinedCtrl_RH, f_vector_input_RH);
	ControlInputSaturation(&robotDataObj_LH, &posCtrl_LH, &gravCompDataObj_LH, &impedanceCtrl_LH, &StepCurr_LH , &UserDefinedCtrl_LH, f_vector_input_LH);
	algorithmCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
    ExtRoutines(&algorithmCtrlTask.routine);
}

static void StateError_Run(void)
{

}

static void UpdateRobotData(RobotData_t* robotDataObj, StudentsData_t* StudentsDataObj)
{
	robotDataObj->thighTheta_act = StudentsDataObj->thighThetaAct;
	robotDataObj->position_act = StudentsDataObj->positionAct;
	robotDataObj->accX = StudentsDataObj->accX;
	robotDataObj->accY = StudentsDataObj->accY;
	robotDataObj->gyrZ = StudentsDataObj->gyrZ;
}

static void UpdateExtensionBoardData(Extpack_Data_t* ExtPackDataObj)
{
	// EMG_Rawsignal = ExtPackDataObj->emg_data.emg_R2_rawSign[0] / 2048.0f;
	EMG_R1_Rawsignal = ExtPackDataObj->emg_data.emg_R1_rawSign[0] / 2048.0f;
	EMG_L1_Rawsignal = ExtPackDataObj->emg_data.emg_L1_rawSign[0] / 2048.0f;

	fsr1_R1 = ExtPackDataObj->fsr_data.fsr_R1_raw;
	fsr2_R2 = ExtPackDataObj->fsr_data.fsr_R2_raw;
	fsr1_L1 = ExtPackDataObj->fsr_data.fsr_L1_raw;
	fsr2_L2 = ExtPackDataObj->fsr_data.fsr_L2_raw;

	free_var1 = robotDataObj_RH.accX;
	free_var2 = robotDataObj_RH.accY;
	free_var3 = robotDataObj_RH.accX;
	// free_var4 = robotDataObj_RH.accY;
	// free_var5 = robotDataObj_RH.accX;
}

static void PvectorTrigger(P_Vector_Decoder* pvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft)
{
	if (*triggerSignal == true) {
		*triggerSignal = false;			// Reset

		pvectorObj->yd_f = robotDataObj->position_act * M_PI / 180.0f;

		if (*MM_ID >= 0 && *MM_ID < 40) {
			*pvectorObj = MotionMap_File->MS[*MM_ID].MD[isLeft].p_vector_decoder;		// Assign
			// *MM_ID = 99;		// Reset
		}
	}
}

static void InitFvectorMaxTorque(F_Vector_Decoder* fvectorObj, uint8_t isLeft)
{
	memset(fvectorObj, 0, sizeof(*fvectorObj));

	// Max 8~10Nm이나 매우 위험할 수 있으므로 Test시에는 2~3Nm로 실험 하세요
	// default 8Nm
	// 로봇 설정 최대치 10Nm
	if (isLeft) {
		LeftHipFlexionTorque = 2.0f;	// 3Nm, saturation
		LeftHipExtensionTorque = 2.0f;	// 3Nm
	} else {
		RightHipFlexionTorque = 2.0f;	// 3Nm
		RightHipExtensionTorque = 2.0f;	// 3Nm
	}
}

static void FvectorTrigger(F_Vector_Decoder* fvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft)
{
	if (*triggerSignal == true) {
		*triggerSignal = false;			// Reset

		if (*MM_ID >= 0 && *MM_ID < 40) {
			*fvectorObj = MotionMap_File->MS[*MM_ID].MD[isLeft].f_vector_decoder;		// Assign
			// *MM_ID = 99;		// Reset
		}

		// global variable ID에 따른 Torque Max assign
		float t_tauMax = 0.0f;
		for (int i = 0; i < F_VECTOR_BUFF_SIZE; i++) {
			if (fvectorObj->f_buffer[i].globalVariableID == 0x01) {
				t_tauMax = RightHipFlexionTorque;
			}
			if (fvectorObj->f_buffer[i].globalVariableID == 0x02) {
				t_tauMax = RightHipExtensionTorque;
			}
			if (fvectorObj->f_buffer[i].globalVariableID == 0x03) {
				t_tauMax = LeftHipFlexionTorque;
			}
			if (fvectorObj->f_buffer[i].globalVariableID == 0x04) {
				t_tauMax = LeftHipExtensionTorque;
			}

			// Torque -> Current Input Conversion
			fvectorObj->f_buffer[i].tau_max = (t_tauMax / GEAR_RATIO / MOTOR_TORQUE_CONSTANT) * (fvectorObj->f_buffer[i].coefficient * 0.01);

			// F vector Reset Logic
			// if (fvectorObj->f_buffer[i].mode_idx == 255) {
			// 	if (isLeft) f_vector_input_LH = 0.0f;
			// 	else f_vector_input_RH = 0.0f;
			// 	fvectorObj->f_buffer[i].mode_idx = 0;
			// 	fvectorObj->f_buffer[i].tau_max = 0;
			// 	fvectorObj->f_buffer[i].delay = 0;
			// 	fvectorObj->f_buffer[i].u = 0;
			// 	fvectorObj->f_buffer[i].u_old1 = 0;
			// 	fvectorObj->f_buffer[i].u_old2 = 0;
			// 	fvectorObj->f_buffer[i].tau = 0;
			// 	fvectorObj->f_buffer[i].tau_old1 = 0;
			// 	fvectorObj->f_buffer[i].tau_old2 = 0;
			// 	fvectorObj->f_buffer[i].t_end = 0;
			// 	fvectorObj->f_buffer[i].time_stamp = 0;
			// 	fvectorObj->f_buffer[i].is_full = 0;
			// }
		}
	}
}

static void InitPositionControl(PIDObject* posCtrl)
{
	memset(posCtrl, 0, sizeof(*posCtrl));

	posCtrl->Kp = 1.5;
	posCtrl->Kd = 0.2;
}

static void InitPosCtrlHoming(PIDObject* posCtrl, bool* triggerSignal, RobotData_t* robotDataObj, P_Vector_Decoder* pvectorObj)
{
	pvectorObj->yd_f = robotDataObj->position_act * M_PI / 180.0f;
	pvectorObj->N = 1;
	pvectorObj->p_buffer->yd = 0;
	pvectorObj->p_buffer->s0 = 60;
	pvectorObj->p_buffer->sd = 60;
	pvectorObj->p_buffer->L = 3000;
}

static void PositionCtrl_Sample(RobotData_t* robotDataObj, PIDObject* posCtrl)
{
	posCtrl->err = (posCtrl->ref) - (robotDataObj->position_act * M_PI / 180);
	posCtrl->err_diff = (posCtrl->err - posCtrl->err_prev) / DT;
	posCtrl->err_prev =  posCtrl->err;

	posCtrl->control_input = posCtrl->Kp * posCtrl->err + posCtrl->Kd * posCtrl->err_diff;
}

static void InitGravityCompensation(GravComp* gravComp)
{
	memset(gravComp, 0, sizeof(*gravComp));

	gravComp->grav_gain = 0.5;
	gravComp->grav_alpha = 0.99;
}

static void GravityCompensation_Sample(GravComp* gravComp, RobotData_t* robotDataObj)
{
	if (robotDataObj->thighTheta_act > 0) 
	{
		gravComp->grav_comp_torque =  gravComp->grav_gain * (sin((robotDataObj->thighTheta_act) * M_PI / 180));
		gravComp->f_grav_comp_torque = gravComp->grav_alpha * gravComp->f_grav_comp_torque + (1 - gravComp->grav_alpha) * gravComp->grav_comp_torque;
	}
	else
	{
		gravComp->grav_comp_torque = 0;
		gravComp->f_grav_comp_torque = gravComp->grav_alpha * gravComp->f_grav_comp_torque + (1 - gravComp->grav_alpha) * gravComp->grav_comp_torque;
	}

	gravComp->control_input = gravComp->f_grav_comp_torque;
}

static void InitImpedanceSetting(ImpedanceCtrl* impedanceCtrl)
{
	memset(impedanceCtrl, 0, sizeof(*impedanceCtrl));
	
	impedanceCtrl->epsilon = 5.0f * M_PI / 180.0f;
	impedanceCtrl->Kp = 1.5f;
	impedanceCtrl->Kd = 0.2f;
	impedanceCtrl->lambda = 1.0f;
	impedanceCtrl->duration = 300.0f;
}

static void ImpedanceControl_Sample(ImpedanceCtrl* impedanceCtrl, RobotData_t* robotDataObj, PIDObject* posCtrl, bool* isFirstImp)
{
	float t_epsilon = 0.0f;
	float t_Kp = 0.0f;
	float t_Kd = 0.0f;
	float t_lambda = 0.0f;

	if (*isFirstImp == true && impedanceCtrl->ON == 0) {
		posCtrl->ref = robotDataObj->position_act * M_PI / 180.0f;
		t_epsilon = impedanceCtrl->epsilon; // unit: rad   (0.001745329252 = 0.1 * pi/180)
		t_Kp      = impedanceCtrl->Kp;
		t_Kd      = impedanceCtrl->Kd;
		t_lambda  = impedanceCtrl->lambda;

		if (impedanceCtrl->duration > 0) {
			float invT      = 1/impedanceCtrl->duration;

			impedanceCtrl->gap_epsilon = (t_epsilon - impedanceCtrl->epsilon) * invT;
			impedanceCtrl->gap_Kp      = (t_Kp      - impedanceCtrl->Kp)      * invT;
			impedanceCtrl->gap_Kd      = (t_Kd      - impedanceCtrl->Kd)      * invT;
			impedanceCtrl->gap_lambda  = (t_lambda  - impedanceCtrl->lambda)  * invT;
		}

		impedanceCtrl->i  = 0; // initialize 1ms counter
		impedanceCtrl->ON = 1;
		*isFirstImp = false;
	}

	if (impedanceCtrl->ON == 1) {
		if (impedanceCtrl->duration == 0) {
			impedanceCtrl->epsilon = t_epsilon;
			impedanceCtrl->Kp      = t_Kp;
			impedanceCtrl->Kd      = t_Kd;
			impedanceCtrl->lambda  = t_lambda;
		} else {
			impedanceCtrl->epsilon = impedanceCtrl->epsilon + impedanceCtrl->gap_epsilon;
			impedanceCtrl->Kp      = impedanceCtrl->Kp      + impedanceCtrl->gap_Kp;
			impedanceCtrl->Kd      = impedanceCtrl->Kd      + impedanceCtrl->gap_Kd;
			impedanceCtrl->lambda  = impedanceCtrl->lambda  + impedanceCtrl->gap_lambda;
			impedanceCtrl->i++;
		}

		if (impedanceCtrl->i >= impedanceCtrl->duration)	{
			impedanceCtrl->ON = 0;
			impedanceCtrl->i = 0;
		}
	}

	/* Impedance Controller */
	impedanceCtrl->e = posCtrl->ref - (robotDataObj->position_act * M_PI / 180.0f);

	error_filter2(impedanceCtrl);

	float t_ef_diff = 0.0;

	if (((impedanceCtrl->ef > 0) & (impedanceCtrl->ef_diff > 0)) | ((impedanceCtrl->ef <= 0) & (impedanceCtrl->ef_diff <= 0))) {
		t_ef_diff = +impedanceCtrl->ef_diff;
	} else {
		t_ef_diff = -impedanceCtrl->ef_diff;
	}

	impedanceCtrl->control_input = impedanceCtrl->Kp * impedanceCtrl->ef + impedanceCtrl->Kd * t_ef_diff;
}

static void error_filter2(ImpedanceCtrl *impedanceCtrl)
{
	// f2(e,t) = lambda * e + (1 - lambda)*sign(e)*max(|e| - epsilon, 0)
	float t_abs_e = 0.0;
	float t_sign_e = 0.0;
	float t_max = 0.0;
	float t_diff = 0.0;
	float y_ef = 0.0;

	/* Calculate 'sign(e) & |e|' */
	if (impedanceCtrl->e > 0)	{t_abs_e = +impedanceCtrl->e; t_sign_e = +1; }
	else						{t_abs_e = -impedanceCtrl->e; t_sign_e = -1; }

	/* Calculate 'max(|e| - epsilon, 0)' */
	t_diff = t_abs_e - impedanceCtrl->epsilon;
	if (t_diff > 0) {t_max = t_diff;}
	else            {t_max = 0;}

	y_ef = (impedanceCtrl->lambda * impedanceCtrl->e) + (1 - impedanceCtrl->lambda) * t_sign_e * t_max;

	impedanceCtrl->ef_diff = (y_ef - impedanceCtrl->ef) * 0.001;

	impedanceCtrl->ef = y_ef;
}

static void ControlInputSaturation(RobotData_t *robotDataObj, PIDObject *posCtrl, GravComp *gravComp, ImpedanceCtrl *impedanceCtrl, StepCurr *StepCurr, UserDefinedCtrl *UserDefinedCtrl, float f_vector_input)
{
	// Saturate control input to ±9 using fminf and fmaxf
	robotDataObj->u_totalInput = fminf(fmaxf((posCtrl->control_input + /* control input of PID position controller */
								gravComp->control_input + /* control input of gravity compensation */
								impedanceCtrl->control_input + /* control input of impedance controller */
								StepCurr->control_input + /* control input of step current input controller */
								UserDefinedCtrl->control_input + /* control input of user defined controller */
								f_vector_input) * assist_level, -9.0f), 9.0f); /* control input of F-vector decoded */
								// assist_level 0 ~ 1.0 (0~100%, 0.1, 10%단위)
}

/*----------- (3) START of STUDENT CODE (Define the functions to use) ------------*/





/*------------ (3) END of STUDENT CODE (Define the functions to use) -------------*/
