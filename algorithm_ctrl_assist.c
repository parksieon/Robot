#include "algorithm_ctrl.h"

/* ------------------- Default Variables ------------------ */
TaskObj_t algorithmCtrlTask;

uint8_t CM_connect_signal = 0;
uint8_t CM_disconnect_signal = 0;

uint8_t motionMap_selection = 99;
uint8_t startPvector_decoding = 0;
ControlMode controlMode = DEFAULT_CONTRL_MODE;

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

float assist_level = 0.008f; // *** FIX: 1.0f -> 0.008f로 변경 (사용자 튜닝값 적용) ***

// *** 추가: 사용자 정의 모드를 위한 실시간 변경 가능 중력보상 게인 K ***
float user_defined_K = 0.5f; // K 상수를 #define 대신 전역 변수로 선언 (초기값 0.5f)

// *** 추가: 사용자 정의 충격 완화(보조) 알고리즘용 게인 ***
// *** FIX: 좌우 게인 및 민감도 분리 ***
float assist_gain_RH = 1.5f;      // 오른쪽 보조 게인 (값은 튜닝 필요)
float assist_gain_LH = 1.5f;      // 왼쪽 보조 게인 (값은 튜닝 필요)
float angle_threshold_RH = 10.0f; // 오른쪽 착지 감지 다리 각도 차이 (deg) (값은 튜닝 필요)
float angle_threshold_LH = 10.0f; // 왼쪽 착지 감지 다리 각도 차이 (deg) (값은 튜닝 필요)
float vel_threshold_RH = 4.0f;    // 오른쪽 착지 감지 다리 각속도 (deg/s) (값은 튜닝 필요)
float vel_threshold_LH = 5.0f;    // 왼쪽 착지 감지 다리 각속도 (deg/s) (값은 튜닝 필요)


static bool first_run_assist = true;

// 이전 허벅지 각도
static float thighTheta_RH_prev = 0.0f;
static float thighTheta_LH_prev = 0.0f;

// 저역통과 필터 적용 각속도
static float angularVel_RH_filtered = 0.0f; // *** FIX: Added missing variable name ***
static float angularVel_LH_filtered = 0.0f;

// 착지 감지 상태
static bool landing_detect_RH = false;
static bool landing_detect_LH = false;

// 보조 토크
float assist_RH = 0.0f;
float assist_LH = 0.0f;

// 중력 보상
float grav_RH = 0.0f;
float grav_LH = 0.0f;

/* -------------------------------------------------------- */

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

/*---------- (1) END of STUDENT CODE (Declare the functi	ons to use) -------------*/

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
		// *** FIX: Use float literals for correct float division ***
		algorithmCtrlTimeElap = ((4294967295.0f - STUDENTcodeStartTick) + STUDENTcodeEndTick) / 480.0f;	// in microsecond (Roll-over)
	}
	else {
		// *** FIX: Use float literals and cast for correct float division ***
		algorithmCtrlTimeElap = (float)(DWT->CYCCNT - STUDENTcodeStartTick) / 480.0f;							// in microsecond
	}
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
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
	// NOTE: SUIT_State_curr is assumed to be defined externally (e.g. in algorithm_ctrl.h)
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

		} else if (controlMode == USER_DEFINED_CTRL) {   // 6 - User Defined Control

		    // ---------------------- 1) 양쪽 다리 각도 측정 ----------------------
		    float angle_RH = robotDataObj_RH.thighTheta_act;  // 오른쪽 허벅지 각도 [deg]
		    float angle_LH = robotDataObj_LH.thighTheta_act;  // 왼쪽 허벅지 각도 [deg]
		    float angle_diff = angle_RH - angle_LH;           // 양 다리 각도 차이
		    float angle_diff_abs = fabsf(angle_diff);         // 절대값

		    // ---------------------- 2) 각속도 계산 (수치미분) ----------------------
		    // *** FIX: Use DT constant defined in header (assuming it's 0.001f) ***
		    const float dt = DT;  // 샘플 주기 1ms (기존 코드의 0.001f 대신 DT 사용)
		    float angularVelocity_RH = 0.0f;
		    float angularVelocity_LH = 0.0f;

		    if (first_run_assist) {  // 최초 실행 시 이전 각도 초기화
		        thighTheta_RH_prev = angle_RH;
		        thighTheta_LH_prev = angle_LH;
		        first_run_assist = false;
		    } else {
		        angularVelocity_RH = (angle_RH - thighTheta_RH_prev) / dt;
		        angularVelocity_LH = (angle_LH - thighTheta_LH_prev) / dt;
		    }

		    // 이전 각도 업데이트
		    thighTheta_RH_prev = angle_RH;
		    thighTheta_LH_prev = angle_LH;

		    // ---------------------- 3) 각속도 저역통과 필터 ----------------------

		    const float alpha_vel = 0.3f;  // 필터 계수 (작을수록 부드럽게)
		    angularVel_RH_filtered = alpha_vel * angularVel_RH_filtered + (1.0f - alpha_vel) * angularVelocity_RH;
		    angularVel_LH_filtered = alpha_vel * angularVel_LH_filtered + (1.0f - alpha_vel) * angularVelocity_LH;

		    float vel_RH = angularVel_RH_filtered;  // [deg/s], 양수=신전(아래), 음수=굴곡(위)
		    float vel_LH = angularVel_LH_filtered;

		    // ---------------------- 4) 중력보상 ----------------------
		    float theta_RH_rad = angle_RH * M_PI / 180.0f;
		    float theta_LH_rad = angle_LH * M_PI / 180.0f;
		    // *** 수정: #define 'K' 대신 새로운 전역 변수 'user_defined_K' 사용 ***
		    grav_RH = user_defined_K * sinf(theta_RH_rad);
		    grav_LH = user_defined_K * sinf(theta_LH_rad);

			// *** --- NEW LOGIC START (착지 보조 알고리즘) --- ***
		    // ---------------------- 5) 착지 직전 감지 및 보조 (충격 완화) ----------------------
		    // 사용자 좌표계: 음수 = 발 내림 (착지 방향), 양수 = 발 뒤로 (신전)
		    // 목표: 각도가 음수로 내려가서 임계값 이하일 때, 0°에 가까워질수록 큰 저항
		    
		    assist_RH = 0.0f; // 보조 토크 초기화
		    assist_LH = 0.0f; // 보조 토크 초기화

		    // 오른쪽 다리 착지 직전 조건
		    // 조건 1: 오른쪽 다리가 왼쪽보다 앞(아래)에 있음 (angle_RH < angle_LH)
		    // 조건 2: 각도가 임계값 이하 (예: angle_RH < -10°)
		    // 조건 3: 음수 방향으로 움직임 (vel_RH < 0, 발이 내려감)
		    if (angle_RH < -angle_threshold_RH && vel_RH < -vel_threshold_RH) {
		        landing_detect_RH = true;
		        // 0°에 가까워질수록 큰 힘: |angle_RH|가 작을수록 큰 게인
		        // assist = gain × (1 / (|angle| + 1)) × |vel|
		        // 예: angle=-5° → 1/6, angle=-2° → 1/3 (더 큰 저항)
		        float distance_to_zero = fabsf(angle_RH) + 1.0f; // +1은 0 나눗셈 방지
		        assist_RH = assist_gain_RH * (1.0f / distance_to_zero) * fabsf(vel_RH);
		        // 양수 방향(위로) 저항력 제공
		    } else {
				landing_detect_RH = false;
			}

		    // 왼쪽 다리 착지 직전 조건
		    // 조건 1: 왼쪽 다리가 오른쪽보다 앞(아래)에 있음 (angle_LH < angle_RH)
		    // 조건 2: 각도가 임계값 이하 (예: angle_LH < -10°)
		    // 조건 3: 음수 방향으로 움직임 (vel_LH < 0, 발이 내려감)
		    if (angle_LH < -angle_threshold_LH && vel_LH < -vel_threshold_LH) {
		        landing_detect_LH = true;
		        float distance_to_zero = fabsf(angle_LH) + 1.0f;
		        assist_LH = assist_gain_LH * (1.0f / distance_to_zero) * fabsf(vel_LH);
		    } else {
				landing_detect_LH = false;
			}

		    // ---------------------- 6) 최종 제어 입력 (중력보상 + 충격완화보조) ----------------------
		    UserDefinedCtrl_RH.control_input = grav_RH + assist_RH;
		    UserDefinedCtrl_LH.control_input = grav_LH + assist_LH;

		    // ---------------------- 7) 다른 제어 입력 초기화 (User Defined 모드이므로) ----------------------
		    posCtrl_RH.control_input = 0.0f;
		    posCtrl_LH.control_input = 0.0f;
		    gravCompDataObj_RH.control_input = 0.0f;
		    gravCompDataObj_LH.control_input = 0.0f;
		    impedanceCtrl_RH.control_input = 0.0f;
		    impedanceCtrl_LH.control_input = 0.0f;
		    f_vector_input_RH = 0.0f;
		    f_vector_input_LH = 0.0f;
		    StepCurr_RH.control_input = 0.0f;
		    StepCurr_LH.control_input = 0.0f;
			// *** --- NEW LOGIC END --- ***

		} else {
		    // ---------------------- Default : SUIT H10 Assist Mode ----------------------
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
	} // *** FIX: Added missing closing brace for SUIT_State_curr check ***
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
	// free_var5 = robotDataObj_RH.accX; // *** FIX: Corrected typo Rㅌ₩H -> RH ***
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
	if (isLeft) { // Assuming isLeft == LH_MOTOR (1)
		LeftHipFlexionTorque = 2.0f;	// 3Nm, saturation
		LeftHipExtensionTorque = 2.0f;	// 3Nm
	} else { // Assuming isLeft == RH_MOTOR (0)
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
			// *** FIX: Use float literal 0.01f ***
			fvectorObj->f_buffer[i].tau_max = (t_tauMax / GEAR_RATIO / MOTOR_TORQUE_CONSTANT) * (fvectorObj->f_buffer[i].coefficient * 0.01f);

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

	// *** FIX: Use float literals ***
	posCtrl->Kp = 1.5f;
	posCtrl->Kd = 0.2f;
}

static void InitPosCtrlHoming(PIDObject* posCtrl, bool* triggerSignal, RobotData_t* robotDataObj, P_Vector_Decoder* pvectorObj)
{
	pvectorObj->yd_f = robotDataObj->position_act * M_PI / 180.0f;
	pvectorObj->N = 1;
	// *** FIX: Access p_buffer as an array ***
	pvectorObj->p_buffer[0].yd = 0;
	pvectorObj->p_buffer[0].s0 = 60;
	pvectorObj->p_buffer[0].sd = 60;
	pvectorObj->p_buffer[0].L = 3000;
}

static void PositionCtrl_Sample(RobotData_t* robotDataObj, PIDObject* posCtrl)
{
	// *** FIX: Use float literals for conversion ***
	posCtrl->err = (posCtrl->ref) - (robotDataObj->position_act * M_PI / 180.0f);
	posCtrl->err_diff = (posCtrl->err - posCtrl->err_prev) / DT;
	posCtrl->err_prev =  posCtrl->err;

	posCtrl->control_input = posCtrl->Kp * posCtrl->err + posCtrl->Kd * posCtrl->err_diff;
}

static void InitGravityCompensation(GravComp* gravComp)
{
	memset(gravComp, 0, sizeof(*gravComp));

	// *** FIX: Use float literals ***
	gravComp->grav_gain = 0.5f;
	gravComp->grav_alpha = 0.99f;
}

static void GravityCompensation_Sample(GravComp* gravComp, RobotData_t* robotDataObj)
{
	if (robotDataObj->thighTheta_act > 0)
	{
		// *** FIX: Use sinf() for float and float literals ***
		gravComp->grav_comp_torque =  gravComp->grav_gain * (sinf((robotDataObj->thighTheta_act) * M_PI / 180.0f));
	}
	else
	{
		// *** FIX: Use float literal ***
		gravComp->grav_comp_torque = 0.0f;
	}

	// *** FIX: Use float literal 1.0f ***
	gravComp->f_grav_comp_torque = gravComp->grav_alpha * gravComp->f_grav_comp_torque + (1.0f - gravComp->grav_alpha) * gravComp->grav_comp_torque;

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
	// NOTE: If ef_diff logic is changed to use a separate 'ef_prev' field,
	// it should be initialized here:
	// impedanceCtrl->ef_prev_for_diff = 0.0f;
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
			// *** FIX: Use 1.0f for float division ***
			float invT      = 1.0f / impedanceCtrl->duration;

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

	float t_ef_diff = 0.0f;

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
	float t_abs_e = 0.0f;
	float t_sign_e = 0.0f;
	float t_max = 0.0f;
	float t_diff = 0.0f;
	float y_ef = 0.0f;

	/* Calculate 'sign(e) & |e|' */
	if (impedanceCtrl->e > 0)	{t_abs_e = +impedanceCtrl->e; t_sign_e = +1.0f; } // Use 1.0f
	else						{t_abs_e = -impedanceCtrl->e; t_sign_e = -1.0f; } // Use -1.0f

	/* Calculate 'max(|e| - epsilon, 0)' */
	t_diff = t_abs_e - impedanceCtrl->epsilon;
	if (t_diff > 0) {t_max = t_diff;}
	else            {t_max = 0.0f;} // Use 0.0f

	// *** FIX: Use 1.0f for float math ***
	y_ef = (impedanceCtrl->lambda * impedanceCtrl->e) + (1.0f - impedanceCtrl->lambda) * t_sign_e * t_max;

	// *** FIX: Derivative is (current - previous) / DT ***
	// (Assumes DT is defined as 0.001f in the header)
	impedanceCtrl->ef_diff = (y_ef - impedanceCtrl->ef) / DT;

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





/*—————— (3) END of STUDENT CODE (Define the functions to use) ——————*/
