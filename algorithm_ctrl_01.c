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

float assist_level = 1.0f;



/* -------------------------------------------------------- */
float   user_control_mode = 0.0f;
#define PID_CONTROL_MODE 0
#define GRAVITY_COMP_MODE 1
#define ASSIST_REDUCING_SHOCK_MODE 2
#define UNDERWATER_RESISTANCE_MODE 3

/* --- PID_CONTROL_MODE --- */
float Kp = 0.1f;
float Ki =0.008f;
static float Kd = 0.01f;              // D 게인(도/초 기준이면 그대로, 라디안 쓰면 맞게 조정)

float Lerror = 0.0f;
float Ldeg = 0.0f;
float count =0.0f;
float sum =0.0f;
float K = 0.0f;

static float Lerror_prev = 0.0f;
static float d_meas_lp = 0.0f;       // 저역통과된 미분(도/초)
static const float dT = 0.001f;
static const float DERIV_ALPHA = 0.15f; // 0~1, 작을수록 더 부드러움(지연↑)

static const float U_MAX = 9.0f;     // 출력 포화(아래 Saturation과 일치)
static const float I_MIN = -2000.0f; // 적분 한계(상황 맞게 조정)
static const float I_MAX =  2000.0f;

static bool user_first = true;


/* --- GRAVITY_COMP_MODE --- */
// 물리 상수 기반 파라미터
volatile float K_Nm_RH = 12.0f;   // [Nm] = m*g*l
volatile float K_Nm_LH = 12.0f;   // [Nm]

// 토크→전류 변환 계수 (등가 Kt): [Nm/A] = 모터Kt * 감속비 * 효율
volatile float Kt_equiv_RH = 1.8f;
volatile float Kt_equiv_LH = 1.8f;

// 안전 및 스무딩
#define IA_MAX     6.0f    // [A]
#define DI_MAX    2.0f    // [A/ms] 전류 slew 제한
#define ALPHA_GC  0.20f   // 1차 LPF 계수(0.1~0.3)
#define I_STICTION 0.20f  // 정마찰 보정[A](0.1~0.3)
#define I_DEAD     0.05f  // 데드존 임계[A]

typedef struct {
    float lpf_y;      // LPF 상태
    float prev_i;     // Slew 기준
    uint8_t entered;  // 모드 최초 진입 플래그
} UDState;
static UDState ud_RH={0}, ud_LH={0};

static inline float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }
static inline float lpf1(float x, float y, float a){ return y + a*(x - y); }
static inline float slew(float x_des, float x_prev, float di_max, float dt_ms){
    float dmax = di_max * dt_ms, dx = x_des - x_prev;
    if (dx >  dmax) x_des = x_prev + dmax;
    if (dx < -dmax) x_des = x_prev - dmax;
    return x_des;
}
static inline float sgn(float x){ return (x>0)-(x<0); }


/* --- ASSIST_REDUCING_SHOCK_MODE --- */


/* --- UNDERWATER_RESISTANCE_MODE --- */



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
   TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_OFF,      NULL,            StateOff_Run,       NULL,                true);
   TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_STANDBY,  NULL,            StateStandby_Run,   NULL,                false);
   TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,      StateEnable_Run,    StateEnable_Ext,    false);
   TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_ERROR,    NULL,            StateError_Run,       NULL,             false);

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
      algorithmCtrlTimeElap = ((4294967295 - STUDENTcodeStartTick) + STUDENTcodeEndTick) / 480;   // in microsecond (Roll-over)
   }
   else {
      algorithmCtrlTimeElap = (DWT->CYCCNT - STUDENTcodeStartTick) / 480;                     // in microsecond
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
      if (controlMode == POSITION_CTRL) {   // 1 - Position Control
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
      } else if (controlMode == GRAVITY_COMPENSATION) {   // 2 - Gravity Compensation
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

      } else if (controlMode == USER_DEFINED_CTRL) {   // 6 - User Defined Control
        
        switch ((uint8_t)user_control_mode) {
          case PID_CONTROL_MODE:
          {
            /* ------------------------------------------ PID_CONTROL_MODE ------------------------------------------------ */
            // 1) 모드 진입시 상태 초기화 (한 번만)
            if (user_first) {
              sum = 0.0f;
              Lerror_prev = 0.0f;
              d_meas_lp = 0.0f;
              user_first = false;
            }

            // 2) 측정값 & 오차(단위: 도)
            Ldeg   = robotDataObj_LH.thighTheta_act;      // [deg]
            Lerror = 30.0f - Ldeg;                        // 목표 30도

            // 3) D항: "측정치 기반 미분(= -dθ/dt)" 로 derivative kick 방지
            //    가. 자이로가 신뢰된다면 그 값을 그대로 사용 (단위 맞추기)
            //       - robotDataObj_LH.gyrZ 가 [deg/s] 라고 가정
            float dtheta_meas = robotDataObj_LH.gyrZ;     // [deg/s]

            //    나. 저역통과(1차 IIR)로 노이즈 완화
            d_meas_lp = DERIV_ALPHA * d_meas_lp + (1.0f - DERIV_ALPHA) * dtheta_meas;

            //    다. D항은 측정 미분의 음수 (오차 미분 대신 측정치 미분 ⇒ kick 억제)
            float Dterm = -Kd * d_meas_lp;

            // (만약 자이로가 없다면) 오차 차분으로 대체:
            // float derr = (Lerror - Lerror_prev) / DT;
            // d_meas_lp  = DERIV_ALPHA * d_meas_lp + (1.0f - DERIV_ALPHA) * derr;
            // float Dterm = Kd * d_meas_lp;  // 이때는 부호 반대가 아님(오차 미분이므로)

            // 4) P, I항
            float Pterm = Kp * Lerror;

            // 적분은 누적 전에 **안티윈드업** 처리:
            // (a) 기본: 포화 상태에서 '출력과 같은 부호의 오차'일 때 적분 중지
            // (출력 계산 전에 미리 예측 포화 검사하려면 아래 u_unsat로 판단)
            float Iterm = sum * Ki;

            // 5) 일단 비포화 출력 계산
            float u_unsat = Pterm + Iterm + Dterm;

            // 6) 포화 적용
            float u = u_unsat;
            if (u > U_MAX) u = U_MAX;
            if (u < -U_MAX) u = -U_MAX;

            // 7) 안티윈드업(기본형: 포화+동부호 시 적분 중지, 반대부호면 풀어줌)
            bool is_saturated = (u != u_unsat);
            if (is_saturated) {
              // 출력과 오차가 같은 부호이면 포화 유지 방향으로 더 밀지 않도록 적분 정지
              if (!((u > 0.0f && Lerror < 0.0f) || (u < 0.0f && Lerror > 0.0f))) {
                  // 적분하지 않음
              } else {
                  sum += Lerror * dT;   // 반대부호면 적분 허용(복원)
              }
            } else {
              sum += Lerror * dT;       // 정상적으로 적분
            }

            // 적분 한계(하드 클램프)
            if (sum > I_MAX) sum = I_MAX;
            if (sum < I_MIN) sum = I_MIN;

            // 8) 상태 업데이트 & 출력
            Lerror_prev = Lerror;
            UserDefinedCtrl_LH.control_input = u;  // 이후 공통 Saturation에서 한 번 더 제한됨
          } break;

          case GRAVITY_COMP_MODE:
          {
            /* ------------------------------------------ GRAVITY_COMP_MODE ------------------------------------------------ */
            // 왼쪽 허벅지 각도 (deg 단위라면 rad로 변환)
            float theta_deg = robotDataObj_LH.thighTheta_act;
            float theta_rad = theta_deg * 3.141592f / 180.0f;

            // 중력보상 토크 계산 (단순화된 모델)
            float torque = K * sinf(theta_rad) * assist_level;

            // 실제로는 토크 대신 전류 제어 입력으로 보낼 수도 있음
            UserDefinedCtrl_LH.control_input = torque;

            // 오른쪽 다리에도 적용하려면 동일하게 복사
            float thetaR_deg = robotDataObj_RH.thighTheta_act;
            float thetaR_rad = thetaR_deg * 3.141592f / 180.0f;
            UserDefinedCtrl_RH.control_input = K * sinf(thetaR_rad);
          } break;

          case ASSIST_REDUCING_SHOCK_MODE:
          {
            /* ------------------------------------------ ASSIST_REDUCING_SHOCK_MODE ------------------------------------------------ */
            
            // 각도 차이 계산
            float angle_diff = robotDataObj_RH.thighTheta_act - robotDataObj_LH.thighTheta_act;
            
            // 중력보상
            float grav_RH = K * sinf(robotDataObj_RH.thighTheta_act * 3.141592f / 180.0f);
            float grav_LH = K * sinf(robotDataObj_LH.thighTheta_act * 3.141592f / 180.0f);
            
            // 착지 직전 위로 보조 (오른쪽)
            if (angle_diff > 20.0f && robotDataObj_RH.gyrZ > 10.0f)
                UserDefinedCtrl_RH.control_input = grav_RH - 0.3f * robotDataObj_RH.gyrZ;
            else
                UserDefinedCtrl_RH.control_input = grav_RH;
            
            // 착지 직전 위로 보조 (왼쪽)
            if (angle_diff < -20.0f && robotDataObj_LH.gyrZ > 10.0f)
                UserDefinedCtrl_LH.control_input = grav_LH - 0.3f * robotDataObj_LH.gyrZ;
            else
                UserDefinedCtrl_LH.control_input = grav_LH;
            
            // 다른 제어 입력 초기화
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
          } break;

          case UNDERWATER_RESISTANCE_MODE:
          {
            /* ------------------------------------------ UNDERWATER_RESISTANCE_MODE ------------------------------------------------ */
            
            // 물의 저항력 = 속도에 비례하는 댐핑 (Damping = -c × v)
            float resistance_RH = -0.15f * robotDataObj_RH.gyrZ;  // 저항 계수 0.15
            float resistance_LH = -0.15f * robotDataObj_LH.gyrZ;
            
            // 부력 효과 = 중력의 50% 감소 (물속에서 가벼워지는 느낌)
            float buoyancy_RH = 0.5f * K * sinf(robotDataObj_RH.thighTheta_act * 3.141592f / 180.0f);
            float buoyancy_LH = 0.5f * K * sinf(robotDataObj_LH.thighTheta_act * 3.141592f / 180.0f);
            
            // 최종 출력 = 부력 + 저항력
            UserDefinedCtrl_RH.control_input = buoyancy_RH + resistance_RH;
            UserDefinedCtrl_LH.control_input = buoyancy_LH + resistance_LH;
            
            // 다른 제어 입력 초기화
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
          } break;
        }

      } else {
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
      *triggerSignal = false;         // Reset

      pvectorObj->yd_f = robotDataObj->position_act * M_PI / 180.0f;

      if (*MM_ID >= 0 && *MM_ID < 40) {
         *pvectorObj = MotionMap_File->MS[*MM_ID].MD[isLeft].p_vector_decoder;      // Assign
         // *MM_ID = 99;      // Reset
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
      LeftHipFlexionTorque = 2.0f;   // 3Nm, saturation
      LeftHipExtensionTorque = 2.0f;   // 3Nm
   } else {
      RightHipFlexionTorque = 2.0f;   // 3Nm
      RightHipExtensionTorque = 2.0f;   // 3Nm
   }
}

static void FvectorTrigger(F_Vector_Decoder* fvectorObj, MotionMapFileInfo* MotionMap_File, 
                     RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft)
{
   if (*triggerSignal == true) {
      *triggerSignal = false;         // Reset

      if (*MM_ID >= 0 && *MM_ID < 40) {
         *fvectorObj = MotionMap_File->MS[*MM_ID].MD[isLeft].f_vector_decoder;      // Assign
         // *MM_ID = 99;      // Reset
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
         //    if (isLeft) f_vector_input_LH = 0.0f;
         //    else f_vector_input_RH = 0.0f;
         //    fvectorObj->f_buffer[i].mode_idx = 0;
         //    fvectorObj->f_buffer[i].tau_max = 0;
         //    fvectorObj->f_buffer[i].delay = 0;
         //    fvectorObj->f_buffer[i].u = 0;
         //    fvectorObj->f_buffer[i].u_old1 = 0;
         //    fvectorObj->f_buffer[i].u_old2 = 0;
         //    fvectorObj->f_buffer[i].tau = 0;
         //    fvectorObj->f_buffer[i].tau_old1 = 0;
         //    fvectorObj->f_buffer[i].tau_old2 = 0;
         //    fvectorObj->f_buffer[i].t_end = 0;
         //    fvectorObj->f_buffer[i].time_stamp = 0;
         //    fvectorObj->f_buffer[i].is_full = 0;
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

      if (impedanceCtrl->i >= impedanceCtrl->duration)   {
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
   if (impedanceCtrl->e > 0)   {t_abs_e = +impedanceCtrl->e; t_sign_e = +1; }
   else                  {t_abs_e = -impedanceCtrl->e; t_sign_e = -1; }

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
