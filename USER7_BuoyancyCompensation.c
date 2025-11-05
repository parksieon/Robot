/*------------------------------------ (7) USER7 : Impact Reduction Control ------------------------------------*/
else if (ControlType == USER7_CTRL)
{
    /* (1) 변수 선언 */
    static float prev_fsr_L1 = 0.0f, prev_fsr_R1 = 0.0f;
    float fsr_diff_L = fsr1_L1 - prev_fsr_L1;
    float fsr_diff_R = fsr1_R1 - prev_fsr_R1;
    prev_fsr_L1 = fsr1_L1;
    prev_fsr_R1 = fsr1_R1;

    /* (2) 착지 감지 */
    bool isLanding_L = (fsr_diff_L > 500.0f);
    bool isLanding_R = (fsr_diff_R > 500.0f);

    /* (3) damping 변수 */
    const float base_damping = 0.5f;
    const float landing_damping = 3.0f;

    static float damping_L = base_damping;
    static float damping_R = base_damping;

    if (isLanding_L)
        damping_L = landing_damping;
    else
        damping_L = 0.95f * damping_L + 0.05f * base_damping; // 점진 복귀

    if (isLanding_R)
        damping_R = landing_damping;
    else
        damping_R = 0.95f * damping_R + 0.05f * base_damping;

    /* (4) 부력보상 항 추가 */
    // 예: Ksin(θ) 형태의 중력+부력 보상
    float gravity_comp_L = MASS_LEG_L * 9.81f * sin(robotDataObj_LH.theta);
    float gravity_comp_R = MASS_LEG_R * 9.81f * sin(robotDataObj_RH.theta);
    float buoyancy_coeff = 0.6f; // 부력 비율 (60% 정도로 감소)

    float buoyancy_comp_L = buoyancy_coeff * gravity_comp_L;
    float buoyancy_comp_R = buoyancy_coeff * gravity_comp_R;

    /* (5) 최종 제어 입력 (감쇠 + 부력 보상 포함) */
    UserDefinedCtrl_LH.control_input =
        -damping_L * robotDataObj_LH.gyrZ + buoyancy_comp_L;
    UserDefinedCtrl_RH.control_input =
        -damping_R * robotDataObj_RH.gyrZ + buoyancy_comp_R;

    /* (6) 출력 */
    UserDefinedCtrl_LH.flag = true;
    UserDefinedCtrl_RH.flag = true;
}
