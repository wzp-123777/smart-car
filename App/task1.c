#include "task.h"
#include "state_machine.h"

#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "openmv.h"
#include "infrared.h"
#include "syn6658.h"
#include "pid.h"

extern StateMachine_TypeDef g_state_machine;
extern PID_TypeDef g_pid_left;
extern PID_TypeDef g_pid_right;
extern MPU6050_DataTypeDef g_mpu_data;
extern IR_DataTypeDef g_ir_data;
extern volatile int16_t g_encoder_left;
extern volatile int16_t g_encoder_right;
extern uint32_t HAL_GetTick(void);
extern void delay_ms(uint32_t);
extern void LineFollow_CalibrateGyroBias(void);
extern CarEvent_TypeDef DetectEvent(void);
extern void Debug_LogIR(CarEvent_TypeDef event);

static int pc13_led_timer = 0;

static void TurnOn_LED_PC13(void)
{
    static uint8_t init_done = 0;
    GPIO_InitTypeDef GPIO_InitStructure;

    if (init_done == 0U)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        init_done = 1U;
    }

    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    pc13_led_timer = 100;
}

static void Tick_LED_PC13(void)
{
    if (pc13_led_timer > 0)
    {
        pc13_led_timer--;
        if (pc13_led_timer == 0)
        {
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
        }
    }
}

static void Task_StopAndHold(void)
{
    g_pid_left.target = 0.0f;
    g_pid_right.target = 0.0f;
    Motor_Stop();

    while (1)
    {
        g_pid_left.target = 0.0f;
        g_pid_right.target = 0.0f;
        Motor_Stop();
        Tick_LED_PC13();
        delay_ms(100);
    }
}

static void Task_LineFollowEnter(uint8_t task_id, uint8_t total_points)
{
    LineFollow_UseTaskProfile(task_id);
    SM_Init(&g_state_machine, total_points);
    MPU6050_ResetYaw(&g_mpu_data);
    LineFollow_CalibrateGyroBias();
    IR_ResetTracking();
    OpenMV_SendCmd(OPENMV_CMD_STOP);
    OpenMV_ResetResult();
    SM_Process(&g_state_machine, EVENT_START);
}

#define TASK2_DIAGONAL_ANGLE_OFFSET_AC        41.0f
#define TASK2_DIAGONAL_ANGLE_OFFSET_BD        33.0f
#define TASK2_DIAGONAL_MIN_TRAVEL_PULSE       1200
#define TASK2_DIAGONAL_ASSIST_PULSE           0
#define TASK2_DIAGONAL_FINISH_PULSE           7800
#define TASK2_DIAGONAL_FINISH_PULSE_BD        5000
#define TASK2_DIAGONAL_PULSE_LIMIT            9300
#define TASK2_BD_IR_PROTECT_PULSE             1000
#define TASK2_DIAGONAL_BASE_SPEED             22.0f
#define TASK2_DIAGONAL_MAX_CORRECTION         10.0f
#define TASK2_DIAGONAL_IR_GAIN                 0.35f
#define TASK2_DIAGONAL_IR_MAX                  8.0f
#define TASK2_ALIGN_TOLERANCE_DEG              8.0f
#define TASK2_ALIGN_TURN_MAX                  35.0f
#define TASK2_ALIGN_TURN_MIN                   8.0f
#define TASK2_ALIGN_STABLE_COUNT               6U
#define TASK2_ALIGN_TIMEOUT_LOOPS           400U
#define TASK2_STRAIGHT_CAPTURE_TOLERANCE_DEG  10.0f
#define TASK2_STRAIGHT_CAPTURE_STABLE_COUNT    5U
#define TASK2_C_RETURN_LEFT_DELTA            25.0f
#define TASK2_IR_ALIGN_TURN_MAX              25.0f
#define TASK2_IR_ALIGN_TURN_MIN               6.0f
#define TASK2_ARC_FINISH_YAW_DELTA           175.0f
#define TASK2_CB_ARC_FINISH_YAW_DELTA        167.0f
#define TASK2_CB_ALIGN_TOLERANCE_DEG           2.0f
#define TASK2_A_TO_D_YAW_DELTA              170.0f
#define TASK2_D_TO_A_YAW_DELTA              177.0f
#define TASK2_DA_ARC_TARGET_YAW_DELTA       178.0f
#define TASK2_DA_ARC_TOLERANCE_DEG            2.0f
#define TASK_POINT_REPORT_DELAY_MS          400U
#define TASK_TARGET_DETECT_MS             3000U

static float Task2_AbsFloat(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

static float Task2_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static float Task2_NormalizeDelta(float delta)
{
    while (delta > 180.0f)
    {
        delta -= 360.0f;
    }

    while (delta < -180.0f)
    {
        delta += 360.0f;
    }

    return delta;
}

static int32_t Task2_GetPulseStep(void)
{
    int32_t left = g_encoder_left;
    int32_t right = g_encoder_right;

    if (left < 0)
    {
        left = -left;
    }

    if (right < 0)
    {
        right = -right;
    }

    return (left + right) / 2;
}

static uint8_t Task2_LineDetected(void)
{
    return (g_ir_data.raw_byte != 0U) ? 1U : 0U;
}

static uint8_t Task2_MidSensorsChanged(uint8_t initial_raw_byte)
{
    return (((g_ir_data.raw_byte ^ initial_raw_byte) & 0x0EU) != 0U) ? 1U : 0U;
}

static void Task2_StopMotion(uint32_t settle_ms)
{
    g_pid_left.target = 0.0f;
    g_pid_right.target = 0.0f;
    PID_Reset(&g_pid_left);
    PID_Reset(&g_pid_right);
    Motor_Stop();

    if (settle_ms > 0U)
    {
        delay_ms(settle_ms);
    }
}

static void Task2_SetManualTargets(float left_target, float right_target)
{
    g_pid_left.target = left_target;
    g_pid_right.target = right_target;
}

static void Task2_ManualServiceLoop(void)
{
    MPU6050_ReadAll(&g_mpu_data);
    Tick_LED_PC13();
    delay_ms(10);
}

static void Task2_LineFollowServiceLoop(void)
{
    MPU6050_ReadAll(&g_mpu_data);
    Debug_LogIR(EVENT_NONE);
    Tick_LED_PC13();
    delay_ms(10);
}

static void Task2_AlignToHeadingEx(float target_heading, float tolerance_deg)
{
    uint32_t stable_count = 0U;
    uint32_t loop_count = 0U;

    g_state_machine.current_state = STATE_IDLE;

    while (loop_count < TASK2_ALIGN_TIMEOUT_LOOPS)
    {
        float yaw_error;
        float turn_cmd;

        Task2_ManualServiceLoop();
        yaw_error = Task2_NormalizeDelta(target_heading - g_mpu_data.yaw);

        if (Task2_AbsFloat(yaw_error) <= tolerance_deg)
        {
            stable_count++;
            Task2_SetManualTargets(0.0f, 0.0f);

            if (stable_count >= TASK2_ALIGN_STABLE_COUNT)
            {
                break;
            }
        }
        else
        {
            stable_count = 0U;
            turn_cmd = yaw_error * 1.0f;

            if (turn_cmd > 0.0f)
            {
                turn_cmd = Task2_ClampFloat(turn_cmd, TASK2_ALIGN_TURN_MIN, TASK2_ALIGN_TURN_MAX);
            }
            else
            {
                turn_cmd = Task2_ClampFloat(turn_cmd, -TASK2_ALIGN_TURN_MAX, -TASK2_ALIGN_TURN_MIN);
            }

            Task2_SetManualTargets(-turn_cmd, turn_cmd);
        }

        loop_count++;
    }

    g_state_machine.current_state = STATE_IDLE;
    Task2_StopMotion(80U);
}

static void Task2_AlignToHeading(float target_heading)
{
    Task2_AlignToHeadingEx(target_heading, TASK2_ALIGN_TOLERANCE_DEG);
}

static void Task2_TurnRelative(float delta_deg, float tolerance_deg)
{
    float target_heading = Task2_NormalizeDelta(g_mpu_data.yaw + delta_deg);
    Task2_AlignToHeadingEx(target_heading, tolerance_deg);
}

static void Task2_CaptureTrack(void)
{
    uint32_t stable_count = 0U;
    uint32_t loop_count = 0U;
    int8_t last_position = 0;

    g_state_machine.current_state = STATE_IDLE;

    while (loop_count < TASK2_ALIGN_TIMEOUT_LOOPS)
    {
        float turn_cmd;

        Task2_ManualServiceLoop();

        if (Task2_LineDetected() != 0U)
        {
            last_position = g_ir_data.position;
        }

        if (((g_ir_data.sensor[2] != 0U) &&
              (Task2_AbsFloat((float)g_ir_data.position) <= 20.0f)) ||
            (g_ir_data.all_black != 0U))
        {
            stable_count++;
            if (stable_count >= TASK2_STRAIGHT_CAPTURE_STABLE_COUNT)
            {
                break;
            }
        }
        else
        {
            stable_count = 0U;

            if (Task2_LineDetected() == 0U)
            {
                if (last_position <= 0)
                {
                    turn_cmd = TASK2_IR_ALIGN_TURN_MIN;
                }
                else
                {
                    turn_cmd = -TASK2_IR_ALIGN_TURN_MIN;
                }
            }
            else
            {
                turn_cmd = -0.5f * (float)g_ir_data.position;

                if (turn_cmd > 0.0f)
                {
                    turn_cmd = Task2_ClampFloat(turn_cmd,
                                                TASK2_IR_ALIGN_TURN_MIN,
                                                TASK2_IR_ALIGN_TURN_MAX);
                }
                else
                {
                    turn_cmd = Task2_ClampFloat(turn_cmd,
                                                -TASK2_IR_ALIGN_TURN_MAX,
                                                -TASK2_IR_ALIGN_TURN_MIN);
                }
            }

            Task2_SetManualTargets(-turn_cmd, turn_cmd);
        }

        loop_count++;
    }

    g_state_machine.current_state = STATE_IDLE;
    Task2_StopMotion(80U);
}

static void Task2_ReachPointFromDiagonal(float diagonal_heading, float straight_heading, int32_t finish_pulse)
{
    int32_t total_pulse = 0;
    uint8_t initial_raw_byte = g_ir_data.raw_byte;

    g_state_machine.current_state = STATE_IDLE;

    while (1)
    {
        float yaw_error;
        float correction;
        float left_target;
        float right_target;

        Task2_ManualServiceLoop();
        total_pulse += Task2_GetPulseStep();

        yaw_error = Task2_NormalizeDelta(diagonal_heading - g_mpu_data.yaw);
        correction = (yaw_error * 1.20f) - (g_mpu_data.gyro_z_dps * 0.15f);
        correction = Task2_ClampFloat(correction,
                                      -TASK2_DIAGONAL_MAX_CORRECTION,
                                      TASK2_DIAGONAL_MAX_CORRECTION);

        left_target = TASK2_DIAGONAL_BASE_SPEED - correction;
        right_target = TASK2_DIAGONAL_BASE_SPEED + correction;
        Task2_SetManualTargets(left_target, right_target);

        if (Task2_MidSensorsChanged(initial_raw_byte) != 0U)
        {
            break;
        }

        if (total_pulse >= finish_pulse)
        {
            break;
        }

        if (total_pulse >= TASK2_DIAGONAL_PULSE_LIMIT)
        {
            break;
        }
    }

    (void)straight_heading;
    Task2_StopMotion(60U);
}

static void Task2_ReachPointFromDiagonal_BD(float diagonal_heading, float straight_heading, int32_t finish_pulse)
{
    int32_t total_pulse = 0;
    uint8_t initial_raw_byte = g_ir_data.raw_byte;
    uint8_t ir_changed = 0U;

    g_state_machine.current_state = STATE_IDLE;

    while (1)
    {
        float yaw_error;
        float correction;
        float left_target;
        float right_target;

        Task2_ManualServiceLoop();
        total_pulse += Task2_GetPulseStep();

        yaw_error = Task2_NormalizeDelta(diagonal_heading - g_mpu_data.yaw);
        correction = (yaw_error * 0.60f) - (g_mpu_data.gyro_z_dps * 0.05f);
        correction = Task2_ClampFloat(correction,
                                      -TASK2_DIAGONAL_MAX_CORRECTION,
                                      TASK2_DIAGONAL_MAX_CORRECTION);

        left_target = TASK2_DIAGONAL_BASE_SPEED - correction;
        right_target = TASK2_DIAGONAL_BASE_SPEED + correction;
        Task2_SetManualTargets(left_target, right_target);

        if (Task2_MidSensorsChanged(initial_raw_byte) != 0U)
        {
            ir_changed = 1U;
            break;
        }

        if (total_pulse >= finish_pulse)
        {
            break;
        }

        if (total_pulse >= TASK2_DIAGONAL_PULSE_LIMIT)
        {
            break;
        }
    }

    Task2_StopMotion(0U);
    if (ir_changed != 0U)
    {
        Task2_AlignToHeadingEx(straight_heading, TASK2_CB_ALIGN_TOLERANCE_DEG);
    }
}
static void Task2_FollowArcUntilYawWindow(float target_delta_deg, float tolerance_deg)
{
    float start_yaw;
    uint8_t stable_count = 0U;

    g_state_machine.current_state = STATE_LINE_FOLLOW;
    start_yaw = g_mpu_data.yaw;

    while (1)
    {
        float yaw_delta;
        float yaw_error;

        Task2_LineFollowServiceLoop();
        yaw_delta = Task2_AbsFloat(Task2_NormalizeDelta(g_mpu_data.yaw - start_yaw));
        yaw_error = Task2_AbsFloat(yaw_delta - target_delta_deg);

        if (yaw_error <= tolerance_deg)
        {
            stable_count++;
            if (stable_count >= TASK2_STRAIGHT_CAPTURE_STABLE_COUNT)
            {
                break;
            }
        }
        else
        {
            stable_count = 0U;
        }
    }

    g_state_machine.current_state = STATE_IDLE;
    Task2_StopMotion(80U);
}

static void Task2_FollowUntilAbsoluteYaw(float target_yaw, float tolerance_deg)
{
    uint8_t stable_count = 0U;

    g_state_machine.current_state = STATE_LINE_FOLLOW;

    while (1)
    {
        float yaw_error;

        Task2_LineFollowServiceLoop();
        yaw_error = Task2_NormalizeDelta(target_yaw - g_mpu_data.yaw);

        if (Task2_AbsFloat(yaw_error) <= tolerance_deg)
        {
            stable_count++;
            if (stable_count >= TASK2_STRAIGHT_CAPTURE_STABLE_COUNT)
            {
                break;
            }
        }
        else
        {
            stable_count = 0U;
        }
    }

    g_state_machine.current_state = STATE_IDLE;
    Task2_StopMotion(80U);
}

typedef enum
{
    TASK2_FLOW_BOOT_WAIT = 0,
    TASK2_FLOW_PRESTART_STOP,
    TASK2_FLOW_ALIGN_AC,
    TASK2_FLOW_DIAG_AC,
    TASK2_FLOW_TURN_LEFT_C,
    TASK2_FLOW_CAPTURE_C,
    TASK2_FLOW_REPORT_C,
    TASK2_FLOW_FOLLOW_TO_B,
    TASK2_FLOW_REPORT_B,
    TASK2_FLOW_ALIGN_180_PRE_BD,
    TASK2_FLOW_ALIGN_BD,
    TASK2_FLOW_DIAG_BD,
    TASK2_FLOW_FOLLOW_TO_D,
    TASK2_FLOW_ALIGN_D,
    TASK2_FLOW_REPORT_D,
    TASK2_FLOW_FOLLOW_TO_A,
    TASK2_FLOW_DELAY_A_REPORT,
    TASK2_FLOW_REPORT_A,
    TASK2_FLOW_HOLD
} Task2FlowState_TypeDef;

typedef struct
{
    Task2FlowState_TypeDef state;
    uint32_t state_tick;
    uint32_t last_service_tick;
    uint32_t stable_count;
    int32_t pulse_accum;
    uint8_t initial_raw_byte;
    int8_t last_position;
    float target_heading;
    float a_yaw;
    float start_yaw;
} Task2FlowContext_TypeDef;

static uint8_t Task_HasReportedObject(uint8_t reported_mask, uint8_t object_id);

static void Task2_StopNow(void)
{
    g_pid_left.target = 0.0f;
    g_pid_right.target = 0.0f;
    PID_Reset(&g_pid_left);
    PID_Reset(&g_pid_right);
    Motor_Stop();
}

static uint8_t Task_DetectPointObject(uint8_t detect_cmd, uint8_t reported_mask)
{
    uint32_t start_tick;
    uint8_t object_id = OBJ_NONE;

    g_state_machine.current_state = STATE_STOP_AND_DETECT;
    Task2_StopNow();
    OpenMV_ResetResult();
    OpenMV_SendCmd(detect_cmd);
    start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < TASK_TARGET_DETECT_MS)
    {
        OpenMV_DataTypeDef mv;

        Task2_StopNow();

        if (OpenMV_HasNewData() != 0U)
        {
            mv = OpenMV_GetResult();

            if ((object_id == OBJ_NONE) &&
                (mv.is_valid != 0U) &&
                (mv.object_id >= OBJ_LIGHTER) &&
                (mv.object_id <= OBJ_HAMMER) &&
                (Task_HasReportedObject(reported_mask, mv.object_id) == 0U))
            {
                object_id = mv.object_id;
                TurnOn_LED_PC13();
            }

            OpenMV_ClearNewFlag();
        }

        Tick_LED_PC13();
        delay_ms(10);
    }

    OpenMV_SendCmd(OPENMV_CMD_STOP);
    OpenMV_ClearNewFlag();
    Task2_StopNow();
    g_state_machine.current_state = STATE_IDLE;

    return object_id;
}

static void Task2_EnterFlowState(Task2FlowContext_TypeDef *ctx,
                                 Task2FlowState_TypeDef next_state)
{
    ctx->state = next_state;
    ctx->state_tick = HAL_GetTick();
    ctx->stable_count = 0U;
}

static uint8_t Task2_ServiceAlignStep(Task2FlowContext_TypeDef *ctx,
                                      float target_heading,
                                      float tolerance_deg)
{
    float yaw_error;
    float turn_cmd;

    g_state_machine.current_state = STATE_IDLE;
    yaw_error = Task2_NormalizeDelta(target_heading - g_mpu_data.yaw);

    if (Task2_AbsFloat(yaw_error) <= tolerance_deg)
    {
        ctx->stable_count++;
        Task2_SetManualTargets(0.0f, 0.0f);
        if (ctx->stable_count >= TASK2_ALIGN_STABLE_COUNT)
        {
            return 1U;
        }
    }
    else
    {
        ctx->stable_count = 0U;
        turn_cmd = yaw_error * 1.0f;

        if (turn_cmd > 0.0f)
        {
            turn_cmd = Task2_ClampFloat(turn_cmd,
                                        TASK2_ALIGN_TURN_MIN,
                                        TASK2_ALIGN_TURN_MAX);
        }
        else
        {
            turn_cmd = Task2_ClampFloat(turn_cmd,
                                        -TASK2_ALIGN_TURN_MAX,
                                        -TASK2_ALIGN_TURN_MIN);
        }

        Task2_SetManualTargets(-turn_cmd, turn_cmd);
    }

    return 0U;
}

static uint8_t Task2_ServiceDiagonalStep(Task2FlowContext_TypeDef *ctx,
                                         float diagonal_heading,
                                         int32_t finish_pulse,
                                         float heading_gain,
                                         float gyro_gain,
                                         uint8_t stop_on_mid_change)
{
    float yaw_error;
    float correction;
    float left_target;
    float right_target;

    g_state_machine.current_state = STATE_IDLE;
    ctx->pulse_accum += Task2_GetPulseStep();

    yaw_error = Task2_NormalizeDelta(diagonal_heading - g_mpu_data.yaw);
    correction = (yaw_error * heading_gain) - (g_mpu_data.gyro_z_dps * gyro_gain);
    correction = Task2_ClampFloat(correction,
                                  -TASK2_DIAGONAL_MAX_CORRECTION,
                                  TASK2_DIAGONAL_MAX_CORRECTION);

    left_target = TASK2_DIAGONAL_BASE_SPEED - correction;
    right_target = TASK2_DIAGONAL_BASE_SPEED + correction;
    Task2_SetManualTargets(left_target, right_target);

    if ((stop_on_mid_change != 0U) &&
        (Task2_MidSensorsChanged(ctx->initial_raw_byte) != 0U))
    {
        return 1U;
    }

    if ((ctx->pulse_accum >= finish_pulse) ||
        (ctx->pulse_accum >= TASK2_DIAGONAL_PULSE_LIMIT))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t Task2_ServiceCaptureTrackStep(Task2FlowContext_TypeDef *ctx)
{
    float turn_cmd;

    g_state_machine.current_state = STATE_IDLE;

    if (Task2_LineDetected() != 0U)
    {
        ctx->last_position = g_ir_data.position;
    }

    if (((g_ir_data.sensor[2] != 0U) &&
         (Task2_AbsFloat((float)g_ir_data.position) <= 20.0f)) ||
        (g_ir_data.all_black != 0U))
    {
        ctx->stable_count++;
        Task2_SetManualTargets(0.0f, 0.0f);
        if (ctx->stable_count >= TASK2_STRAIGHT_CAPTURE_STABLE_COUNT)
        {
            return 1U;
        }
    }
    else
    {
        ctx->stable_count = 0U;

        if (Task2_LineDetected() == 0U)
        {
            if (ctx->last_position <= 0)
            {
                turn_cmd = TASK2_IR_ALIGN_TURN_MIN;
            }
            else
            {
                turn_cmd = -TASK2_IR_ALIGN_TURN_MIN;
            }
        }
        else
        {
            turn_cmd = -0.5f * (float)g_ir_data.position;

            if (turn_cmd > 0.0f)
            {
                turn_cmd = Task2_ClampFloat(turn_cmd,
                                            TASK2_IR_ALIGN_TURN_MIN,
                                            TASK2_IR_ALIGN_TURN_MAX);
            }
            else
            {
                turn_cmd = Task2_ClampFloat(turn_cmd,
                                            -TASK2_IR_ALIGN_TURN_MAX,
                                            -TASK2_IR_ALIGN_TURN_MIN);
            }
        }

        Task2_SetManualTargets(-turn_cmd, turn_cmd);
    }

    return 0U;
}

static uint8_t Task2_ServiceFollowAbsoluteYawStep(Task2FlowContext_TypeDef *ctx,
                                                  float target_yaw,
                                                  float tolerance_deg)
{
    float yaw_error;

    g_state_machine.current_state = STATE_LINE_FOLLOW;
    Debug_LogIR(EVENT_NONE);
    yaw_error = Task2_NormalizeDelta(target_yaw - g_mpu_data.yaw);

    if (Task2_AbsFloat(yaw_error) <= tolerance_deg)
    {
        ctx->stable_count++;
        if (ctx->stable_count >= TASK2_STRAIGHT_CAPTURE_STABLE_COUNT)
        {
            return 1U;
        }
    }
    else
    {
        ctx->stable_count = 0U;
    }

    return 0U;
}

static uint8_t Task2_ServiceFollowArcStep(Task2FlowContext_TypeDef *ctx,
                                          float target_delta_deg,
                                          float tolerance_deg)
{
    float yaw_delta;
    float yaw_error;

    g_state_machine.current_state = STATE_LINE_FOLLOW;
    Debug_LogIR(EVENT_NONE);
    yaw_delta = Task2_AbsFloat(Task2_NormalizeDelta(g_mpu_data.yaw - ctx->start_yaw));
    yaw_error = Task2_AbsFloat(yaw_delta - target_delta_deg);

    if (yaw_error <= tolerance_deg)
    {
        ctx->stable_count++;
        if (ctx->stable_count >= TASK2_STRAIGHT_CAPTURE_STABLE_COUNT)
        {
            return 1U;
        }
    }
    else
    {
        ctx->stable_count = 0U;
    }

    return 0U;
}

static uint8_t Task2_ServiceFollowArcReachedStep(Task2FlowContext_TypeDef *ctx,
                                                 float threshold_delta_deg)
{
    float yaw_delta;

    g_state_machine.current_state = STATE_LINE_FOLLOW;
    Debug_LogIR(EVENT_NONE);
    ctx->pulse_accum += Task2_GetPulseStep();
    yaw_delta = Task2_AbsFloat(Task2_NormalizeDelta(g_mpu_data.yaw - ctx->start_yaw));

    if ((yaw_delta >= threshold_delta_deg) &&
        (ctx->pulse_accum >= TASK2_BD_IR_PROTECT_PULSE) &&
        (Task2_LineDetected() != 0U))
    {
        return 1U;
    }

    return 0U;
}
void Task1_Run(void)
{
    int32_t total_pulse = 0;
    uint8_t passed_B = 0;
    uint8_t passed_C = 0;
    uint8_t passed_D = 0;
    uint8_t is_yaw_cleared_at_C = 0;
    int delay_timer_C = 0;
    float yaw_at_C = 0.0f;
    float abs_yaw;
    float yaw_diff_from_C;
    CarEvent_TypeDef event;

    TurnOn_LED_PC13();
    delay_ms(500);
    Task_LineFollowEnter(1U, 3);

    while (1)
    {
        MPU6050_ReadAll(&g_mpu_data);

        total_pulse += g_encoder_left;

        abs_yaw = g_mpu_data.yaw;
        if (abs_yaw < 0.0f)
        {
            abs_yaw = -abs_yaw;
        }

        if ((passed_B == 0U) && (total_pulse >= 6800))
        {
            SYN6658_ReportStation('B');
            TurnOn_LED_PC13();
            passed_B = 1U;
        }

        if ((passed_C == 0U) && (abs_yaw >= 178.0f))
        {
            SYN6658_ReportStation('C');
            TurnOn_LED_PC13();
            passed_C = 1U;
            total_pulse = 0;
            delay_timer_C = 100;
        }

        if ((passed_C != 0U) && (is_yaw_cleared_at_C == 0U))
        {
            if (delay_timer_C > 0)
            {
                delay_timer_C--;
            }
            else
            {
                yaw_at_C = g_mpu_data.yaw;
                is_yaw_cleared_at_C = 1U;
            }
        }

        if ((passed_C != 0U) && (passed_D == 0U) && (total_pulse >= 6800))
        {
            SYN6658_ReportStation('D');
            TurnOn_LED_PC13();
            passed_D = 1U;
        }

        yaw_diff_from_C = g_mpu_data.yaw - yaw_at_C;
        if (yaw_diff_from_C < 0.0f)
        {
            yaw_diff_from_C = -yaw_diff_from_C;
        }

        if ((is_yaw_cleared_at_C != 0U) && (yaw_diff_from_C >= 176.0f))
        {
            SYN6658_ReportStation('A');
            TurnOn_LED_PC13();
            delay_ms(100);
            g_state_machine.current_state = STATE_IDLE;
            Task_StopAndHold();
        }

        event = DetectEvent();
        SM_Process(&g_state_machine, event);
        Debug_LogIR(event);
        Tick_LED_PC13();
        delay_ms(10);
    }
}

void Task2_Run(void)
{
    float first_diagonal_heading = -TASK2_DIAGONAL_ANGLE_OFFSET_AC;
    float second_diagonal_heading = 180.0f + TASK2_DIAGONAL_ANGLE_OFFSET_BD;
    Task2FlowContext_TypeDef ctx;

    ctx.state = TASK2_FLOW_BOOT_WAIT;
    ctx.state_tick = HAL_GetTick();
    ctx.last_service_tick = HAL_GetTick();
    ctx.stable_count = 0U;
    ctx.pulse_accum = 0;
    ctx.initial_raw_byte = 0U;
    ctx.last_position = 0;
    ctx.target_heading = 0.0f;
    ctx.a_yaw = 0.0f;
    ctx.start_yaw = 0.0f;

    TurnOn_LED_PC13();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if ((now - ctx.last_service_tick) < 10U)
        {
            continue;
        }
        ctx.last_service_tick = now;

        MPU6050_ReadAll(&g_mpu_data);
        Tick_LED_PC13();

        switch (ctx.state)
        {
        case TASK2_FLOW_BOOT_WAIT:
            if ((now - ctx.state_tick) >= 500U)
            {
                Task_LineFollowEnter(2U, 3U);
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_PRESTART_STOP);
            }
            break;

        case TASK2_FLOW_PRESTART_STOP:
            Task2_StopNow();
            if ((now - ctx.state_tick) >= 150U)
            {
                ctx.a_yaw = g_mpu_data.yaw;
                ctx.target_heading = first_diagonal_heading;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_ALIGN_AC);
            }
            break;

        case TASK2_FLOW_ALIGN_AC:
            if (Task2_ServiceAlignStep(&ctx,
                                       ctx.target_heading,
                                       TASK2_ALIGN_TOLERANCE_DEG) != 0U)
            {
                Task2_StopNow();
                ctx.pulse_accum = 0;
                ctx.initial_raw_byte = g_ir_data.raw_byte;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_DIAG_AC);
            }
            break;

        case TASK2_FLOW_DIAG_AC:
            if (Task2_ServiceDiagonalStep(&ctx,
                                          first_diagonal_heading,
                                          TASK2_DIAGONAL_FINISH_PULSE,
                                          1.20f,
                                          0.15f,
                                          1U) != 0U)
            {
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_TURN_LEFT_C);
                ctx.target_heading = Task2_NormalizeDelta(g_mpu_data.yaw + TASK2_C_RETURN_LEFT_DELTA);
            }
            break;

        case TASK2_FLOW_TURN_LEFT_C:
            if (Task2_ServiceAlignStep(&ctx,
                                       ctx.target_heading,
                                       TASK2_CB_ALIGN_TOLERANCE_DEG) != 0U)
            {
                Task2_StopNow();
                ctx.last_position = 0;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_CAPTURE_C);
            }
            break;

        case TASK2_FLOW_CAPTURE_C:
            if (Task2_ServiceCaptureTrackStep(&ctx) != 0U)
            {
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_REPORT_C);
            }
            break;

        case TASK2_FLOW_REPORT_C:
            SYN6658_ReportStation('C');
            TurnOn_LED_PC13();
            Task2_EnterFlowState(&ctx, TASK2_FLOW_FOLLOW_TO_B);
            break;

        case TASK2_FLOW_FOLLOW_TO_B:
            if (Task2_ServiceFollowAbsoluteYawStep(&ctx,
                                                   TASK2_CB_ARC_FINISH_YAW_DELTA,
                                                   TASK2_CB_ALIGN_TOLERANCE_DEG) != 0U)
            {
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_REPORT_B);
            }
            break;

        case TASK2_FLOW_REPORT_B:
            SYN6658_ReportStation('B');
            TurnOn_LED_PC13();
            ctx.target_heading = 180.0f;
            Task2_EnterFlowState(&ctx, TASK2_FLOW_ALIGN_180_PRE_BD);
            break;

        case TASK2_FLOW_ALIGN_180_PRE_BD:
            if (Task2_ServiceAlignStep(&ctx,
                                       ctx.target_heading,
                                       TASK2_CB_ALIGN_TOLERANCE_DEG) != 0U)
            {
                Task2_StopNow();
                ctx.target_heading = second_diagonal_heading;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_ALIGN_BD);
            }
            break;

        case TASK2_FLOW_ALIGN_BD:
            if (Task2_ServiceAlignStep(&ctx,
                                       ctx.target_heading,
                                       TASK2_ALIGN_TOLERANCE_DEG) != 0U)
            {
                Task2_StopNow();
                ctx.pulse_accum = 0;
                ctx.initial_raw_byte = g_ir_data.raw_byte;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_DIAG_BD);
            }
            break;

        case TASK2_FLOW_DIAG_BD:
            if (Task2_ServiceDiagonalStep(&ctx,
                                          second_diagonal_heading,
                                          TASK2_DIAGONAL_FINISH_PULSE_BD,
                                          0.60f,
                                          0.05f,
                                          1U) != 0U)
            {
                Task2_StopNow();
                ctx.pulse_accum = 0;
                ctx.last_position = 0;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_FOLLOW_TO_D);
            }
            break;

        case TASK2_FLOW_FOLLOW_TO_D:
            ctx.start_yaw = ctx.a_yaw;
            if (Task2_ServiceFollowArcReachedStep(&ctx,
                                                  TASK2_A_TO_D_YAW_DELTA) != 0U)
            {
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_REPORT_D);
            }
            break;

        case TASK2_FLOW_ALIGN_D:
            Task2_EnterFlowState(&ctx, TASK2_FLOW_REPORT_D);
            break;

        case TASK2_FLOW_REPORT_D:
            SYN6658_ReportStation('D');
            TurnOn_LED_PC13();
            ctx.start_yaw = g_mpu_data.yaw;
            Task2_EnterFlowState(&ctx, TASK2_FLOW_FOLLOW_TO_A);
            break;

        case TASK2_FLOW_FOLLOW_TO_A:
            if (Task2_ServiceFollowArcStep(&ctx,
                                           TASK2_D_TO_A_YAW_DELTA,
                                           TASK2_DA_ARC_TOLERANCE_DEG) != 0U)
            {
                Task2_EnterFlowState(&ctx, TASK2_FLOW_DELAY_A_REPORT);
            }
            break;

        case TASK2_FLOW_DELAY_A_REPORT:
            g_state_machine.current_state = STATE_LINE_FOLLOW;
            Debug_LogIR(EVENT_NONE);
            if ((now - ctx.state_tick) >= TASK_POINT_REPORT_DELAY_MS)
            {
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_REPORT_A);
            }
            break;

        case TASK2_FLOW_REPORT_A:
            SYN6658_ReportStation('A');
            TurnOn_LED_PC13();
            Task2_EnterFlowState(&ctx, TASK2_FLOW_HOLD);
            break;

        case TASK2_FLOW_HOLD:
        default:
            g_state_machine.current_state = STATE_IDLE;
            Task2_StopNow();
            break;
        }
    }
}
static uint8_t Task3_GetPointIndex(char point)
{
    switch (point)
    {
    case 'B':
        return 0U;
    case 'C':
        return 1U;
    case 'D':
        return 2U;
    default:
        return 3U;
    }
}

static uint8_t Task_ObjectSpeechBit(uint8_t object_id)
{
    switch (object_id)
    {
    case OBJ_LIGHTER:
        return 0x01U;
    case OBJ_SCISSORS:
        return 0x02U;
    case OBJ_HAMMER:
        return 0x04U;
    default:
        return 0U;
    }
}

static uint8_t Task_HasReportedObject(uint8_t reported_mask, uint8_t object_id)
{
    uint8_t object_bit = Task_ObjectSpeechBit(object_id);

    if (object_bit == 0U)
    {
        return 0U;
    }

    return (uint8_t)((reported_mask & object_bit) != 0U);
}

static void Task_MarkObjectReported(uint8_t *reported_mask, uint8_t object_id)
{
    uint8_t object_bit;

    if (reported_mask == 0)
    {
        return;
    }

    object_bit = Task_ObjectSpeechBit(object_id);
    *reported_mask = (uint8_t)(*reported_mask | object_bit);
}

static void Task3_SpeakPointObject(char point, uint8_t object_id)
{
    char buf[24];
    const char *suffix = 0;
    uint8_t i = 0U;
    uint8_t j = 0U;

    switch (object_id)
    {
    case OBJ_LIGHTER:
        suffix = "\xB7\xA2\xCF\xD6\xB4\xF2\xBB\xF0\xBB\xFA";
        break;
    case OBJ_SCISSORS:
        suffix = "\xB7\xA2\xCF\xD6\xBC\xF4\xB5\xB6";
        break;
    case OBJ_HAMMER:
        suffix = "\xB7\xA2\xCF\xD6\xB4\xB8\xD7\xD3";
        break;
    default:
        return;
    }

    buf[i++] = point;
    buf[i++] = (char)0xB5;
    buf[i++] = (char)0xE3;

    while ((suffix[j] != '\0') && (i < (sizeof(buf) - 1U)))
    {
        buf[i++] = suffix[j++];
    }

    buf[i] = '\0';
    SYN6658_Speak(buf);
}

static void Task3_ReportPointResult(char point, uint8_t *object_slot, uint8_t *reported_mask)
{
    uint8_t object_id = 0U;

    g_pid_left.target = 0.0f;
    g_pid_right.target = 0.0f;
    Motor_Stop();
    delay_ms(60);

    if (object_slot != 0)
    {
        object_id = *object_slot;
    }

    if ((object_id != 0U) && (Task_HasReportedObject(*reported_mask, object_id) == 0U))
    {
        Task3_SpeakPointObject(point, object_id);
        Task_MarkObjectReported(reported_mask, object_id);
    }
    else
    {
        SYN6658_ReportStation(point);
    }

    if (object_slot != 0)
    {
        *object_slot = 0U;
    }
}

static void Task4_RememberTargetObject(char target_point,
                                       uint8_t remembered_object[4],
                                       uint8_t reported_mask)
{
    OpenMV_DataTypeDef mv;
    uint8_t point_index;

    if (OpenMV_HasNewData() == 0U)
    {
        return;
    }

    mv = OpenMV_GetResult();
    point_index = Task3_GetPointIndex(target_point);

    if ((mv.is_valid != 0U) &&
        (mv.object_id >= OBJ_LIGHTER) &&
        (mv.object_id <= OBJ_HAMMER) &&
        (Task_HasReportedObject(reported_mask, mv.object_id) == 0U) &&
        (remembered_object[point_index] == 0U))
    {
        remembered_object[point_index] = mv.object_id;
        TurnOn_LED_PC13();
    }

    OpenMV_ClearNewFlag();
}

static void Task4_ManualServiceLoop(uint8_t remembered_object[4],
                                    char target_point,
                                    uint8_t reported_mask)
{
    Task2_ManualServiceLoop();
    Task4_RememberTargetObject(target_point, remembered_object, reported_mask);
}

static void Task4_LineFollowServiceLoop(uint8_t remembered_object[4],
                                        char target_point,
                                        uint8_t reported_mask)
{
    Task2_LineFollowServiceLoop();
    Task4_RememberTargetObject(target_point, remembered_object, reported_mask);
}

static void Task4_ReportPointResult(char point, uint8_t *object_slot, uint8_t *reported_mask)
{
    Task2_StopMotion(60U);

    if ((object_slot != 0) &&
        (*object_slot != 0U) &&
        (Task_HasReportedObject(*reported_mask, *object_slot) == 0U))
    {
        Task3_SpeakPointObject(point, *object_slot);
        Task_MarkObjectReported(reported_mask, *object_slot);
    }
    else
    {
        SYN6658_ReportStation(point);
    }

    if (object_slot != 0)
    {
        *object_slot = 0U;
    }
}

static void Task4_AlignToHeadingEx(float target_heading,
                                   float tolerance_deg,
                                   uint8_t remembered_object[4],
                                   char target_point,
                                   uint8_t reported_mask)
{
    uint32_t stable_count = 0U;
    uint32_t loop_count = 0U;

    g_state_machine.current_state = STATE_IDLE;

    while (loop_count < TASK2_ALIGN_TIMEOUT_LOOPS)
    {
        float yaw_error;
        float turn_cmd;

        Task4_ManualServiceLoop(remembered_object, target_point, reported_mask);
        yaw_error = Task2_NormalizeDelta(target_heading - g_mpu_data.yaw);

        if (Task2_AbsFloat(yaw_error) <= tolerance_deg)
        {
            stable_count++;
            Task2_SetManualTargets(0.0f, 0.0f);

            if (stable_count >= TASK2_ALIGN_STABLE_COUNT)
            {
                break;
            }
        }
        else
        {
            stable_count = 0U;
            turn_cmd = yaw_error * 1.5f;

            if (turn_cmd > 0.0f)
            {
                turn_cmd = Task2_ClampFloat(turn_cmd,
                                            TASK2_ALIGN_TURN_MIN,
                                            TASK2_ALIGN_TURN_MAX);
            }
            else
            {
                turn_cmd = Task2_ClampFloat(turn_cmd,
                                            -TASK2_ALIGN_TURN_MAX,
                                            -TASK2_ALIGN_TURN_MIN);
            }

            Task2_SetManualTargets(-turn_cmd, turn_cmd);
        }

        loop_count++;
    }

    g_state_machine.current_state = STATE_IDLE;
    Task2_StopMotion(80U);
}

static void Task4_AlignToHeading(float target_heading,
                                 uint8_t remembered_object[4],
                                 char target_point,
                                 uint8_t reported_mask)
{
    Task4_AlignToHeadingEx(target_heading,
                           TASK2_ALIGN_TOLERANCE_DEG,
                           remembered_object,
                           target_point,
                           reported_mask);
}

static void Task4_ReachPointFromDiagonal(float diagonal_heading,
                                         float straight_heading,
                                         int32_t finish_pulse,
                                         uint8_t remembered_object[4],
                                         char target_point,
                                         uint8_t reported_mask)
{
    int32_t total_pulse = 0;
    uint8_t start_line_cleared = 0U;

    g_state_machine.current_state = STATE_IDLE;

    while (1)
    {
        float yaw_error;
        float correction;
        float left_target;
        float right_target;

        Task4_ManualServiceLoop(remembered_object, target_point, reported_mask);
        total_pulse += Task2_GetPulseStep();

        yaw_error = Task2_NormalizeDelta(diagonal_heading - g_mpu_data.yaw);
        correction = (yaw_error * 0.60f) - (g_mpu_data.gyro_z_dps * 0.05f);
        correction = Task2_ClampFloat(correction,
                                      -TASK2_DIAGONAL_MAX_CORRECTION,
                                      TASK2_DIAGONAL_MAX_CORRECTION);

        left_target = TASK2_DIAGONAL_BASE_SPEED - correction;
        right_target = TASK2_DIAGONAL_BASE_SPEED + correction;
        Task2_SetManualTargets(left_target, right_target);

        if ((total_pulse >= TASK2_DIAGONAL_MIN_TRAVEL_PULSE) && (Task2_LineDetected() == 0U))
        {
            start_line_cleared = 1U;
        }

        if ((total_pulse >= finish_pulse) ||
            ((start_line_cleared != 0U) && (Task2_LineDetected() != 0U)))
        {
            break;
        }

        if (total_pulse >= TASK2_DIAGONAL_PULSE_LIMIT)
        {
            break;
        }
    }

    Task2_StopMotion(60U);
    Task4_AlignToHeading(straight_heading,
                         remembered_object,
                         target_point,
                         reported_mask);
}

static void Task4_ReachPointFromDiagonal_BD(float diagonal_heading,
                                            float straight_heading,
                                            int32_t finish_pulse,
                                            uint8_t remembered_object[4],
                                            char target_point,
                                            uint8_t reported_mask)
{
    int32_t total_pulse = 0;
    uint8_t start_line_cleared = 0U;

    (void)straight_heading;
    g_state_machine.current_state = STATE_IDLE;

    while (1)
    {
        float yaw_error;
        float correction;
        float left_target;
        float right_target;

        Task4_ManualServiceLoop(remembered_object, target_point, reported_mask);
        total_pulse += Task2_GetPulseStep();

        yaw_error = Task2_NormalizeDelta(diagonal_heading - g_mpu_data.yaw);
        correction = (yaw_error * 0.60f) - (g_mpu_data.gyro_z_dps * 0.05f);
        correction = Task2_ClampFloat(correction,
                                      -TASK2_DIAGONAL_MAX_CORRECTION,
                                      TASK2_DIAGONAL_MAX_CORRECTION);

        left_target = TASK2_DIAGONAL_BASE_SPEED - correction;
        right_target = TASK2_DIAGONAL_BASE_SPEED + correction;
        Task2_SetManualTargets(left_target, right_target);

        if ((total_pulse >= TASK2_DIAGONAL_MIN_TRAVEL_PULSE) && (Task2_LineDetected() == 0U))
        {
            start_line_cleared = 1U;
        }

        if ((total_pulse >= finish_pulse) ||
            ((start_line_cleared != 0U) && (Task2_LineDetected() != 0U)))
        {
            break;
        }

        if (total_pulse >= TASK2_DIAGONAL_PULSE_LIMIT)
        {
            break;
        }
    }

    Task2_StopMotion(0U);
}

static void Task4_FollowArcUntilYawWindow(float target_delta_deg,
                                          float tolerance_deg,
                                          uint8_t remembered_object[4],
                                          char target_point,
                                          uint8_t reported_mask)
{
    float start_yaw;
    uint8_t stable_count = 0U;

    g_state_machine.current_state = STATE_LINE_FOLLOW;
    start_yaw = g_mpu_data.yaw;

    while (1)
    {
        float yaw_delta;
        float yaw_error;

        Task4_LineFollowServiceLoop(remembered_object, target_point, reported_mask);
        yaw_delta = Task2_AbsFloat(Task2_NormalizeDelta(g_mpu_data.yaw - start_yaw));
        yaw_error = Task2_AbsFloat(yaw_delta - target_delta_deg);

        if (yaw_error <= tolerance_deg)
        {
            stable_count++;
            if (stable_count >= TASK2_STRAIGHT_CAPTURE_STABLE_COUNT)
            {
                break;
            }
        }
        else
        {
            stable_count = 0U;
        }
    }

}

static void Task4_ContinueLineFollowMs(uint32_t duration_ms,
                                       uint8_t remembered_object[4],
                                       char target_point,
                                       uint8_t reported_mask)
{
    uint32_t start_tick = HAL_GetTick();

    g_state_machine.current_state = STATE_LINE_FOLLOW;

    while ((HAL_GetTick() - start_tick) < duration_ms)
    {
        Task4_LineFollowServiceLoop(remembered_object, target_point, reported_mask);
    }

    g_state_machine.current_state = STATE_IDLE;
    Task2_StopMotion(80U);
}

static void Task4_FollowArcUntilYawReached(float start_yaw,
                                           float target_delta_deg,
                                           uint8_t remembered_object[4],
                                           char target_point,
                                           uint8_t reported_mask)
{
    g_state_machine.current_state = STATE_LINE_FOLLOW;

    while (1)
    {
        float yaw_delta;

        Task4_LineFollowServiceLoop(remembered_object, target_point, reported_mask);
        yaw_delta = Task2_AbsFloat(Task2_NormalizeDelta(g_mpu_data.yaw - start_yaw));

        if (yaw_delta >= target_delta_deg)
        {
            break;
        }
    }

    g_state_machine.current_state = STATE_IDLE;
    Task2_StopMotion(80U);
}

static void Task4_FollowUntilAbsoluteYaw(float target_yaw,
                                         float tolerance_deg,
                                         uint8_t remembered_object[4],
                                         char target_point,
                                         uint8_t reported_mask)
{
    uint8_t stable_count = 0U;

    g_state_machine.current_state = STATE_LINE_FOLLOW;

    while (1)
    {
        float yaw_error;

        Task4_LineFollowServiceLoop(remembered_object, target_point, reported_mask);
        yaw_error = Task2_NormalizeDelta(target_yaw - g_mpu_data.yaw);

        if (Task2_AbsFloat(yaw_error) <= tolerance_deg)
        {
            stable_count++;
            if (stable_count >= TASK2_STRAIGHT_CAPTURE_STABLE_COUNT)
            {
                break;
            }
        }
        else
        {
            stable_count = 0U;
        }
    }

    g_state_machine.current_state = STATE_IDLE;
    Task2_StopMotion(80U);
}
void Task3_Run(void)
{
    int32_t total_pulse = 0;
    uint8_t passed_B = 0U;
    uint8_t passed_C = 0U;
    uint8_t passed_D = 0U;
    uint8_t is_yaw_cleared_at_C = 0U;
    int delay_timer_C = 0;
    float yaw_at_C = 0.0f;
    float abs_yaw = 0.0f;
    float yaw_diff_from_C = 0.0f;
    uint8_t reported_object_mask = 0U;
    uint8_t detected_object = OBJ_NONE;
    CarEvent_TypeDef event = EVENT_NONE;

    TurnOn_LED_PC13();
    delay_ms(500);
    Task_LineFollowEnter(1U, 3);
    OpenMV_ClearNewFlag();

    while (1)
    {
        MPU6050_ReadAll(&g_mpu_data);

        total_pulse += g_encoder_left;

        abs_yaw = g_mpu_data.yaw;
        if (abs_yaw < 0.0f)
        {
            abs_yaw = -abs_yaw;
        }

        if ((passed_B == 0U) && (total_pulse >= 6800))
        {
            TurnOn_LED_PC13();
            passed_B = 1U;
            detected_object = Task_DetectPointObject(OPENMV_CMD_DETECT_LEFT, reported_object_mask);
            Task3_ReportPointResult('B', &detected_object, &reported_object_mask);
            g_state_machine.current_state = STATE_LINE_FOLLOW;
        }

        if ((passed_C == 0U) && (abs_yaw >= 178.0f))
        {
            TurnOn_LED_PC13();
            passed_C = 1U;
            total_pulse = 0;
            delay_timer_C = 100;
            detected_object = Task_DetectPointObject(OPENMV_CMD_DETECT_LEFT, reported_object_mask);
            Task3_ReportPointResult('C', &detected_object, &reported_object_mask);
            g_state_machine.current_state = STATE_LINE_FOLLOW;
        }

        if ((passed_C != 0U) && (is_yaw_cleared_at_C == 0U))
        {
            if (delay_timer_C > 0)
            {
                delay_timer_C--;
            }
            else
            {
                yaw_at_C = g_mpu_data.yaw;
                is_yaw_cleared_at_C = 1U;
            }
        }

        if ((passed_C != 0U) && (passed_D == 0U) && (total_pulse >= 6750))
        {
            TurnOn_LED_PC13();
            passed_D = 1U;
            detected_object = Task_DetectPointObject(OPENMV_CMD_DETECT_LEFT, reported_object_mask);
            Task3_ReportPointResult('D', &detected_object, &reported_object_mask);
            g_state_machine.current_state = STATE_LINE_FOLLOW;
        }

        yaw_diff_from_C = g_mpu_data.yaw - yaw_at_C;
        if (yaw_diff_from_C < 0.0f)
        {
            yaw_diff_from_C = -yaw_diff_from_C;
        }

        if ((is_yaw_cleared_at_C != 0U) && (yaw_diff_from_C >= 176.0f))
        {
            TurnOn_LED_PC13();
            detected_object = Task_DetectPointObject(OPENMV_CMD_DETECT_LEFT, reported_object_mask);
            Task3_ReportPointResult('A', &detected_object, &reported_object_mask);
            delay_ms(100);
            g_state_machine.current_state = STATE_IDLE;
            Task_StopAndHold();
        }

        event = DetectEvent();
        SM_Process(&g_state_machine, event);
        Debug_LogIR(event);
        Tick_LED_PC13();
        delay_ms(10);
    }
}
void Task4_Run(void)
{
    float first_diagonal_heading = -TASK2_DIAGONAL_ANGLE_OFFSET_AC;
    float second_diagonal_heading = 180.0f + TASK2_DIAGONAL_ANGLE_OFFSET_BD;
    uint8_t reported_object_mask = 0U;
    uint8_t detected_object = OBJ_NONE;
    Task2FlowContext_TypeDef ctx;

    ctx.state = TASK2_FLOW_BOOT_WAIT;
    ctx.state_tick = HAL_GetTick();
    ctx.last_service_tick = HAL_GetTick();
    ctx.stable_count = 0U;
    ctx.pulse_accum = 0;
    ctx.initial_raw_byte = 0U;
    ctx.last_position = 0;
    ctx.target_heading = 0.0f;
    ctx.a_yaw = 0.0f;
    ctx.start_yaw = 0.0f;

    TurnOn_LED_PC13();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if ((now - ctx.last_service_tick) < 10U)
        {
            continue;
        }
        ctx.last_service_tick = now;

        MPU6050_ReadAll(&g_mpu_data);
        Tick_LED_PC13();

        switch (ctx.state)
        {
        case TASK2_FLOW_BOOT_WAIT:
            if ((now - ctx.state_tick) >= 500U)
            {
                Task_LineFollowEnter(4U, 3U);
                Task2_StopNow();
                OpenMV_ClearNewFlag();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_PRESTART_STOP);
            }
            break;

        case TASK2_FLOW_PRESTART_STOP:
            Task2_StopNow();
            if ((now - ctx.state_tick) >= 150U)
            {
                ctx.a_yaw = g_mpu_data.yaw;
                ctx.target_heading = first_diagonal_heading;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_ALIGN_AC);
            }
            break;

        case TASK2_FLOW_ALIGN_AC:
            if (Task2_ServiceAlignStep(&ctx,
                                       ctx.target_heading,
                                       TASK2_ALIGN_TOLERANCE_DEG) != 0U)
            {
                Task2_StopNow();
                ctx.pulse_accum = 0;
                ctx.initial_raw_byte = g_ir_data.raw_byte;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_DIAG_AC);
            }
            break;

        case TASK2_FLOW_DIAG_AC:
            if (Task2_ServiceDiagonalStep(&ctx,
                                          first_diagonal_heading,
                                          TASK2_DIAGONAL_FINISH_PULSE,
                                          1.20f,
                                          0.15f,
                                          1U) != 0U)
            {
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_TURN_LEFT_C);
                ctx.target_heading = Task2_NormalizeDelta(g_mpu_data.yaw + TASK2_C_RETURN_LEFT_DELTA);
            }
            break;

        case TASK2_FLOW_TURN_LEFT_C:
            if (Task2_ServiceAlignStep(&ctx,
                                       ctx.target_heading,
                                       TASK2_CB_ALIGN_TOLERANCE_DEG) != 0U)
            {
                Task2_StopNow();
                ctx.last_position = 0;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_CAPTURE_C);
            }
            break;

        case TASK2_FLOW_CAPTURE_C:
            if (Task2_ServiceCaptureTrackStep(&ctx) != 0U)
            {
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_REPORT_C);
            }
            break;

        case TASK2_FLOW_REPORT_C:
            TurnOn_LED_PC13();
            detected_object = Task_DetectPointObject(OPENMV_CMD_DETECT_RIGHT, reported_object_mask);
            Task4_ReportPointResult('C', &detected_object, &reported_object_mask);
            Task2_EnterFlowState(&ctx, TASK2_FLOW_FOLLOW_TO_B);
            break;

        case TASK2_FLOW_FOLLOW_TO_B:
            if (Task2_ServiceFollowAbsoluteYawStep(&ctx,
                                                   TASK2_CB_ARC_FINISH_YAW_DELTA,
                                                   TASK2_CB_ALIGN_TOLERANCE_DEG) != 0U)
            {
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_REPORT_B);
            }
            break;

        case TASK2_FLOW_REPORT_B:
            TurnOn_LED_PC13();
            detected_object = Task_DetectPointObject(OPENMV_CMD_DETECT_RIGHT, reported_object_mask);
            Task4_ReportPointResult('B', &detected_object, &reported_object_mask);
            ctx.target_heading = 180.0f;
            Task2_EnterFlowState(&ctx, TASK2_FLOW_ALIGN_180_PRE_BD);
            break;

        case TASK2_FLOW_ALIGN_180_PRE_BD:
            if (Task2_ServiceAlignStep(&ctx,
                                       ctx.target_heading,
                                       TASK2_CB_ALIGN_TOLERANCE_DEG) != 0U)
            {
                Task2_StopNow();
                ctx.target_heading = second_diagonal_heading;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_ALIGN_BD);
            }
            break;

        case TASK2_FLOW_ALIGN_BD:
            if (Task2_ServiceAlignStep(&ctx,
                                       ctx.target_heading,
                                       TASK2_ALIGN_TOLERANCE_DEG) != 0U)
            {
                Task2_StopNow();
                ctx.pulse_accum = 0;
                ctx.initial_raw_byte = g_ir_data.raw_byte;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_DIAG_BD);
            }
            break;

        case TASK2_FLOW_DIAG_BD:
            if (Task2_ServiceDiagonalStep(&ctx,
                                          second_diagonal_heading,
                                          TASK2_DIAGONAL_FINISH_PULSE_BD,
                                          0.60f,
                                          0.05f,
                                          1U) != 0U)
            {
                Task2_StopNow();
                ctx.pulse_accum = 0;
                ctx.last_position = 0;
                Task2_EnterFlowState(&ctx, TASK2_FLOW_FOLLOW_TO_D);
            }
            break;

        case TASK2_FLOW_FOLLOW_TO_D:
            ctx.start_yaw = ctx.a_yaw;
            if (Task2_ServiceFollowArcReachedStep(&ctx,
                                                  TASK2_A_TO_D_YAW_DELTA) != 0U)
            {
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_REPORT_D);
            }
            break;

        case TASK2_FLOW_ALIGN_D:
            Task2_EnterFlowState(&ctx, TASK2_FLOW_REPORT_D);
            break;

        case TASK2_FLOW_REPORT_D:
            TurnOn_LED_PC13();
            detected_object = Task_DetectPointObject(OPENMV_CMD_DETECT_LEFT, reported_object_mask);
            Task4_ReportPointResult('D', &detected_object, &reported_object_mask);
            ctx.start_yaw = g_mpu_data.yaw;
            Task2_EnterFlowState(&ctx, TASK2_FLOW_FOLLOW_TO_A);
            break;

        case TASK2_FLOW_FOLLOW_TO_A:
            if (Task2_ServiceFollowArcStep(&ctx,
                                           TASK2_D_TO_A_YAW_DELTA,
                                           TASK2_DA_ARC_TOLERANCE_DEG) != 0U)
            {
                Task2_EnterFlowState(&ctx, TASK2_FLOW_DELAY_A_REPORT);
            }
            break;

        case TASK2_FLOW_DELAY_A_REPORT:
            g_state_machine.current_state = STATE_LINE_FOLLOW;
            Debug_LogIR(EVENT_NONE);
            if ((now - ctx.state_tick) >= TASK_POINT_REPORT_DELAY_MS)
            {
                Task2_StopNow();
                Task2_EnterFlowState(&ctx, TASK2_FLOW_REPORT_A);
            }
            break;

        case TASK2_FLOW_REPORT_A:
            TurnOn_LED_PC13();
            detected_object = Task_DetectPointObject(OPENMV_CMD_DETECT_LEFT, reported_object_mask);
            Task4_ReportPointResult('A', &detected_object, &reported_object_mask);
            Task2_EnterFlowState(&ctx, TASK2_FLOW_HOLD);
            break;

        case TASK2_FLOW_HOLD:
        default:
            g_state_machine.current_state = STATE_IDLE;
            Task2_StopNow();
            break;
        }
    }
}

