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
    OpenMV_ClearNewFlag();
    SM_Process(&g_state_machine, EVENT_START);
}

static void Task_RunBasicLineFollow(uint8_t task_id)
{
    CarEvent_TypeDef event;

    TurnOn_LED_PC13();
    delay_ms(500);
    Task_LineFollowEnter(task_id, 3);

    while (1)
    {
        MPU6050_ReadAll(&g_mpu_data);
        event = DetectEvent();
        SM_Process(&g_state_machine, event);
        Debug_LogIR(event);
        Tick_LED_PC13();
        delay_ms(10);
    }
}

#define TASK2_DIAGONAL_ANGLE_OFFSET_AC        39.0f
#define TASK2_DIAGONAL_ANGLE_OFFSET_BD        41.0f
#define TASK2_DIAGONAL_MIN_TRAVEL_PULSE       1200
#define TASK2_DIAGONAL_ASSIST_PULSE           0
#define TASK2_DIAGONAL_FINISH_PULSE           7900
#define TASK2_DIAGONAL_FINISH_PULSE_BD        6000
#define TASK2_DIAGONAL_PULSE_LIMIT            9300
#define TASK2_DIAGONAL_BASE_SPEED             28.0f
#define TASK2_DIAGONAL_MAX_CORRECTION         10.0f
#define TASK2_DIAGONAL_IR_GAIN                 0.35f
#define TASK2_DIAGONAL_IR_MAX                  8.0f
#define TASK2_ALIGN_TOLERANCE_DEG              8.0f
#define TASK2_ALIGN_TURN_MAX                  22.0f
#define TASK2_ALIGN_TURN_MIN                   8.0f
#define TASK2_ALIGN_STABLE_COUNT               6U
#define TASK2_ALIGN_TIMEOUT_LOOPS           400U
#define TASK2_STRAIGHT_CAPTURE_TOLERANCE_DEG  10.0f
#define TASK2_STRAIGHT_CAPTURE_STABLE_COUNT    5U
#define TASK2_ARC_FINISH_YAW_DELTA           175.0f
#define TASK2_CB_ARC_FINISH_YAW_DELTA        167.0f
#define TASK2_CB_ALIGN_TOLERANCE_DEG           2.0f
#define TASK2_DA_ARC_TARGET_YAW_DELTA       178.0f
#define TASK2_DA_ARC_TOLERANCE_DEG            2.0f

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
            turn_cmd = yaw_error * 0.85f;

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

static uint8_t Task2_Center234Off(void)
{
    return ((g_ir_data.sensor[1] == 0U) &&
            (g_ir_data.sensor[2] == 0U) &&
            (g_ir_data.sensor[3] == 0U)) ? 1U : 0U;
}

static void Task2_ReachPointFromDiagonal(float diagonal_heading, float straight_heading, int32_t finish_pulse)
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
    Task2_AlignToHeading(straight_heading);
}

static void Task2_ReachPointFromDiagonal_BD(float diagonal_heading, float straight_heading, int32_t finish_pulse)
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

static void Task2_FollowArcUntilYawDelta(float finish_delta_deg)
{
    float start_yaw;

    g_state_machine.current_state = STATE_LINE_FOLLOW;
    start_yaw = g_mpu_data.yaw;

    while (1)
    {
        float yaw_delta;

        Task2_LineFollowServiceLoop();
        yaw_delta = Task2_NormalizeDelta(g_mpu_data.yaw - start_yaw);

        if (Task2_AbsFloat(yaw_delta) >= finish_delta_deg)
        {
            break;
        }
    }

    g_state_machine.current_state = STATE_IDLE;
    Task2_StopMotion(80U);
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

        if ((passed_B == 0U) && (total_pulse >= 6856))
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

        if ((passed_C != 0U) && (passed_D == 0U) && (total_pulse >= 6856))
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

        if ((is_yaw_cleared_at_C != 0U) && (yaw_diff_from_C >= 178.0f))
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

    TurnOn_LED_PC13();
    delay_ms(500);
    Task_LineFollowEnter(2U, 3U);
    Task2_StopMotion(150U);

    Task2_AlignToHeading(first_diagonal_heading);
    Task2_ReachPointFromDiagonal(first_diagonal_heading, 0.0f, TASK2_DIAGONAL_FINISH_PULSE);
    SYN6658_ReportStation('C');
    TurnOn_LED_PC13();

    Task2_FollowUntilAbsoluteYaw(TASK2_CB_ARC_FINISH_YAW_DELTA, TASK2_CB_ALIGN_TOLERANCE_DEG);
    SYN6658_ReportStation('B');
    TurnOn_LED_PC13();

    Task2_AlignToHeadingEx(180.0f, TASK2_CB_ALIGN_TOLERANCE_DEG);
    Task2_AlignToHeading(second_diagonal_heading);
    Task2_ReachPointFromDiagonal_BD(second_diagonal_heading, 180.0f, TASK2_DIAGONAL_FINISH_PULSE_BD);
    Task2_FollowUntilAbsoluteYaw(180.0f, TASK2_CB_ALIGN_TOLERANCE_DEG);
    Task2_AlignToHeadingEx(180.0f, TASK2_CB_ALIGN_TOLERANCE_DEG);
    SYN6658_ReportStation('D');
    TurnOn_LED_PC13();

    Task2_FollowArcUntilYawWindow(TASK2_DA_ARC_TARGET_YAW_DELTA, TASK2_DA_ARC_TOLERANCE_DEG);
    g_state_machine.current_state = STATE_IDLE;
    Task2_StopMotion(80U);
    SYN6658_ReportStation('A');
    TurnOn_LED_PC13();
    Task_StopAndHold();
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

static char Task3_GetTargetPoint(uint8_t passed_B, uint8_t passed_C, uint8_t passed_D)
{
    if (passed_B == 0U)
    {
        return 'B';
    }

    if (passed_C == 0U)
    {
        return 'C';
    }

    if (passed_D == 0U)
    {
        return 'D';
    }

    return 'A';
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

static void Task3_ReportPointResult(char point, uint8_t *object_slot)
{
    g_pid_left.target = 0.0f;
    g_pid_right.target = 0.0f;
    Motor_Stop();
    delay_ms(60);

    if (*object_slot != 0U)
    {
        Task3_SpeakPointObject(point, *object_slot);
        *object_slot = 0U;
    }
    else
    {
        SYN6658_ReportStation(point);
    }
}

static void Task4_RememberTargetObject(char target_point, uint8_t remembered_object[4])
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
        (remembered_object[point_index] == 0U))
    {
        remembered_object[point_index] = mv.object_id;
        TurnOn_LED_PC13();
    }

    OpenMV_ClearNewFlag();
}

static void Task4_ManualServiceLoop(uint8_t remembered_object[4], char target_point)
{
    Task2_ManualServiceLoop();
    Task4_RememberTargetObject(target_point, remembered_object);
}

static void Task4_LineFollowServiceLoop(uint8_t remembered_object[4], char target_point)
{
    Task2_LineFollowServiceLoop();
    Task4_RememberTargetObject(target_point, remembered_object);
}

static void Task4_ReportPointResult(char point, uint8_t *object_slot)
{
    Task2_StopMotion(60U);

    if (*object_slot != 0U)
    {
        Task3_SpeakPointObject(point, *object_slot);
        *object_slot = 0U;
    }
    else
    {
        SYN6658_ReportStation(point);
    }
}

static void Task4_AlignToHeadingEx(float target_heading,
                                   float tolerance_deg,
                                   uint8_t remembered_object[4],
                                   char target_point)
{
    uint32_t stable_count = 0U;
    uint32_t loop_count = 0U;

    g_state_machine.current_state = STATE_IDLE;

    while (loop_count < TASK2_ALIGN_TIMEOUT_LOOPS)
    {
        float yaw_error;
        float turn_cmd;

        Task4_ManualServiceLoop(remembered_object, target_point);
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
            turn_cmd = yaw_error * 0.85f;

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
                                 char target_point)
{
    Task4_AlignToHeadingEx(target_heading,
                           TASK2_ALIGN_TOLERANCE_DEG,
                           remembered_object,
                           target_point);
}

static void Task4_ReachPointFromDiagonal(float diagonal_heading,
                                         float straight_heading,
                                         int32_t finish_pulse,
                                         uint8_t remembered_object[4],
                                         char target_point)
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

        Task4_ManualServiceLoop(remembered_object, target_point);
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
    Task4_AlignToHeading(straight_heading, remembered_object, target_point);
}

static void Task4_ReachPointFromDiagonal_BD(float diagonal_heading,
                                            float straight_heading,
                                            int32_t finish_pulse,
                                            uint8_t remembered_object[4],
                                            char target_point)
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

        Task4_ManualServiceLoop(remembered_object, target_point);
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
                                          char target_point)
{
    float start_yaw;
    uint8_t stable_count = 0U;

    g_state_machine.current_state = STATE_LINE_FOLLOW;
    start_yaw = g_mpu_data.yaw;

    while (1)
    {
        float yaw_delta;
        float yaw_error;

        Task4_LineFollowServiceLoop(remembered_object, target_point);
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

static void Task4_FollowUntilAbsoluteYaw(float target_yaw,
                                         float tolerance_deg,
                                         uint8_t remembered_object[4],
                                         char target_point)
{
    uint8_t stable_count = 0U;

    g_state_machine.current_state = STATE_LINE_FOLLOW;

    while (1)
    {
        float yaw_error;

        Task4_LineFollowServiceLoop(remembered_object, target_point);
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
    uint8_t remembered_object[4] = {0U, 0U, 0U, 0U};
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

        if ((passed_B == 0U) && (total_pulse >= 6856))
        {
            TurnOn_LED_PC13();
            passed_B = 1U;
            Task3_ReportPointResult('B', &remembered_object[0]);
        }

        if ((passed_C == 0U) && (abs_yaw >= 178.0f))
        {
            TurnOn_LED_PC13();
            passed_C = 1U;
            total_pulse = 0;
            delay_timer_C = 100;
            Task3_ReportPointResult('C', &remembered_object[1]);
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

        if ((passed_C != 0U) && (passed_D == 0U) && (total_pulse >= 6856))
        {
            TurnOn_LED_PC13();
            passed_D = 1U;
            Task3_ReportPointResult('D', &remembered_object[2]);
        }

        yaw_diff_from_C = g_mpu_data.yaw - yaw_at_C;
        if (yaw_diff_from_C < 0.0f)
        {
            yaw_diff_from_C = -yaw_diff_from_C;
        }

        if (OpenMV_HasNewData())
        {
            OpenMV_DataTypeDef mv = OpenMV_GetResult();
            char target_point = Task3_GetTargetPoint(passed_B, passed_C, passed_D);
            uint8_t point_index = Task3_GetPointIndex(target_point);

            if ((mv.is_valid != 0U) &&
                (mv.object_id >= OBJ_LIGHTER) &&
                (mv.object_id <= OBJ_HAMMER) &&
                (remembered_object[point_index] == 0U))
            {
                remembered_object[point_index] = mv.object_id;
                TurnOn_LED_PC13();
            }

            OpenMV_ClearNewFlag();
        }

        if ((is_yaw_cleared_at_C != 0U) && (yaw_diff_from_C >= 178.0f))
        {
            TurnOn_LED_PC13();
            Task3_ReportPointResult('A', &remembered_object[3]);
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
    uint8_t remembered_object[4] = {0U, 0U, 0U, 0U};

    TurnOn_LED_PC13();
    delay_ms(500);
    Task_LineFollowEnter(4U, 3U);
    Task2_StopMotion(150U);
    OpenMV_ClearNewFlag();

    Task4_AlignToHeading(first_diagonal_heading, remembered_object, 'C');
    Task4_ReachPointFromDiagonal(first_diagonal_heading,
                                 0.0f,
                                 TASK2_DIAGONAL_FINISH_PULSE,
                                 remembered_object,
                                 'C');
    TurnOn_LED_PC13();
    Task4_ReportPointResult('C', &remembered_object[Task3_GetPointIndex('C')]);

    Task4_FollowUntilAbsoluteYaw(TASK2_CB_ARC_FINISH_YAW_DELTA,
                                 TASK2_CB_ALIGN_TOLERANCE_DEG,
                                 remembered_object,
                                 'B');
    TurnOn_LED_PC13();
    Task4_ReportPointResult('B', &remembered_object[Task3_GetPointIndex('B')]);

    Task4_AlignToHeadingEx(180.0f,
                           TASK2_CB_ALIGN_TOLERANCE_DEG,
                           remembered_object,
                           'D');
    Task4_AlignToHeading(second_diagonal_heading, remembered_object, 'D');
    Task4_ReachPointFromDiagonal_BD(second_diagonal_heading,
                                    180.0f,
                                    TASK2_DIAGONAL_FINISH_PULSE_BD,
                                    remembered_object,
                                    'D');
    Task4_FollowUntilAbsoluteYaw(180.0f,
                                 TASK2_CB_ALIGN_TOLERANCE_DEG,
                                 remembered_object,
                                 'D');
    Task4_AlignToHeadingEx(180.0f,
                           TASK2_CB_ALIGN_TOLERANCE_DEG,
                           remembered_object,
                           'D');
    TurnOn_LED_PC13();
    Task4_ReportPointResult('D', &remembered_object[Task3_GetPointIndex('D')]);

    Task4_FollowArcUntilYawWindow(TASK2_DA_ARC_TARGET_YAW_DELTA,
                                  TASK2_DA_ARC_TOLERANCE_DEG,
                                  remembered_object,
                                  'A');
    g_state_machine.current_state = STATE_IDLE;
    TurnOn_LED_PC13();
    Task4_ReportPointResult('A', &remembered_object[Task3_GetPointIndex('A')]);
    Task_StopAndHold();
}

