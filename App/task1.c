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
    Task_RunBasicLineFollow(2U);
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
    Task_RunBasicLineFollow(4U);
}
