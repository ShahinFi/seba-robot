#include "actuator_control.h"

#include "current_sensor/current_sensor.h"
#include "motor/motor.h"

#include "stm32g4xx_hal.h"

#include <stddef.h>

/*
 * Startup defaults for the current loop. The Raspberry Pi tuner
 * uses matching values so the UI reflects firmware behavior at
 * power-up.
 */
#define ACTUATOR_CONTROL_DEFAULT_PERIOD_MS           1U
#define ACTUATOR_CONTROL_DEFAULT_BATTERY_MV     11100L
#define ACTUATOR_CONTROL_DEFAULT_KP_MV_PER_A     3000L
#define ACTUATOR_CONTROL_DEFAULT_KI_MV_PER_A_S      0L
#define ACTUATOR_CONTROL_DEFAULT_INTEGRAL_MV     3000L
#define ACTUATOR_CONTROL_DEFAULT_MAX_CURRENT_MA   5000L
#define ACTUATOR_CONTROL_DEFAULT_MAX_COMMAND     100
#define ACTUATOR_CONTROL_DEFAULT_KTW_MNM_PER_A  1000L

/*
 * TIM7 runs the actuator scheduler at 1 kHz. The configured
 * control period divides this fixed tick.
 */
#define ACTUATOR_CONTROL_TIMER_HZ               1000U

/*
 * Current control runs from TIM7. Command/config updates can
 * arrive from the serial command path, so shared state is
 * protected with short critical sections.
 */
static ActuatorControlConfig config;
static ActuatorControlStatus status;
static volatile uint32_t control_tick_count;

static int32_t ActuatorControl_ClampInt32(
    int32_t value,
    int32_t minimum,
    int32_t maximum
);

static int16_t ActuatorControl_ClampCommandPermille(
    int32_t command_permille
);

static int32_t ActuatorControl_TorqueToCurrent(
    int32_t torque_mnm
);

static int16_t ActuatorControl_UpdateChannel(
    int32_t reference_ma,
    int32_t measured_ma,
    int32_t *integral_mv,
    int32_t *error_ma
);

static void ActuatorControl_ResetState(void);

static int32_t ActuatorControl_CommandToVoltageLimit(void);

static int16_t ActuatorControl_VoltageToCommandPermille(
    int32_t control_mv
);

static void ActuatorControl_TimerInit(void);
static void ActuatorControl_ProcessTick(void);
static uint32_t ActuatorControl_EnterCritical(void);
static void ActuatorControl_ExitCritical(uint32_t interrupt_state);

void ActuatorControl_Init(void)
{
    config.control_period_ms =
        ACTUATOR_CONTROL_DEFAULT_PERIOD_MS;

    config.battery_voltage_mv =
        ACTUATOR_CONTROL_DEFAULT_BATTERY_MV;

    config.proportional_gain_mv_per_a =
        ACTUATOR_CONTROL_DEFAULT_KP_MV_PER_A;

    config.integral_gain_mv_per_a_s =
        ACTUATOR_CONTROL_DEFAULT_KI_MV_PER_A_S;

    config.integral_limit_mv =
        ACTUATOR_CONTROL_DEFAULT_INTEGRAL_MV;

    config.max_current_reference_ma =
        ACTUATOR_CONTROL_DEFAULT_MAX_CURRENT_MA;

    config.max_command_percent =
        ACTUATOR_CONTROL_DEFAULT_MAX_COMMAND;

    config.wheel_torque_constant_mnm_per_a =
        ACTUATOR_CONTROL_DEFAULT_KTW_MNM_PER_A;

    ActuatorControl_ResetState();
    Motor_Disable();
    ActuatorControl_TimerInit();
}

static void ActuatorControl_ProcessTick(void)
{
    CurrentSensorReading left;
    CurrentSensorReading right;
    uint32_t control_period_ms;

    if (!status.enabled)
    {
        control_tick_count = 0U;
        return;
    }

    control_period_ms =
        config.control_period_ms;

    control_tick_count++;

    if (control_tick_count < control_period_ms)
    {
        return;
    }

    control_tick_count = 0U;

    if (!CurrentSensor_ReadBoth(
            &left,
            &right
        ))
    {
        status.read_failed = true;
        ActuatorControl_Disable();
        return;
    }

    status.left_current_measured_ma =
        left.current_ma;

    status.right_current_measured_ma =
        right.current_ma;

    status.fault_active =
        left.fault_active ||
        right.fault_active;

    if (status.fault_active)
    {
        /*
         * Current-sensor faults indicate an electrical fault
         * condition reported by the ACS711 boards.
         */
        ActuatorControl_Disable();
        return;
    }

    status.left_command_permille =
        ActuatorControl_UpdateChannel(
            status.left_current_reference_ma,
            status.left_current_measured_ma,
            &status.left_integral_mv,
            &status.left_error_ma
        );

    status.right_command_permille =
        ActuatorControl_UpdateChannel(
            status.right_current_reference_ma,
            status.right_current_measured_ma,
            &status.right_integral_mv,
            &status.right_error_ma
        );

    Motor_SetLeftPermille(
        status.left_command_permille
    );

    Motor_SetRightPermille(
        status.right_command_permille
    );
}

void ActuatorControl_Enable(void)
{
    uint32_t interrupt_state;

    /*
     * Keep the timer ISR inactive while the motor drivers are
     * waking, then enable current control from a clean state.
     */
    interrupt_state =
        ActuatorControl_EnterCritical();

    status.enabled = false;
    control_tick_count = 0U;

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    Motor_Enable();

    interrupt_state =
        ActuatorControl_EnterCritical();

    status.enabled = true;
    status.read_failed = false;
    status.fault_active = false;
    status.left_integral_mv = 0;
    status.right_integral_mv = 0;
    status.left_command_permille = 0;
    status.right_command_permille = 0;
    control_tick_count = 0U;

    ActuatorControl_ExitCritical(
        interrupt_state
    );
}

void ActuatorControl_Disable(void)
{
    uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    /*
     * Disable clears all command and integrator state before
     * stopping the motor outputs, so the next enable starts from
     * zero current demand.
     */
    status.enabled = false;
    status.left_current_reference_ma = 0;
    status.right_current_reference_ma = 0;
    status.left_error_ma = 0;
    status.right_error_ma = 0;
    status.left_integral_mv = 0;
    status.right_integral_mv = 0;
    status.left_command_permille = 0;
    status.right_command_permille = 0;
    control_tick_count = 0U;

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    Motor_StopAll();
    Motor_Disable();
}

bool ActuatorControl_IsEnabled(void)
{
    bool enabled;
    uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    enabled = status.enabled;

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    return enabled;
}

bool ActuatorControl_SetCurrentReferences(
    int32_t left_current_ma,
    int32_t right_current_ma
)
{
    if (
        left_current_ma >
        config.max_current_reference_ma ||
        left_current_ma <
        -config.max_current_reference_ma ||
        right_current_ma >
        config.max_current_reference_ma ||
        right_current_ma <
        -config.max_current_reference_ma
    )
    {
        return false;
    }

    /*
     * References are updated together so the timer tick never
     * sees a mixed left/right command pair.
     */
    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    status.left_current_reference_ma =
        left_current_ma;

    status.right_current_reference_ma =
        right_current_ma;

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool ActuatorControl_SetWheelTorqueReferences(
    int32_t left_torque_mnm,
    int32_t right_torque_mnm
)
{
    const int32_t left_current_ma =
        ActuatorControl_TorqueToCurrent(
            left_torque_mnm
        );

    const int32_t right_current_ma =
        ActuatorControl_TorqueToCurrent(
            right_torque_mnm
        );

    return ActuatorControl_SetCurrentReferences(
        left_current_ma,
        right_current_ma
    );
}

bool ActuatorControl_SetControlPeriod(uint32_t control_period_ms)
{
    if (
        control_period_ms == 0U ||
        control_period_ms > 1000U
    )
    {
        return false;
    }

    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    /*
     * Reset the divider so the new period starts from a full
     * timing interval.
     */
    config.control_period_ms =
        control_period_ms;

    control_tick_count = 0U;

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool ActuatorControl_SetBatteryVoltage(int32_t battery_voltage_mv)
{
    if (battery_voltage_mv <= 0)
    {
        return false;
    }

    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    config.battery_voltage_mv =
        battery_voltage_mv;

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool ActuatorControl_SetProportionalGain(int32_t gain_mv_per_a)
{
    if (gain_mv_per_a < 0)
    {
        return false;
    }

    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    config.proportional_gain_mv_per_a =
        gain_mv_per_a;

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool ActuatorControl_SetIntegralGain(int32_t gain_mv_per_a_s)
{
    if (gain_mv_per_a_s < 0)
    {
        return false;
    }

    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    config.integral_gain_mv_per_a_s =
        gain_mv_per_a_s;

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool ActuatorControl_SetIntegralLimit(int32_t integral_limit_mv)
{
    if (integral_limit_mv < 0)
    {
        return false;
    }

    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    /*
     * Apply the new integrator limit immediately to any stored
     * integral state.
     */
    config.integral_limit_mv =
        integral_limit_mv;

    status.left_integral_mv =
        ActuatorControl_ClampInt32(
            status.left_integral_mv,
            -config.integral_limit_mv,
            config.integral_limit_mv
        );

    status.right_integral_mv =
        ActuatorControl_ClampInt32(
            status.right_integral_mv,
            -config.integral_limit_mv,
            config.integral_limit_mv
        );

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool ActuatorControl_SetMaxCurrentReference(int32_t max_current_ma)
{
    if (max_current_ma < 0)
    {
        return false;
    }

    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    /*
     * Apply a reduced current limit immediately to any active
     * current references.
     */
    config.max_current_reference_ma =
        max_current_ma;

    status.left_current_reference_ma =
        ActuatorControl_ClampInt32(
            status.left_current_reference_ma,
            -config.max_current_reference_ma,
            config.max_current_reference_ma
        );

    status.right_current_reference_ma =
        ActuatorControl_ClampInt32(
            status.right_current_reference_ma,
            -config.max_current_reference_ma,
            config.max_current_reference_ma
        );

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool ActuatorControl_SetMaxCommandPercent(int16_t max_command_percent)
{
    if (
        max_command_percent < 0 ||
        max_command_percent > 100
    )
    {
        return false;
    }

    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    config.max_command_percent =
        max_command_percent;

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool ActuatorControl_SetWheelTorqueConstant(int32_t torque_constant_mnm_per_a)
{
    if (torque_constant_mnm_per_a <= 0)
    {
        return false;
    }

    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    /*
     * The torque constant is used on the next torque-reference
     * update to convert wheel torque into motor current.
     */
    config.wheel_torque_constant_mnm_per_a =
        torque_constant_mnm_per_a;

    ActuatorControl_ExitCritical(
        interrupt_state
    );

    return true;
}

void ActuatorControl_GetConfig(
    ActuatorControlConfig *output
)
{
    if (output == NULL)
    {
        return;
    }

    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    *output = config;

    ActuatorControl_ExitCritical(
        interrupt_state
    );
}

void ActuatorControl_GetStatus(
    ActuatorControlStatus *output
)
{
    if (output == NULL)
    {
        return;
    }

    const uint32_t interrupt_state =
        ActuatorControl_EnterCritical();

    *output = status;

    ActuatorControl_ExitCritical(
        interrupt_state
    );
}

static int32_t ActuatorControl_ClampInt32(
    int32_t value,
    int32_t minimum,
    int32_t maximum
)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static int16_t ActuatorControl_ClampCommandPermille(
    int32_t command_permille
)
{
    const int32_t maximum_command_permille =
        (int32_t)config.max_command_percent *
        10L;

    command_permille =
        ActuatorControl_ClampInt32(
            command_permille,
            -maximum_command_permille,
            maximum_command_permille
        );

    return (int16_t)command_permille;
}

static int32_t ActuatorControl_TorqueToCurrent(
    int32_t torque_mnm
)
{
    /*
     * torque_constant is mN*m/A, so multiplying torque by 1000
     * converts amperes to milliamps.
     */
    return (int32_t)(
        (
            (int64_t)torque_mnm *
            1000LL
        ) /
        config.wheel_torque_constant_mnm_per_a
    );
}

static int16_t ActuatorControl_UpdateChannel(
    int32_t reference_ma,
    int32_t measured_ma,
    int32_t *integral_mv,
    int32_t *error_ma
)
{
    int32_t proportional_mv;
    int32_t candidate_integral_mv;
    int32_t requested_mv;
    int32_t saturated_mv;
    const int32_t voltage_limit_mv =
        ActuatorControl_CommandToVoltageLimit();

    if (reference_ma == 0)
    {
        /*
         * A zero reference is treated as a hard stop for this
         * channel so residual measured current cannot command
         * reverse PWM.
         */
        *error_ma =
            -measured_ma;

        *integral_mv = 0;
        return 0;
    }

    *error_ma =
        reference_ma -
        measured_ma;

    /*
     * Gains use physical actuator-control units:
     * proportional gain is mV/A and integral gain is
     * mV/(A*s).
     */
    proportional_mv = (int32_t)(
        (
            (int64_t)config.proportional_gain_mv_per_a *
            *error_ma
        ) /
        1000LL
    );

    candidate_integral_mv =
        *integral_mv +
        (int32_t)(
            (
                (int64_t)config.integral_gain_mv_per_a_s *
                *error_ma *
                (int32_t)config.control_period_ms
            ) /
            1000000LL
        );

    candidate_integral_mv =
        ActuatorControl_ClampInt32(
            candidate_integral_mv,
            -config.integral_limit_mv,
            config.integral_limit_mv
        );

    requested_mv =
        proportional_mv +
        candidate_integral_mv;

    saturated_mv =
        ActuatorControl_ClampInt32(
            requested_mv,
            -voltage_limit_mv,
            voltage_limit_mv
        );

    if (
        requested_mv == saturated_mv ||
        (
            requested_mv > saturated_mv &&
            *error_ma < 0
        ) ||
        (
            requested_mv < saturated_mv &&
            *error_ma > 0
        )
    )
    {
        /*
         * Integrate only when the output is not saturated, or
         * when the error would drive the saturated output back
         * toward the valid range.
         */
        *integral_mv =
            candidate_integral_mv;
    }

    requested_mv =
        proportional_mv +
        *integral_mv;

    saturated_mv =
        ActuatorControl_ClampInt32(
            requested_mv,
            -voltage_limit_mv,
            voltage_limit_mv
        );

    return ActuatorControl_VoltageToCommandPermille(
        saturated_mv
    );
}

static void ActuatorControl_ResetState(void)
{
    status.enabled = false;
    status.fault_active = false;
    status.read_failed = false;
    status.left_current_reference_ma = 0;
    status.right_current_reference_ma = 0;
    status.left_current_measured_ma = 0;
    status.right_current_measured_ma = 0;
    status.left_error_ma = 0;
    status.right_error_ma = 0;
    status.left_integral_mv = 0;
    status.right_integral_mv = 0;
    status.left_command_permille = 0;
    status.right_command_permille = 0;
    control_tick_count = 0U;
}

static int32_t ActuatorControl_CommandToVoltageLimit(void)
{
    /*
     * Max PWM limits the maximum average motor voltage available
     * to the current controller.
     */
    return (int32_t)(
        (
            (int64_t)config.battery_voltage_mv *
            config.max_command_percent
        ) /
        100L
    );
}

static int16_t ActuatorControl_VoltageToCommandPermille(
    int32_t control_mv
)
{
    int32_t command_permille;

    /*
     * Convert signed average motor voltage to signed PWM
     * per-mille with symmetric rounding.
     */
    if (control_mv >= 0)
    {
        command_permille = (int32_t)(
            (
                (int64_t)control_mv *
                1000LL +
                config.battery_voltage_mv / 2L
            ) /
            config.battery_voltage_mv
        );
    }
    else
    {
        command_permille = (int32_t)(
            (
                (int64_t)control_mv *
                1000LL -
                config.battery_voltage_mv / 2L
            ) /
            config.battery_voltage_mv
        );
    }

    return ActuatorControl_ClampCommandPermille(
        command_permille
    );
}

static void ActuatorControl_TimerInit(void)
{
    __HAL_RCC_TIM7_FORCE_RESET();
    __HAL_RCC_TIM7_RELEASE_RESET();
    __HAL_RCC_TIM7_CLK_ENABLE();

    TIM7->CR1 = 0U;
    TIM7->CR2 = 0U;
    TIM7->DIER = 0U;

    /*
     * TIM7 provides a 1 kHz actuator scheduler. The controller
     * can then divide this fixed tick by config.control_period_ms.
     */
    TIM7->PSC = 0U;
    TIM7->ARR =
        (
            SystemCoreClock /
            ACTUATOR_CONTROL_TIMER_HZ
        ) -
        1U;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->SR = 0U;
    TIM7->DIER = TIM_DIER_UIE;

    HAL_NVIC_SetPriority(
        TIM7_DAC_IRQn,
        6U,
        0U
    );

    HAL_NVIC_EnableIRQ(
        TIM7_DAC_IRQn
    );

    TIM7->CR1 =
        TIM_CR1_CEN;
}

static uint32_t ActuatorControl_EnterCritical(void)
{
    const uint32_t interrupt_state =
        __get_PRIMASK();

    __disable_irq();

    return interrupt_state;
}

static void ActuatorControl_ExitCritical(uint32_t interrupt_state)
{
    __set_PRIMASK(
        interrupt_state
    );
}

void TIM7_DAC_IRQHandler(void)
{
    if ((TIM7->SR & TIM_SR_UIF) == 0U)
    {
        return;
    }

    TIM7->SR &= (uint32_t)~TIM_SR_UIF;

    ActuatorControl_ProcessTick();
}
