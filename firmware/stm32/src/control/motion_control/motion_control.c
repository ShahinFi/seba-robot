#include "motion_control.h"

#include "control/actuator/actuator.h"
#include "control/control_parameters.h"

#include <stddef.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

#define MOTION_CONTROL_MNM_PER_NM  1000.0F
#define MOTION_CONTROL_AUGMENTED_COUNT    6U

typedef struct
{
    float left_torque_mnm;
    float right_torque_mnm;
} MotionControlTorque;

static ControlParameters parameters;
static MotionControlStatus status;
static uint32_t last_command_ms;

static void MotionControl_ResetControllerState(void);
static void MotionControl_ResetCommandState(void);
static void MotionControl_ClearFaultState(void);
static void MotionControl_ClearFaultContext(void);
static void MotionControl_EnterFault(
    MotionControlFault fault,
    const RobotState *fault_state
);
static void MotionControl_UpdateCommandAge(void);
static void MotionControl_ApplyCommandWatchdog(void);

static float MotionControl_Clamp(
    float value,
    float minimum,
    float maximum
);

static float MotionControl_IntegrateTorque(
    float current_torque_mnm,
    float torque_rate_mnm_s,
    float sample_period_s
);

static int32_t MotionControl_TorqueToMillinewtonMeters(
    float torque_mnm
);

static uint32_t MotionControl_EnterCritical(void);
static void MotionControl_ExitCritical(uint32_t interrupt_state);

void MotionControl_Init(void)
{
    ControlParameters_Get(
        &parameters
    );

    status.enabled = false;
    status.fault_active = false;
    status.state_invalid = false;
    status.fall_detected = false;
    status.fault =
        MOTION_CONTROL_FAULT_NONE;
    MotionControl_ClearFaultContext();
    status.max_wheel_torque_mnm =
        parameters.max_wheel_torque_mnm;
    status.motion_gain_scale =
        parameters.motion_gain_scale;

    MotionControl_ResetCommandState();
    MotionControl_ResetControllerState();
}

void MotionControl_Enable(void)
{
    uint32_t interrupt_state =
        MotionControl_EnterCritical();

    status.enabled = false;
    MotionControl_ResetCommandState();
    MotionControl_ResetControllerState();
    MotionControl_ClearFaultState();

    MotionControl_ExitCritical(
        interrupt_state
    );

    Actuator_Enable();

    interrupt_state =
        MotionControl_EnterCritical();

    status.enabled = true;

    MotionControl_ExitCritical(
        interrupt_state
    );
}

void MotionControl_Disable(void)
{
    const uint32_t interrupt_state =
        MotionControl_EnterCritical();

    status.enabled = false;

    MotionControl_ResetCommandState();
    MotionControl_ResetControllerState();
    MotionControl_ClearFaultState();

    MotionControl_ExitCritical(
        interrupt_state
    );

    Actuator_Disable();
}

bool MotionControl_IsEnabled(void)
{
    bool enabled;
    const uint32_t interrupt_state =
        MotionControl_EnterCritical();

    enabled =
        status.enabled;

    MotionControl_ExitCritical(
        interrupt_state
    );

    return enabled;
}

bool MotionControl_SetMaxWheelTorque(
    float max_wheel_torque_mnm
)
{
    if (max_wheel_torque_mnm < 0.0F)
    {
        return false;
    }

    const uint32_t interrupt_state =
        MotionControl_EnterCritical();

    parameters.max_wheel_torque_mnm =
        max_wheel_torque_mnm;

    status.max_wheel_torque_mnm =
        max_wheel_torque_mnm;

    status.left_torque_command_mnm =
        MotionControl_Clamp(
            status.left_torque_command_mnm,
            -parameters.max_wheel_torque_mnm,
            parameters.max_wheel_torque_mnm
        );

    status.right_torque_command_mnm =
        MotionControl_Clamp(
            status.right_torque_command_mnm,
            -parameters.max_wheel_torque_mnm,
            parameters.max_wheel_torque_mnm
        );

    MotionControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool MotionControl_SetGainScale(
    float gain_scale
)
{
    if (gain_scale < 0.0F)
    {
        return false;
    }

    const uint32_t interrupt_state =
        MotionControl_EnterCritical();

    parameters.motion_gain_scale =
        gain_scale;

    status.motion_gain_scale =
        gain_scale;

    MotionControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool MotionControl_SetCommand(
    float forward_velocity_command_mps,
    float yaw_rate_command_rads
)
{
    const uint32_t now_ms =
        HAL_GetTick();
    const uint32_t interrupt_state =
        MotionControl_EnterCritical();

    status.forward_velocity_command_mps =
        forward_velocity_command_mps;

    status.yaw_rate_command_rads =
        yaw_rate_command_rads;

    status.command_update_count++;
    status.command_age_ms = 0U;
    last_command_ms =
        now_ms;

    MotionControl_ExitCritical(
        interrupt_state
    );

    return true;
}

bool MotionControl_SetGain(
    uint32_t row,
    uint32_t column,
    float gain
)
{
    if (
        row >= 2U ||
        column >= MOTION_CONTROL_AUGMENTED_COUNT
    )
    {
        return false;
    }

    const uint32_t interrupt_state =
        MotionControl_EnterCritical();

    parameters.rslqr_gain_nm_s[row][column] =
        gain;

    MotionControl_ExitCritical(
        interrupt_state
    );

    return true;
}

void MotionControl_Update(
    const RobotState *state
)
{
    float augmented_state[MOTION_CONTROL_AUGMENTED_COUNT];
    MotionControlTorque candidate;
    const float sample_period_s =
        1.0F /
        (float)parameters.motion_controller_rate_hz;

    if (!status.enabled)
    {
        return;
    }

    MotionControl_ApplyCommandWatchdog();

    if (
        state == NULL ||
        !state->valid
    )
    {
        if (
            state != NULL &&
            state->imu_stale
        )
        {
            MotionControl_EnterFault(
                MOTION_CONTROL_FAULT_IMU_STALE,
                state
            );
        }
        else
        {
            MotionControl_EnterFault(
                MOTION_CONTROL_FAULT_STATE_INVALID,
                state
            );
        }

        return;
    }

    if (
        state->pitch_rad >
        parameters.fall_angle_rad ||
        state->pitch_rad <
        -parameters.fall_angle_rad
    )
    {
        MotionControl_EnterFault(
            MOTION_CONTROL_FAULT_FALL,
            state
        );
        return;
    }

    augmented_state[0] =
        state->forward_velocity_mps -
        status.forward_velocity_command_mps;

    augmented_state[1] =
        state->yaw_rate_rads -
        status.yaw_rate_command_rads;

    augmented_state[2] =
        state->forward_acceleration_mps2;

    augmented_state[3] =
        state->pitch_rate_rads;

    augmented_state[4] =
        state->pitch_acceleration_rads2;

    augmented_state[5] =
        state->yaw_acceleration_rads2;

    status.left_torque_rate_mnm_s = 0.0F;
    status.right_torque_rate_mnm_s = 0.0F;

    for (uint32_t index = 0U; index < MOTION_CONTROL_AUGMENTED_COUNT; index++)
    {
        status.left_torque_rate_mnm_s -=
            parameters.motion_gain_scale *
            parameters.rslqr_gain_nm_s[0][index] *
            MOTION_CONTROL_MNM_PER_NM *
            augmented_state[index];

        status.right_torque_rate_mnm_s -=
            parameters.motion_gain_scale *
            parameters.rslqr_gain_nm_s[1][index] *
            MOTION_CONTROL_MNM_PER_NM *
            augmented_state[index];
    }

    candidate.left_torque_mnm =
        MotionControl_IntegrateTorque(
            status.left_torque_command_mnm,
            status.left_torque_rate_mnm_s,
            sample_period_s
        );

    candidate.right_torque_mnm =
        MotionControl_IntegrateTorque(
            status.right_torque_command_mnm,
            status.right_torque_rate_mnm_s,
            sample_period_s
        );

    status.left_torque_command_mnm =
        candidate.left_torque_mnm;

    status.right_torque_command_mnm =
        candidate.right_torque_mnm;

    if (!Actuator_SetWheelTorqueReferences(
            MotionControl_TorqueToMillinewtonMeters(
                status.left_torque_command_mnm
            ),
            MotionControl_TorqueToMillinewtonMeters(
                status.right_torque_command_mnm
            )
        ))
    {
        MotionControl_EnterFault(
            MOTION_CONTROL_FAULT_ACTUATOR,
            state
        );
    }
}

void MotionControl_GetStatus(
    MotionControlStatus *output
)
{
    if (output == NULL)
    {
        return;
    }

    const uint32_t interrupt_state =
        MotionControl_EnterCritical();

    MotionControl_UpdateCommandAge();

    *output = status;

    MotionControl_ExitCritical(
        interrupt_state
    );
}

const char *MotionControl_FaultName(
    MotionControlFault fault
)
{
    switch (fault)
    {
        case MOTION_CONTROL_FAULT_NONE:
            return "none";

        case MOTION_CONTROL_FAULT_STATE_INVALID:
            return "state_invalid";

        case MOTION_CONTROL_FAULT_IMU_STALE:
            return "imu_stale";

        case MOTION_CONTROL_FAULT_FALL:
            return "fall";

        case MOTION_CONTROL_FAULT_ACTUATOR:
            return "actuator";

        default:
            return "unknown";
    }
}

static void MotionControl_ResetControllerState(void)
{
    status.left_torque_command_mnm = 0.0F;
    status.right_torque_command_mnm = 0.0F;
    status.left_torque_rate_mnm_s = 0.0F;
    status.right_torque_rate_mnm_s = 0.0F;
}

static void MotionControl_ResetCommandState(void)
{
    status.forward_velocity_command_mps = 0.0F;
    status.yaw_rate_command_rads = 0.0F;
    status.command_age_ms = 0U;
    last_command_ms =
        HAL_GetTick();
}

static void MotionControl_UpdateCommandAge(void)
{
    status.command_age_ms =
        HAL_GetTick() -
        last_command_ms;
}

static void MotionControl_ApplyCommandWatchdog(void)
{
    const uint32_t interrupt_state =
        MotionControl_EnterCritical();

    MotionControl_UpdateCommandAge();

    if (
        status.command_age_ms >
        parameters.motion_command_timeout_ms
    )
    {
        status.forward_velocity_command_mps = 0.0F;
        status.yaw_rate_command_rads = 0.0F;
    }

    MotionControl_ExitCritical(
        interrupt_state
    );
}

static void MotionControl_ClearFaultState(void)
{
    status.fault_active = false;
    status.state_invalid = false;
    status.fall_detected = false;
    status.fault =
        MOTION_CONTROL_FAULT_NONE;
    MotionControl_ClearFaultContext();
}

static void MotionControl_ClearFaultContext(void)
{
    status.fault_state.valid = false;
    status.fault_state.imu_valid = false;
    status.fault_state.imu_stale = false;
    status.fault_state.encoder_valid = false;
    status.fault_state.update_count = 0U;
    status.fault_state.orientation_update_count = 0U;
    status.fault_state.gyroscope_update_count = 0U;
    status.fault_state.orientation_age_ms = 0U;
    status.fault_state.gyroscope_age_ms = 0U;
    status.fault_state.forward_velocity_mps = 0.0F;
    status.fault_state.pitch_rad = 0.0F;
    status.fault_state.pitch_rate_rads = 0.0F;
    status.fault_state.yaw_rate_rads = 0.0F;
    status.fault_state.forward_acceleration_mps2 = 0.0F;
    status.fault_state.pitch_acceleration_rads2 = 0.0F;
    status.fault_state.yaw_acceleration_rads2 = 0.0F;
}

static void MotionControl_EnterFault(
    MotionControlFault fault,
    const RobotState *fault_state
)
{
    const uint32_t interrupt_state =
        MotionControl_EnterCritical();

    status.enabled = false;
    status.fault_active = true;
    status.state_invalid =
        fault == MOTION_CONTROL_FAULT_STATE_INVALID ||
        fault == MOTION_CONTROL_FAULT_IMU_STALE;
    status.fall_detected =
        fault == MOTION_CONTROL_FAULT_FALL;
    status.fault =
        fault;
    if (fault_state != NULL)
    {
        status.fault_state =
            *fault_state;
    }

    MotionControl_ResetCommandState();
    MotionControl_ResetControllerState();

    MotionControl_ExitCritical(
        interrupt_state
    );

    Actuator_Disable();
}

static float MotionControl_Clamp(
    float value,
    float minimum,
    float maximum
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

static float MotionControl_IntegrateTorque(
    float current_torque_mnm,
    float torque_rate_mnm_s,
    float sample_period_s
)
{
    const float limited_current_torque_mnm =
        MotionControl_Clamp(
            current_torque_mnm,
            -parameters.max_wheel_torque_mnm,
            parameters.max_wheel_torque_mnm
        );

    const float requested_torque_mnm =
        limited_current_torque_mnm +
        torque_rate_mnm_s *
        sample_period_s;

    if (
        limited_current_torque_mnm >=
        parameters.max_wheel_torque_mnm &&
        torque_rate_mnm_s > 0.0F
    )
    {
        return parameters.max_wheel_torque_mnm;
    }

    if (
        limited_current_torque_mnm <=
        -parameters.max_wheel_torque_mnm &&
        torque_rate_mnm_s < 0.0F
    )
    {
        return -parameters.max_wheel_torque_mnm;
    }

    return MotionControl_Clamp(
        requested_torque_mnm,
        -parameters.max_wheel_torque_mnm,
        parameters.max_wheel_torque_mnm
    );
}

static int32_t MotionControl_TorqueToMillinewtonMeters(
    float torque_mnm
)
{
    if (torque_mnm >= 0.0F)
    {
        return (int32_t)(torque_mnm + 0.5F);
    }

    return (int32_t)(torque_mnm - 0.5F);
}

static uint32_t MotionControl_EnterCritical(void)
{
    const uint32_t interrupt_state =
        __get_PRIMASK();

    __disable_irq();

    return interrupt_state;
}

static void MotionControl_ExitCritical(uint32_t interrupt_state)
{
    __set_PRIMASK(
        interrupt_state
    );
}
