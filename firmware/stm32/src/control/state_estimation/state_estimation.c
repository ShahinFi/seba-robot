#include "state_estimation.h"

#include "control/control_parameters.h"
#include "control/motion_control/motion_control.h"
#include "encoder/encoder.h"
#include "imu/imu.h"

#include "stm32g4xx_hal.h"

#include <stddef.h>

#define STATE_ESTIMATION_TWO_PI  6.28318530718F
#define STATE_ESTIMATION_TIMER_HZ      1000000U

/*
 * The estimator runs from TIM15 at the same rate as the
 * motion controller. This keeps state updates and control
 * updates aligned in one interrupt-driven path.
 */
static ControlParameters parameters;
static RobotState state;
static int64_t previous_left_position;
static int64_t previous_right_position;
static float previous_forward_velocity_mps;
static float previous_pitch_rate_rads;
static float previous_yaw_rate_rads;
static float velocity_filter_alpha;
static float derivative_filter_alpha;

static void StateEstimation_TimerInit(void);
static void StateEstimation_Update(void);

static float StateEstimation_Filter(
    float previous_output,
    float input,
    float alpha
);

static float StateEstimation_FilterAlpha(
    float cutoff_hz,
    float sample_period_s
);

static uint32_t StateEstimation_EnterCritical(void);
static void StateEstimation_ExitCritical(uint32_t interrupt_state);

void StateEstimation_Init(void)
{
    float sample_period_s;

    ControlParameters_Get(
        &parameters
    );

    sample_period_s =
        1.0F /
        (float)parameters.state_estimator_rate_hz;

    velocity_filter_alpha =
        StateEstimation_FilterAlpha(
            parameters.velocity_filter_cutoff_hz,
            sample_period_s
        );

    derivative_filter_alpha =
        StateEstimation_FilterAlpha(
            parameters.derivative_filter_cutoff_hz,
            sample_period_s
        );

    Encoder_Update();

    previous_left_position =
        Encoder_GetLeftPosition();

    previous_right_position =
        Encoder_GetRightPosition();

    previous_forward_velocity_mps = 0.0F;
    previous_pitch_rate_rads = 0.0F;
    previous_yaw_rate_rads = 0.0F;

    state.valid = false;
    state.imu_valid = false;
    state.imu_stale = false;
    state.encoder_valid = false;
    state.update_count = 0U;
    state.orientation_update_count = 0U;
    state.gyroscope_update_count = 0U;
    state.orientation_age_ms = UINT32_MAX;
    state.gyroscope_age_ms = UINT32_MAX;
    state.forward_velocity_mps = 0.0F;
    state.pitch_rad = 0.0F;
    state.pitch_rate_rads = 0.0F;
    state.yaw_rate_rads = 0.0F;
    state.forward_acceleration_mps2 = 0.0F;
    state.pitch_acceleration_rads2 = 0.0F;
    state.yaw_acceleration_rads2 = 0.0F;

    StateEstimation_TimerInit();
}

void StateEstimation_Reset(void)
{
    const uint32_t interrupt_state =
        StateEstimation_EnterCritical();

    /*
     * Reset only encoder-derived motion state. Keep the current
     * IMU rates as derivative references so reset does not create
     * a synthetic angular-acceleration spike.
     */
    Encoder_Update();

    previous_left_position =
        Encoder_GetLeftPosition();

    previous_right_position =
        Encoder_GetRightPosition();

    previous_forward_velocity_mps = 0.0F;
    previous_pitch_rate_rads =
        state.pitch_rate_rads;
    previous_yaw_rate_rads =
        state.yaw_rate_rads;

    state.forward_velocity_mps = 0.0F;
    state.forward_acceleration_mps2 = 0.0F;
    state.pitch_acceleration_rads2 = 0.0F;
    state.yaw_acceleration_rads2 = 0.0F;

    StateEstimation_ExitCritical(
        interrupt_state
    );
}

void StateEstimation_GetState(
    RobotState *output
)
{
    uint32_t interrupt_state;

    if (output == NULL)
    {
        return;
    }

    interrupt_state =
        StateEstimation_EnterCritical();

    *output = state;

    StateEstimation_ExitCritical(
        interrupt_state
    );
}

static void StateEstimation_Update(void)
{
    IMUData imu;
    int64_t left_position;
    int64_t right_position;
    int64_t left_delta;
    int64_t right_delta;
    float left_velocity_mps;
    float right_velocity_mps;
    float raw_forward_velocity_mps;
    float raw_forward_acceleration_mps2;
    float raw_pitch_acceleration_rads2;
    float raw_yaw_acceleration_rads2;
    bool imu_available;
    bool orientation_fresh;
    bool gyroscope_fresh;
    bool imu_valid;
    const float sample_period_s =
        1.0F /
        (float)parameters.state_estimator_rate_hz;

    const float meters_per_count =
        (
            STATE_ESTIMATION_TWO_PI *
            parameters.wheel_radius_m
        ) /
        (float)parameters.encoder_counts_per_wheel_rev;

    /*
     * IMU freshness is checked independently for orientation
     * and gyroscope reports because the BNO085 publishes them
     * at different rates.
     */
    imu_available =
        IMU_GetLatest(
            &imu
        );

    orientation_fresh =
        imu_available &&
        imu.orientation_valid &&
        imu.orientation_age_ms <=
        parameters.imu_orientation_timeout_ms;

    gyroscope_fresh =
        imu_available &&
        imu.gyroscope_valid &&
        imu.gyroscope_age_ms <=
        parameters.imu_gyroscope_timeout_ms;

    imu_valid =
        orientation_fresh &&
        gyroscope_fresh;

    Encoder_Update();

    left_position =
        Encoder_GetLeftPosition();

    right_position =
        Encoder_GetRightPosition();

    left_delta =
        left_position -
        previous_left_position;

    right_delta =
        right_position -
        previous_right_position;

    previous_left_position =
        left_position;

    previous_right_position =
        right_position;

    left_velocity_mps =
        (
            (float)left_delta *
            meters_per_count
        ) /
        sample_period_s;

    right_velocity_mps =
        (
            (float)right_delta *
            meters_per_count
        ) /
        sample_period_s;

    /*
     * Forward velocity uses the average wheel speed. Wheel signs
     * are already normalized inside the encoder module.
     */
    raw_forward_velocity_mps =
        (
            left_velocity_mps +
            right_velocity_mps
        ) *
        0.5F;

    state.forward_velocity_mps =
        StateEstimation_Filter(
            state.forward_velocity_mps,
            raw_forward_velocity_mps,
            velocity_filter_alpha
        );

    raw_forward_acceleration_mps2 =
        (
            state.forward_velocity_mps -
            previous_forward_velocity_mps
        ) /
        sample_period_s;

    state.forward_acceleration_mps2 =
        StateEstimation_Filter(
            state.forward_acceleration_mps2,
            raw_forward_acceleration_mps2,
            derivative_filter_alpha
        );

    previous_forward_velocity_mps =
        state.forward_velocity_mps;

    state.orientation_update_count =
        imu_available
            ? imu.orientation_update_count
            : 0U;

    state.gyroscope_update_count =
        imu_available
            ? imu.gyroscope_update_count
            : 0U;

    state.orientation_age_ms =
        imu_available
            ? imu.orientation_age_ms
            : UINT32_MAX;

    state.gyroscope_age_ms =
        imu_available
            ? imu.gyroscope_age_ms
            : UINT32_MAX;

    state.imu_stale =
        imu_available &&
        (
            (
                imu.orientation_valid &&
                !orientation_fresh
            ) ||
            (
                imu.gyroscope_valid &&
                !gyroscope_fresh
            )
        );

    if (imu_valid)
    {
        /*
         * Pitch and yaw rates come directly from the IMU module.
         * Their derivatives are filtered here before entering the
         * augmented controller state.
         */
        state.pitch_rad =
            imu.robot_pitch_rad;

        state.pitch_rate_rads =
            imu.robot_pitch_rate_rads;

        state.yaw_rate_rads =
            imu.robot_yaw_rate_rads;

        raw_pitch_acceleration_rads2 =
            (
                state.pitch_rate_rads -
                previous_pitch_rate_rads
            ) /
            sample_period_s;

        raw_yaw_acceleration_rads2 =
            (
                state.yaw_rate_rads -
                previous_yaw_rate_rads
            ) /
            sample_period_s;

        state.pitch_acceleration_rads2 =
            StateEstimation_Filter(
                state.pitch_acceleration_rads2,
                raw_pitch_acceleration_rads2,
                derivative_filter_alpha
            );

        state.yaw_acceleration_rads2 =
            StateEstimation_Filter(
                state.yaw_acceleration_rads2,
                raw_yaw_acceleration_rads2,
                derivative_filter_alpha
            );

        previous_pitch_rate_rads =
            state.pitch_rate_rads;

        previous_yaw_rate_rads =
            state.yaw_rate_rads;
    }

    state.imu_valid =
        imu_valid;

    state.encoder_valid = true;
    state.valid =
        state.imu_valid &&
        state.encoder_valid;

    state.update_count++;

    /*
     * Motion control consumes the completed state snapshot from
     * this same estimator tick.
     */
    MotionControl_Update(
        &state
    );
}

static float StateEstimation_Filter(
    float previous_output,
    float input,
    float alpha
)
{
    return
        previous_output +
        alpha *
        (
            input -
            previous_output
        );
}

static float StateEstimation_FilterAlpha(
    float cutoff_hz,
    float sample_period_s
)
{
    float time_constant_s;

    if (
        cutoff_hz <= 0.0F ||
        sample_period_s <= 0.0F
    )
    {
        return 1.0F;
    }

    time_constant_s =
        1.0F /
        (
            STATE_ESTIMATION_TWO_PI *
            cutoff_hz
        );

    /*
     * First-order low-pass filter coefficient for the fixed
     * estimator sample period.
     */
    return
        sample_period_s /
        (
            time_constant_s +
            sample_period_s
        );
}

static void StateEstimation_TimerInit(void)
{
    __HAL_RCC_TIM15_FORCE_RESET();
    __HAL_RCC_TIM15_RELEASE_RESET();
    __HAL_RCC_TIM15_CLK_ENABLE();

    TIM15->CR1 = 0U;
    TIM15->CR2 = 0U;
    TIM15->DIER = 0U;

    /*
     * TIM15 is a 16-bit timer, so the 200 Hz estimator tick
     * uses a 1 MHz timer base and a 5000-count period.
     */
    TIM15->PSC =
        (
            SystemCoreClock /
            STATE_ESTIMATION_TIMER_HZ
        ) -
        1U;
    TIM15->ARR =
        (
            STATE_ESTIMATION_TIMER_HZ /
            parameters.state_estimator_rate_hz
        ) -
        1U;
    TIM15->EGR = TIM_EGR_UG;
    TIM15->SR = 0U;
    TIM15->DIER = TIM_DIER_UIE;

    HAL_NVIC_SetPriority(
        TIM1_BRK_TIM15_IRQn,
        7U,
        0U
    );

    HAL_NVIC_EnableIRQ(
        TIM1_BRK_TIM15_IRQn
    );

    TIM15->CR1 =
        TIM_CR1_CEN;
}

static uint32_t StateEstimation_EnterCritical(void)
{
    const uint32_t interrupt_state =
        __get_PRIMASK();

    __disable_irq();

    return interrupt_state;
}

static void StateEstimation_ExitCritical(uint32_t interrupt_state)
{
    __set_PRIMASK(
        interrupt_state
    );
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
    if ((TIM15->SR & TIM_SR_UIF) == 0U)
    {
        return;
    }

    TIM15->SR &= (uint32_t)~TIM_SR_UIF;

    StateEstimation_Update();
}
