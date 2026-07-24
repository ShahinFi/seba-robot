#include "event_stream.h"

#include "communication/state_event_format.h"
#include "control/motion_control/motion_control.h"
#include "control/state_estimation/state_estimation.h"
#include "serial/serial.h"

static bool previous_motion_fault_active;
static MotionControlFault previous_motion_fault =
    MOTION_CONTROL_FAULT_NONE;

static void EventStream_UpdateMotionFault(void);

void EventStream_Update(void)
{
    EventStream_UpdateMotionFault();
}

static void EventStream_UpdateMotionFault(void)
{
    RobotState state;
    MotionControlStatus status;

    StateEstimation_GetState(
        &state
    );

    MotionControl_GetStatus(
        &status
    );

    if (
        status.fault_active ==
        previous_motion_fault_active &&
        status.fault ==
        previous_motion_fault
    )
    {
        return;
    }

    /*
     * Events are edge-triggered so the Pi log shows state
     * transitions without being flooded by repeated telemetry.
     */
    previous_motion_fault_active =
        status.fault_active;
    previous_motion_fault =
        status.fault;

    if (status.fault_active)
    {
        Serial_Write("EVT motion fault reason=");
        Serial_Write(
            MotionControl_FaultName(status.fault)
        );
        StateEventFormat_WriteState(
            &status.fault_state
        );
        Serial_WriteLine("");
        return;
    }

    Serial_Write(
        "EVT motion fault-cleared reason=none"
    );
    StateEventFormat_WriteState(
        &state
    );
    Serial_WriteLine("");
}
