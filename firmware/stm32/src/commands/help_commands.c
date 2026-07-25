#include "help_commands.h"
#include "serial/serial.h"

CommandResult HelpCommands_Print(void)
{
    Serial_WriteLine("Commands:");

    Serial_WriteLine(
        "  motor left <-100...100>"
    );

    Serial_WriteLine(
        "  motor right <-100...100>"
    );

    Serial_WriteLine(
        "  motor stop"
    );

    Serial_WriteLine(
        "  encoder read"
    );

    Serial_WriteLine(
        "  encoder reset"
    );

    Serial_WriteLine(
        "  current read"
    );

    Serial_WriteLine(
        "  current scope <left|right> <samples>"
    );

    Serial_WriteLine(
        "  actuator left <mA>"
    );

    Serial_WriteLine(
        "  actuator right <mA>"
    );

    Serial_WriteLine(
        "  actuator both <left_mA> <right_mA>"
    );

    Serial_WriteLine(
        "  actuator torque <left_mNm> <right_mNm>"
    );

    Serial_WriteLine(
        "  actuator stop"
    );

    Serial_WriteLine(
        "  actuator status"
    );

    Serial_WriteLine(
        "  actuator config"
    );

    Serial_WriteLine(
        "  actuator config <kp> <ki> <max_mA> <max_pwm> <int_mV> <ktw>"
    );

    Serial_WriteLine(
        "  actuator kp <mV_per_A>"
    );

    Serial_WriteLine(
        "  actuator ki <mV_per_A_s>"
    );

    Serial_WriteLine(
        "  actuator battery <mV>"
    );

    Serial_WriteLine(
        "  actuator max-current <mA>"
    );

    Serial_WriteLine(
        "  actuator max-pwm <percent>"
    );

    Serial_WriteLine(
        "  actuator integral-limit <mV>"
    );

    Serial_WriteLine(
        "  actuator period <ms>"
    );

    Serial_WriteLine(
        "  actuator torque-constant <mNm_per_A>"
    );

    Serial_WriteLine(
        "  state read"
    );

    Serial_WriteLine(
        "  balance start"
    );

    Serial_WriteLine(
        "  balance stop"
    );

    Serial_WriteLine(
        "  balance status"
    );

    Serial_WriteLine(
        "  balance max-torque <mNm>"
    );

    Serial_WriteLine(
        "  balance gain-scale <percent>"
    );

    Serial_WriteLine(
        "  balance command <v_mps> <yaw_rate_rads>"
    );

    Serial_WriteLine(
        "  balance gain <left|right> <0...5> <value>"
    );

    Serial_WriteLine(
        "  balance config <gain_percent> <max_mNm> <12 gains>"
    );

    Serial_WriteLine(
        "  telemetry read"
    );

    Serial_WriteLine(
        "  system reset"
    );

    Serial_WriteLine(
        "  imu read"
    );

    Serial_WriteLine(
        "  help"
    );

    Serial_WriteLine("");

    Serial_WriteLine(
        "Positive motor command = forward"
    );

    Serial_WriteLine(
        "Negative motor command = reverse"
    );

    Serial_WriteLine(
        "Zero = stop that motor"
    );

    return COMMAND_RESULT_OK;
}
