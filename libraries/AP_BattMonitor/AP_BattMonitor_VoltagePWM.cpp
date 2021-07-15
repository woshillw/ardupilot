#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_VoltagePWM.h"

extern const AP_HAL::HAL &hal;

/// Constructor
AP_BattMonitor_VoltagePWM::AP_BattMonitor_VoltagePWM(AP_BattMonitor &mon,
                                                     AP_BattMonitor::BattMonitor_State &mon_state,
                                                     AP_BattMonitor_Params &params) : AP_BattMonitor_Backend(mon, mon_state, params)
{
    _state.voltage = 1.0; // show a fixed voltage of 1v

    // need to add check
    _state.healthy = false;
}

// read - read the voltage and current
void AP_BattMonitor_VoltagePWM::read()
{
    if (!pwm_source.set_pin(_params._volt_pin, "VoltagePWM"))
    {
        // hal.console->printf("pulse_width=----\n");
        _state.healthy = false;
        return;
    }
    uint16_t pulse_width = pwm_source.get_pwm_us();

    _state.voltage = (float)pulse_width * 5.0f * 0.001f;
    _state.healthy = true;
    _state.last_time_micros = AP_HAL::micros();

    // hal.console->printf("pulse_width=%d,voltage=%f\n", pulse_width, _state.voltage);
}
