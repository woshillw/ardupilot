#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_VoltagePWM : public AP_BattMonitor_Backend
{
public:
    /// Constructor
    AP_BattMonitor_VoltagePWM(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return false; }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return false; }

    void init(void) override {}

protected:
    AP_HAL::PWMSource pwm_source;
};
