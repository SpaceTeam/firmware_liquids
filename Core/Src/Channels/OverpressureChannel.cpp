#include "../../Inc/Channels/OverpressureChannel.h"


int OverpressureChannel::init()
{
    if (!flash.readConfig())
        return -1;

    threshold = flash.readConfigReg(Config::OVERPRESSURE_THRESHOLD);
    operatingMode = static_cast<OverpressureOperatingMode>(flash.readConfigReg(Config::OVERPRESSURE_OPERATING_MODE));

    return 0;
}

int OverpressureChannel::exec()
{
    if (operatingMode == OverpressureOperatingMode::DISABLED)
    {
        return 0;
    }

    uint32_t pressure = getPressureCallback();
    bool hasOverpressure =
        operatingMode == OverpressureOperatingMode::GREATER ? pressure > threshold : pressure < threshold;

    if (hasOverpressure)
    {
        if (overpressureBeginTick.has_value())
        {
            auto delay = STRHAL_Systick_GetTick() - overpressureBeginTick.value();
            if (delay > OVERPRESSURE_REACTION_DELAY)
            {
                ventCallback();
                overpressureBeginTick = std::nullopt;
            }
        }
        else
        {
            overpressureBeginTick = STRHAL_Systick_GetTick();
        }
    }
    else
    {
        overpressureBeginTick = std::nullopt;
    }
    return 0;
}


int OverpressureChannel::getVariable(uint8_t variableId, int32_t& data) const
{

}
