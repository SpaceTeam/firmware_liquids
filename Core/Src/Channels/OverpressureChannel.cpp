#include "../../Inc/Channels/OverpressureChannel.h"

#include "channels/overpressure_channel_def.h"

int OverpressureChannel::init() {
    if (!flash.readConfig())
        return -1;

    threshold = flash.readConfigReg(Config::OVERPRESSURE_THRESHOLD);
    operatingMode = static_cast<OverpressureOperatingMode>(flash.readConfigReg(Config::OVERPRESSURE_OPERATING_MODE));

    return 0;
}

int OverpressureChannel::exec() {
    if (operatingMode == OverpressureOperatingMode::DISABLED) {
        return 0;
    }

    uint32_t pressure = getPressureCallback();
    bool hasOverpressure =
            operatingMode == OverpressureOperatingMode::GREATER ? pressure > threshold : pressure < threshold;

    if (hasOverpressure) {
        if (overpressureBeginTick.has_value()) {
            auto delay = STRHAL_Systick_GetTick() - overpressureBeginTick.value();
            if (delay > OVERPRESSURE_REACTION_DELAY) {
                ventCallback();
                overpressureBeginTick = std::nullopt;
            }
        } else {
            overpressureBeginTick = STRHAL_Systick_GetTick();
        }
    } else {
        overpressureBeginTick = std::nullopt;
    }
    return 0;
}


int OverpressureChannel::getVariable(uint8_t variableId, int32_t &data) const {
    switch (variableId) {
        case OVERPRESSURE_OPERATING_MODE: {
            data = static_cast<int32_t>(operatingMode);
            return 0;
        }
        case OVERPRESSURE_THRESHOLD: {
            data = static_cast<int32_t>(threshold);
            return 0;
        }
        default: {
            return -1;
        }
    }
}

int OverpressureChannel::setVariable(uint8_t variableId, int32_t data) {
    if (data < 0) {
        return -1;
    }
    switch (variableId) {
        case OVERPRESSURE_OPERATING_MODE: {
            if (data >= static_cast<uint32_t>(OverpressureOperatingMode::MODE_COUNT)) {
                // invalid value
                return -1;
            }
            operatingMode = static_cast<OverpressureOperatingMode>(data);
            flash.writeConfigReg(Config::OVERPRESSURE_OPERATING_MODE, data);
            return 0;
        }
        case OVERPRESSURE_THRESHOLD: {
            threshold = data;
            flash.writeConfigReg(Config::OVERPRESSURE_THRESHOLD, data);
            return 0;
        }
        default: {
            return -1;
        }
    }
}
