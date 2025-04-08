#ifndef OVERPRESSURECHANNEL_H
#define OVERPRESSURECHANNEL_H
#include <functional>
#include <optional>

#include "AbstractChannel.h"
#include "Modules/W25Qxx_Flash.h"

enum class OverpressureOperatingMode : uint32_t { GREATER, LESS, DISABLED, MODE_COUNT };

class OverpressureChannel final : public AbstractChannel {
public:
    OverpressureChannel(const uint8_t id, const std::function<void()> &ventCallback,
                        const std::function<uint32_t()> &getPressureCallback, W25Qxx_Flash &flash,
                        const uint32_t refreshDivider) :
        AbstractChannel(CHANNEL_TYPE_OVERPRESSURE, id, refreshDivider), flash(flash), ventCallback(ventCallback),
        getPressureCallback(getPressureCallback) {}

    int init() override;

    int reset() override { return 0; }

    int exec() override;

    int getSensorData(uint8_t *data, uint8_t &n) override { return 0; }

protected:
    int setVariable(uint8_t variableId, int32_t data) override;

    int getVariable(uint8_t variableId, int32_t &data) const override;

private:
    static constexpr uint32_t OVERPRESSURE_REACTION_DELAY = 1;

    W25Qxx_Flash &flash;

    std::function<void()> ventCallback;
    std::function<uint32_t()> getPressureCallback;
    uint32_t threshold = 0;
    OverpressureOperatingMode operatingMode = OverpressureOperatingMode::GREATER;
    std::optional<uint64_t> overpressureBeginTick = std::nullopt;
};


#endif // OVERPRESSURECHANNEL_H
