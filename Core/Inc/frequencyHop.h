#pragma once

#include <cstdint>
#include <array>

struct HopChannel
{
    uint32_t frequency;
    uint8_t dwellPackets;
};

constexpr std::array<HopChannel, 8> HopTable =
{{
	{863000000UL, 1},
	{864000000UL, 1},
	{865000000UL, 1},
	{866000000UL, 1},
	{867000000UL, 1},
	{868000000UL, 1},
	{869000000UL, 1},
	{870000000UL, 1}
}};
