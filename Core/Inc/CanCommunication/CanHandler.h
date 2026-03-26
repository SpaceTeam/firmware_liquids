//
// Created by tobias on 04.03.26.
//

#ifndef CANHANDLER_H
#define CANHANDLER_H
#include <cstdint>
#include <map>
#include <vector>

#include "RingBuf.h"
#include "Models/CanField.h"
#include "Models/CanHandlerStatus.h"
#include "Models/CanProcess.h"
#include "Models/CanTypes.h"

class CanHandler {
    public:
    CanHandler(const CanHandler &canHandler) = delete;
    CanHandler(const CanHandler &&canHandler) = delete;
    CanHandler& operator=(const CanHandler &canHandler) = delete;
    CanHandler& operator=(const CanHandler &&canHandler) = delete;

    static const CanHandler& getInstance(uint32_t nodeId);

    bool registerCanField(const CanField& canField, uint8_t telemetryGroup = -1);

    bool createTelemetryGroup(uint8_t groupId);
    bool addFieldToTelemetryGroup(uint8_t groupId, uint8_t fieldId);

    CanHandlerStatus getStatus();

    private:
    explicit CanHandler(uint32_t nodeId);

    void sendMessage(Can_Identifier_t header, uint8_t* data, uint32_t n);
    void messageCallback(uint32_t id, uint8_t *data, uint32_t n);

    static CanHandler* instance;

    uint8_t nodeId;
    CanHandlerStatus status;


    std::tuple<uint8_t, CanField> canFields[16];
    std::tuple<int8_t, uint8_t> fieldNameIdMap[16];
    std::tuple<uint8_t, CanProcess> processes[16];
    // TODO: replace with array of TelemetryGroupDefinition
    std::tuple<uint8_t, uint8_t[62]> telemetryGroups;
    RingBuf<std::tuple<Can_Identifier_t, uint8_t[64]>, 32> buffer;
};

#endif //CANHANDLER_H
