//
// Created by tobias on 04.03.26.
//

#ifndef CANPROCESS_H
#define CANPROCESS_H
#include <cstdint>

#include "CanField.h"
#include "CanProcessStatus.h"

class CanProcess {
    public:
    CanProcess(uint8_t processType, uint8_t processResponseType, uint8_t fieldId = 0);

    const CanProcessStatus getStatus();
    bool setStatus(CanProcessStatus status);

    const uint8_t getProcessType();

    const uint8_t getProcessResponse();

    const uint8_t getFieldId();

    private:
    CanProcessStatus status;
    uint8_t processType;
    uint8_t processResponseType;
    uint8_t fieldId;
};
#endif //CANPROCESS_H
