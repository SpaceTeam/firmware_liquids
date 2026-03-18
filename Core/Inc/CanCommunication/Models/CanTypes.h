//
// Created by tobias on 04.03.26.
//

#ifndef CANTYPES_H
#define CANTYPES_H
#include <cstdint>

typedef struct {
    uint16_t destinationId: 5;
    uint16_t senderId: 5;
    uint16_t priority: 1;
} Can_Identifier_t;
#endif //CANTYPES_H
