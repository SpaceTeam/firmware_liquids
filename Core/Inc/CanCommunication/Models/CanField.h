//
// Created by tobias on 04.03.26.
//

#ifndef CANFIELD_H
#define CANFIELD_H
#include <cstdint>

class CanField {
    public:
    CanField(uint8_t field_id, uint8_t field_type, const uint8_t* field_name, bool is_internal, void* field_data = nullptr);

    const uint8_t getFieldId() const;
    const uint8_t getFieldType() const;
    const uint8_t* getFieldName() const;

    bool changeLockStatus();
    const bool getLockStatus() const;

    void* getFieldDataPtr();

    private:
    uint8_t field_id = 0;
    int8_t field_name[63];
    uint8_t field_typ = 0;
    bool is_locked = false;
    bool is_internal = false;
    void* field_data_ptr = nullptr;
};
#endif //CANFIELD_H
