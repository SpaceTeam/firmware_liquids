#ifndef FIRMWARE_LIQUIDS_RINGBUF_H
#define FIRMWARE_LIQUIDS_RINGBUF_H

#include <atomic>

template<typename T, uint8_t SIZE>
class RingBuf {
public:
    void push(T value) {
        if (bufSize == SIZE) {
            return; // The buffer is already full.
        }
        buf[bufWriteIndex] = value;
        bufWriteIndex = (bufWriteIndex + 1) % SIZE;
        ++bufSize;
    }

    T pop() {
        T value = buf[bufReadIndex];
        bufReadIndex = (bufReadIndex + 1) % SIZE;
        --bufSize;
        return value;
    }

    const T& peek() const {
        return buf[bufReadIndex];
    }

    [[nodiscard]] bool isEmpty() const {
        return bufSize == 0;
    }

    [[nodiscard]] uint8_t length() const {
        return bufSize;
    }

private:
    T buf[SIZE] = {};
    volatile uint8_t bufReadIndex = 0;
    volatile uint8_t bufWriteIndex = 0;
    std::atomic<uint8_t> bufSize = 0;
};


#endif // FIRMWARE_LIQUIDS_RINGBUF_H
