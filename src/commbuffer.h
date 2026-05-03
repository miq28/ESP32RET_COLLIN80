#pragma once
#include <Arduino.h>
#include "config.h"
#include "esp32_can.h"

#define CB_SIZE 15360

class CommBuffer
{
public:
    CommBuffer();

    uint8_t *getBufferedBytes();
    size_t numAvailableBytes();
    size_t getLinearSize(); // NEW
    void consume(size_t n);

    void clearBufferedBytes();

    // existing API (unchanged)
    void sendByteToBuffer(uint8_t b);

    // restore missing APIs
    void sendString(const char *str);
    void sendString(const String &str);

    // GVRET compatibility (keep signature)
    void sendFrameToBuffer(CAN_FRAME &frame, int whichBus);
    void sendFrameToBuffer(CAN_FRAME_FD &frame, int whichBus);

private:
    volatile size_t head;
    volatile size_t tail;
};