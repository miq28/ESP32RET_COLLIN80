#include "commbuffer.h"
#include "Logger.h"
#include "gvret_comm.h"

static DRAM_ATTR uint8_t buffer[CB_SIZE];

CommBuffer txBuffer;

CommBuffer::CommBuffer()
{
    head = 0;
    tail = 0;
}

size_t CommBuffer::numAvailableBytes()
{
    if (head >= tail)
        return head - tail;
    else
        return CB_SIZE - tail + head;
}

size_t CommBuffer::getLinearSize()
{
    if (head >= tail)
        return head - tail;
    else
        return CB_SIZE - tail;
}

uint8_t *CommBuffer::getBufferedBytes()
{
    return &buffer[tail];
}

void CommBuffer::consume(size_t n)
{
    size_t available = numAvailableBytes();
    if (n > available)
        n = available;

    tail = (tail + n) % CB_SIZE;
}

void CommBuffer::clearBufferedBytes()
{
    head = 0;
    tail = 0;
}

void CommBuffer::sendByteToBuffer(uint8_t b)
{
    size_t next = (head + 1) % CB_SIZE;

    // overwrite oldest if full (safe streaming behavior)
    if (next == tail)
    {
        tail = (tail + 1) % CB_SIZE;
    }

    buffer[head] = b;
    head = next;
}

void CommBuffer::sendString(const char *str)
{
    if (!str)
        return;

    while (*str)
    {
        sendByteToBuffer((uint8_t)*str++);
    }
}

void CommBuffer::sendString(const String &str)
{
    for (size_t i = 0; i < str.length(); i++)
    {
        sendByteToBuffer((uint8_t)str[i]);
    }
}

void CommBuffer::sendFrameToBuffer(CAN_FRAME &frame, int whichBus)
{
    if (!settings.useBinarySerialComm)
        return;

    uint32_t id = frame.id;
    if (frame.extended)
        id |= (1UL << 31);

    sendByteToBuffer(0xF1);
    sendByteToBuffer(0x00); // CMD

    uint32_t now = micros();

    sendByteToBuffer((uint8_t)(now));
    sendByteToBuffer((uint8_t)(now >> 8));
    sendByteToBuffer((uint8_t)(now >> 16));
    sendByteToBuffer((uint8_t)(now >> 24));

    sendByteToBuffer((uint8_t)(frame.id));
    sendByteToBuffer((uint8_t)(frame.id >> 8));
    sendByteToBuffer((uint8_t)(frame.id >> 16));
    sendByteToBuffer((uint8_t)(frame.id >> 24));

    // 🔥 THIS IS THE MOST IMPORTANT LINE
    sendByteToBuffer(frame.length + ((uint8_t)whichBus << 4));

    for (int i = 0; i < frame.length; i++)
    {
        sendByteToBuffer(frame.data.uint8[i]);
    }

    sendByteToBuffer(0); // checksum (must match original behavior)
}

void CommBuffer::sendFrameToBuffer(CAN_FRAME_FD &frame, int whichBus)
{
    if (!settings.useBinarySerialComm)
        return;

    uint32_t id = frame.id;
    if (frame.extended)
        id |= (1UL << 31);

    sendByteToBuffer(0xF1);
    sendByteToBuffer(PROTO_BUILD_FD_FRAME);

    uint32_t now = micros();

    sendByteToBuffer((uint8_t)(now));
    sendByteToBuffer((uint8_t)(now >> 8));
    sendByteToBuffer((uint8_t)(now >> 16));
    sendByteToBuffer((uint8_t)(now >> 24));

    sendByteToBuffer((uint8_t)(frame.id));
    sendByteToBuffer((uint8_t)(frame.id >> 8));
    sendByteToBuffer((uint8_t)(frame.id >> 16));
    sendByteToBuffer((uint8_t)(frame.id >> 24));

    sendByteToBuffer(frame.length);
    sendByteToBuffer((uint8_t)whichBus);

    for (int i = 0; i < frame.length; i++)
    {
        sendByteToBuffer(frame.data.uint8[i]);
    }

    sendByteToBuffer(0);
}