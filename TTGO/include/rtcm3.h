#pragma once

#include <Arduino.h>

static const uint8_t RTCM3_PREAMBLE = 0xD3;
static const size_t RTCM3_MAX_PAYLOAD_SIZE = 1023;
static const size_t RTCM3_MAX_FRAME_SIZE = 3 + RTCM3_MAX_PAYLOAD_SIZE + 3;

typedef void (*RtcmFrameHandler)(const uint8_t *frame, uint16_t frame_len,
                                 uint16_t message_type);

class Rtcm3StreamParser
{
public:
    Rtcm3StreamParser();

    void feed(const uint8_t *data, size_t len, RtcmFrameHandler handler);
    uint32_t crc_error_count() const;
    uint32_t junk_byte_count() const;
    size_t pending_bytes() const;
    void reset_pending();

private:
    uint8_t buffer_[RTCM3_MAX_FRAME_SIZE];
    size_t buffered_;
    uint32_t crc_errors_;
    uint32_t junk_bytes_;

    void process(RtcmFrameHandler handler);
    void discard_prefix(size_t count);
};

uint32_t rtcm3_crc24q(const uint8_t *data, size_t len);
