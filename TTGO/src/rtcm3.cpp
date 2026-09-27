#include "rtcm3.h"

#include <string.h>

static const uint32_t CRC24Q_POLY = 0x1864CFB;

Rtcm3StreamParser::Rtcm3StreamParser()
    : buffered_(0), crc_errors_(0), junk_bytes_(0)
{
}

uint32_t rtcm3_crc24q(const uint8_t *data, size_t len)
{
    uint32_t crc = 0;
    for(size_t i = 0; i < len; i++)
    {
        crc ^= (uint32_t)data[i] << 16;
        for(uint8_t bit = 0; bit < 8; bit++)
        {
            crc <<= 1;
            if(crc & 0x1000000)
            {
                crc ^= CRC24Q_POLY;
            }
        }
    }
    return crc & 0xFFFFFF;
}

void Rtcm3StreamParser::feed(const uint8_t *data, size_t len,
                             RtcmFrameHandler handler)
{
    for(size_t i = 0; i < len; i++)
    {
        if(buffered_ >= sizeof(buffer_))
        {
            // 유효하지 않은 길이/CRC 때문에 버퍼가 가득 찬 경우 한 바이트를 버리고
            // 다음 0xD3를 다시 찾는다. RTCM 메시지 중간 바이트는 임의로 송신하지 않는다.
            discard_prefix(1);
            junk_bytes_++;
        }
        buffer_[buffered_++] = data[i];
        process(handler);
    }
}

void Rtcm3StreamParser::process(RtcmFrameHandler handler)
{
    while(buffered_ > 0)
    {
        size_t preamble_index = 0;
        while(preamble_index < buffered_ && buffer_[preamble_index] != RTCM3_PREAMBLE)
        {
            preamble_index++;
        }

        if(preamble_index == buffered_)
        {
            junk_bytes_ += buffered_;
            buffered_ = 0;
            return;
        }
        if(preamble_index > 0)
        {
            junk_bytes_ += preamble_index;
            discard_prefix(preamble_index);
        }

        if(buffered_ < 3)
        {
            return;
        }

        // RTCM3 header의 두 번째 바이트 상위 6비트는 reserved이며 0이어야 한다.
        if(buffer_[1] & 0xFC)
        {
            discard_prefix(1);
            junk_bytes_++;
            continue;
        }

        const uint16_t payload_len =
            ((uint16_t)(buffer_[1] & 0x03) << 8) | buffer_[2];
        const uint16_t frame_len = 3 + payload_len + 3;
        if(buffered_ < frame_len)
        {
            return;
        }

        const uint32_t received_crc =
            ((uint32_t)buffer_[frame_len - 3] << 16) |
            ((uint32_t)buffer_[frame_len - 2] << 8) |
            (uint32_t)buffer_[frame_len - 1];
        const uint32_t calculated_crc = rtcm3_crc24q(buffer_, frame_len - 3);

        if(received_crc != calculated_crc)
        {
            crc_errors_++;
            // 잘못된 후보 전체를 버리지 않고 첫 바이트만 버려서 그 안에 들어온
            // 다음 정상 RTCM preamble을 다시 검색한다.
            discard_prefix(1);
            continue;
        }

        uint16_t message_type = 0;
        if(payload_len >= 2)
        {
            message_type = ((uint16_t)buffer_[3] << 4) | (buffer_[4] >> 4);
        }
        if(handler != nullptr)
        {
            handler(buffer_, frame_len, message_type);
        }
        discard_prefix(frame_len);
    }
}

void Rtcm3StreamParser::discard_prefix(size_t count)
{
    if(count >= buffered_)
    {
        buffered_ = 0;
        return;
    }
    memmove(buffer_, buffer_ + count, buffered_ - count);
    buffered_ -= count;
}

uint32_t Rtcm3StreamParser::crc_error_count() const
{
    return crc_errors_;
}

uint32_t Rtcm3StreamParser::junk_byte_count() const
{
    return junk_bytes_;
}

size_t Rtcm3StreamParser::pending_bytes() const
{
    return buffered_;
}

void Rtcm3StreamParser::reset_pending()
{
    // PC 패킷이 중간에 끊겼을 때 다음 주기의 RTCM과 섞이지 않도록
    // 완성되지 않은 현재 후보만 버린다. 누적 CRC/잡음 통계는 유지한다.
    buffered_ = 0;
}
