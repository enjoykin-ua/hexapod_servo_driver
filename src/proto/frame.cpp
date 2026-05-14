#include "frame.hpp"
#include "cobs.hpp"
#include "crc.hpp"
#include <cstring>

namespace proto {

size_t encode_frame(const Frame& f, uint8_t* out_buf, size_t out_cap) {
    if (f.len > MAX_PAYLOAD_LEN) return 0;
    if (out_cap < 1) return 0;

    // Build pre-COBS buffer: SEQ + CMD + LEN + PAYLOAD + CRC16(LE)
    uint8_t pre[MAX_FRAME_LEN_PRE];
    pre[0] = f.seq;
    pre[1] = f.cmd;
    pre[2] = f.len;
    if (f.len > 0) {
        std::memcpy(&pre[3], f.payload, f.len);
    }

    const uint16_t crc = crc16_ccitt_false(pre, 3 + f.len);
    pre[3 + f.len + 0] = static_cast<uint8_t>(crc & 0xFF);          // LE low
    pre[3 + f.len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);   // LE high
    const size_t pre_len = 5 + f.len;

    // Leave room for trailing 0x00
    const size_t n = cobs_encode(pre, pre_len, out_buf, out_cap - 1);
    if (n == 0) return 0;

    out_buf[n] = 0x00;
    return n + 1;
}

bool decode_frame(const uint8_t* in_buf, size_t in_len, Frame& out) {
    uint8_t pre[MAX_FRAME_LEN_PRE];
    const size_t pre_len = cobs_decode(in_buf, in_len, pre, sizeof(pre));
    if (pre_len < 5) return false;  // need at least SEQ+CMD+LEN+CRC16

    const uint8_t len = pre[2];
    if (static_cast<size_t>(5) + len != pre_len) return false;  // declared LEN inconsistent

    const uint16_t got_crc  = static_cast<uint16_t>(pre[3 + len])
                            | (static_cast<uint16_t>(pre[3 + len + 1]) << 8);
    const uint16_t want_crc = crc16_ccitt_false(pre, 3 + len);
    if (got_crc != want_crc) return false;

    out.seq = pre[0];
    out.cmd = pre[1];
    out.len = len;
    if (len > 0) {
        std::memcpy(out.payload, &pre[3], len);
    }
    return true;
}

bool FrameAssembler::feed(uint8_t byte, Frame& out_frame) {
    if (byte == 0x00) {
        // End of frame. If we overflowed, drop silently and reset.
        bool ok = false;
        if (!overflowed_ && buf_len_ > 0) {
            ok = decode_frame(buf_, buf_len_, out_frame);
        }
        buf_len_    = 0;
        overflowed_ = false;
        return ok;
    }

    if (overflowed_ || buf_len_ >= MAX_FRAME_LEN_WIRE) {
        overflowed_ = true;
        return false;
    }
    buf_[buf_len_++] = byte;
    return false;
}

void FrameAssembler::reset() {
    buf_len_    = 0;
    overflowed_ = false;
}

}  // namespace proto
