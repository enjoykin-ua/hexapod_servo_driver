#pragma once
#include <cstdint>
#include <cstddef>

namespace proto {

// Frame layout (pre-COBS):
//   [SEQ:1] [CMD:1] [LEN:1] [PAYLOAD:LEN] [CRC16:2 LE]
//
// Max payload = 253 bytes -> pre-COBS frame max = 258 bytes.
// Wire (post-COBS + 0x00 trailer) max = ~261 bytes.

constexpr size_t MAX_PAYLOAD_LEN     = 253;
constexpr size_t MAX_FRAME_LEN_PRE   = 5 + MAX_PAYLOAD_LEN;
constexpr size_t MAX_FRAME_LEN_WIRE  = MAX_FRAME_LEN_PRE + (MAX_FRAME_LEN_PRE / 254) + 2;

struct Frame {
    uint8_t seq;
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD_LEN];
};

// Encode a Frame to wire format (COBS + 0x00 trailer).
//   Returns total bytes written to `out_buf`, including the trailing 0x00.
//   Returns 0 on failure (oversized payload or out-of-space).
size_t encode_frame(const Frame& f, uint8_t* out_buf, size_t out_cap);

// Decode a wire-format buffer (without trailing 0x00) into a Frame.
//   Returns true on success: COBS decodes cleanly, declared LEN matches,
//   CRC matches. Returns false otherwise (frame is discarded by caller).
bool decode_frame(const uint8_t* in_buf, size_t in_len, Frame& out);

// Streaming byte-by-byte frame decoder. Feed one byte at a time. When a
// complete frame is detected (terminating 0x00 received), tries to decode.
//   - Returns true if a valid frame is now in `out_frame` and assembler is
//     reset for next frame.
//   - Returns false in all other cases (need more bytes, or frame was
//     malformed/CRC-bad and was discarded).
// On overflow without seeing a 0x00, the buffer is dropped and the
// assembler waits for the next 0x00 to resync.
class FrameAssembler {
public:
    bool feed(uint8_t byte, Frame& out_frame);
    void reset();

private:
    uint8_t buf_[MAX_FRAME_LEN_WIRE];
    size_t  buf_len_      = 0;
    bool    overflowed_   = false;
};

}  // namespace proto
