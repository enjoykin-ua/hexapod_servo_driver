#include "cobs.hpp"

namespace proto {

size_t cobs_encode(const uint8_t* in, size_t in_len, uint8_t* out, size_t out_cap) {
    // Worst-case output size: input + overhead (one extra byte per 254 input
    // bytes) + leading code byte. Add a safety margin of +2.
    if (out_cap < in_len + (in_len / 254) + 2) return 0;

    size_t  out_idx  = 1;   // out[0] is the first code byte, filled in later
    size_t  code_idx = 0;
    uint8_t code     = 0x01;

    for (size_t i = 0; i < in_len; ++i) {
        if (in[i] == 0x00) {
            out[code_idx] = code;
            code_idx      = out_idx++;
            code          = 0x01;
        } else {
            out[out_idx++] = in[i];
            ++code;
            if (code == 0xFF) {
                out[code_idx] = code;
                code_idx      = out_idx++;
                code          = 0x01;
            }
        }
    }
    out[code_idx] = code;
    return out_idx;
}

size_t cobs_decode(const uint8_t* in, size_t in_len, uint8_t* out, size_t out_cap) {
    if (in_len == 0) return 0;

    size_t in_idx  = 0;
    size_t out_idx = 0;

    while (in_idx < in_len) {
        uint8_t code = in[in_idx++];
        if (code == 0x00) return 0;  // unexpected 0 inside encoded data

        // Copy `code - 1` bytes from input to output.
        for (uint8_t i = 1; i < code; ++i) {
            if (in_idx >= in_len) return 0;     // truncated
            if (out_idx >= out_cap) return 0;   // out of space
            out[out_idx++] = in[in_idx++];
        }

        // Append a 0-byte separator between runs, except after a 0xFF
        // run (which signals a full 254-byte run without 0-byte) or at
        // the very end of input.
        if (code != 0xFF && in_idx < in_len) {
            if (out_idx >= out_cap) return 0;
            out[out_idx++] = 0x00;
        }
    }

    return out_idx;
}

}  // namespace proto
