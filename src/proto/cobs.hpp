#pragma once
#include <cstdint>
#include <cstddef>

namespace proto {

// COBS encode `in_len` bytes from `in` into `out`.
//   - `out_cap` must be >= in_len + (in_len / 254) + 1.
//   - Does NOT append the terminating 0x00 — caller does that.
//   - Returns number of bytes written to `out`, or 0 on out-of-space.
size_t cobs_encode(const uint8_t* in, size_t in_len, uint8_t* out, size_t out_cap);

// COBS decode `in_len` bytes from `in` (without the trailing 0x00) into `out`.
//   - Returns number of bytes written to `out`, or 0 on malformed input
//     (unexpected 0-byte, truncated frame, out-of-space).
size_t cobs_decode(const uint8_t* in, size_t in_len, uint8_t* out, size_t out_cap);

}  // namespace proto
