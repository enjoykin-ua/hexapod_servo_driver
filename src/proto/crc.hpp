#pragma once
#include <cstdint>
#include <cstddef>

namespace proto {

// CRC-16/CCITT-FALSE
//   polynomial = 0x1021
//   init       = 0xFFFF
//   refin      = false
//   refout     = false
//   xorout     = 0x0000
// Self-test: crc16_ccitt_false((const uint8_t*)"123456789", 9) == 0x29B1
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

}  // namespace proto
