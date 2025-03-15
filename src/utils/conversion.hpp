#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>
#include <cstdint>
#include <cstring>
#include <limits>

/**
 * @brief Converts a float to a byte array.
 * 
 * This function stores the binary representation of a float 
 * into a `std::vector<uint8_t>`, keeping the exact byte layout.
 * 
 * @param value The float value to convert.
 * @return std::vector<uint8_t> A 4-byte vector containing the float data.
 */
inline std::vector<uint8_t> float2vec(float value) {
    std::vector<uint8_t> vec(sizeof(float));
    std::memcpy(vec.data(), &value, sizeof(float));
    return vec;
}

/**
 * @brief Converts a byte array to a float.
 * 
 * Extracts a 4-byte float value from a byte vector starting at the given offset.
 * If there aren’t enough bytes, it returns NaN.
 * 
 * @param vec The byte vector containing float data.
 * @param offset The starting position in the vector (default: 0).
 * @return float The extracted float value or NaN if the offset is invalid.
 */
inline float vec2float(const std::vector<uint8_t>& vec, std::size_t offset = 0) {
    float value;
    if (offset + sizeof(float) > vec.size()) {
        value = std::numeric_limits<float>::quiet_NaN(); // Return NaN if out of bounds
    } else {
        std::memcpy(&value, vec.data() + offset, sizeof(float)); // Copy bytes into float
    }
    return value;
}

#endif // UTILS_HPP
