/**
 * @file CRC.h
 * @author yao
 * @date 2021年1月13日
 * @brief C++实现的CRC
 */

#ifndef KDROBOTCPPLIBS_CRC_H
#define KDROBOTCPPLIBS_CRC_H

#include <cstdint>

#include <vector>

namespace custom_serial {
/**
 * @brief Descriptions: CRC8 checksum function
 * @param pchMessage Data to check
 * @param dwLength Stream length
 * @param wCRC initialized checksum
 * @return CRC checksum
 */
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint8_t dwLength,
                           uint8_t ucCRC8);

/**
 * @brief Descriptions: CRC16 checksum function
 * @param pchMessage Data to check
 * @param dwLength Stream length
 * @param wCRC initialized checksum
 * @return CRC checksum
 */
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength,
                             uint16_t wCRC);

/**
 * @brief Descriptions: CRC8 Verify function
 * @param pchMessage Data to Verify
 * @param dwLength Stream length = Data + checksum
 * @return CRC Verify Result
 */
uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint8_t dwLength);

/**
 * overloaded function
 * @param pchMessage Data to Verify
 * @return CRC Verify Result
 */
inline uint8_t Verify_CRC8_Check_Sum(std::vector<uint8_t> pchMessage) {
  return Verify_CRC8_Check_Sum(pchMessage.data(), (uint8_t)pchMessage.size());
}

/**
 * @brief CRC16 Verify function
 * @param pchMessage Data to Verify
 * @param dwLength Stream length = Data + checksum
 * @return CRC Verify Result
 */
uint16_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/**
 * overloaded function
 * @param pchMessage Data to Verify
 * @return CRC Verify Result
 */
inline uint16_t Verify_CRC16_Check_Sum(std::vector<uint8_t> pchMessage) {
  return Verify_CRC16_Check_Sum(pchMessage.data(), (uint8_t)pchMessage.size());
}
} // namespace custom_serial

#endif // VCOMCOMM_PC_CRC_H
