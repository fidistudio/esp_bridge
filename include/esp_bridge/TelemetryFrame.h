#pragma once

#include <stdint.h>

/**
 * @brief Frame binario de telemetría ESP32 → PC a 20 Hz.
 *
 * Protocolo:
 *   [0xAA 0x55] [payload: 28 bytes] [0x0A]
 *   Total: 31 bytes por frame
 *
 * El magic (0xAA55) permite re-sincronización si se pierden bytes.
 * El end marker (0x0A) es redundante pero ayuda al parser de ROS2.
 *
 * IMPORTANTE: ambos lados (ESP32 y ROS2) deben compilar con el
 * mismo struct para garantizar el layout. En ROS2 usar el mismo
 * header copiado o replicar el layout manualmente.
 */

static constexpr uint16_t TELEMETRY_MAGIC = 0xAA55;
static constexpr uint8_t TELEMETRY_END_MARKER = 0x0A;
static constexpr uint8_t TELEMETRY_FRAME_SIZE =
    31; // magic(2) + payload(28) + end(1)

#pragma pack(push, 1)
struct TelemetryPayload {
  float x;         ///< Posición X [m]
  float y;         ///< Posición Y [m]
  float theta;     ///< Orientación [rad]
  float pos_left;  ///< Posición angular rueda izquierda [rad] acumulado
  float pos_right; ///< Posición angular rueda derecha [rad] acumulado
  float vel_left;  ///< Velocidad angular rueda izquierda [rad/s]
  float vel_right; ///< Velocidad angular rueda derecha [rad/s]
                   // 7 × 4 bytes = 28 bytes
};
#pragma pack(pop)

struct TelemetryFrame {
  uint16_t magic;        // 0xAA55
  TelemetryPayload data; // 28 bytes
  uint8_t end;           // 0x0A
};
