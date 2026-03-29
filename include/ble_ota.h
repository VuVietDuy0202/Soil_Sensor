#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "esp_ota_ops.h"

// ================= BLE OTA UUIDs =================
#define OTA_SERVICE_UUID  "fb1e4001-54ae-4a28-9f74-dfccb248601d"
#define OTA_CONTROL_UUID  "fb1e4002-54ae-4a28-9f74-dfccb248601d"
#define OTA_DATA_UUID     "fb1e4003-54ae-4a28-9f74-dfccb248601d"

// OTA Control commands (Web -> ESP32)
#define OTA_CMD_START  0x01  // payload: 4 bytes firmware size (little-endian)
#define OTA_CMD_STOP   0x02  // cancel OTA
#define OTA_CMD_DONE   0x03  // all chunks sent, finalize

// OTA Status responses (ESP32 -> Web via notify)
#define OTA_STATUS_OK       0x00
#define OTA_STATUS_READY    0x01
#define OTA_STATUS_PROGRESS 0x02
#define OTA_STATUS_DONE     0x03  // OTA success, about to reboot
#define OTA_STATUS_ERROR    0xFF

extern volatile bool otaInProgress;

void setupOtaService(BLEServer *pServer, TaskHandle_t sensorTask);
