#include "ble_ota.h"

volatile bool otaInProgress = false;
static volatile bool otaFinalize = false;

static esp_ota_handle_t otaHandle = 0;
static const esp_partition_t *otaPartition = NULL;
static uint32_t otaFirmwareSize = 0;
static uint32_t otaReceived = 0;
static BLECharacteristic *otaControlChar = NULL;
static TaskHandle_t sensorTaskHandle = NULL;
static uint8_t otaLastPct = 0;

static void otaSendStatus(uint8_t status, uint8_t extra = 0)
{
  uint8_t resp[2] = {status, extra};
  otaControlChar->setValue(resp, 2);
  otaControlChar->notify();
}

// Task to finalize OTA (runs outside BLE callback)
static void otaFinalizeTask(void *param)
{
  Serial.printf("[OTA] Finalizing... received=%u/%u\n", otaReceived, otaFirmwareSize);

  esp_err_t err = esp_ota_end(otaHandle);
  if (err != ESP_OK)
  {
    Serial.printf("[OTA] esp_ota_end failed: %s\n", esp_err_to_name(err));
    otaInProgress = false;
    if (sensorTaskHandle) vTaskResume(sensorTaskHandle);
    otaSendStatus(OTA_STATUS_ERROR);
    vTaskDelete(NULL);
    return;
  }

  err = esp_ota_set_boot_partition(otaPartition);
  if (err != ESP_OK)
  {
    Serial.printf("[OTA] set_boot failed: %s\n", esp_err_to_name(err));
    otaInProgress = false;
    if (sensorTaskHandle) vTaskResume(sensorTaskHandle);
    otaSendStatus(OTA_STATUS_ERROR);
    vTaskDelete(NULL);
    return;
  }

  Serial.println("[OTA] Success! Rebooting...");
  otaSendStatus(OTA_STATUS_DONE);
  delay(1500);
  esp_restart();
}

// ================= OTA Control Callback =================
class OTAControlCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pChar) override
  {
    std::string val = pChar->getValue();
    if (val.empty()) return;
    uint8_t cmd = val[0];

    if (cmd == OTA_CMD_START)
    {
      if (val.size() < 5)
      {
        Serial.println("[OTA] START missing size");
        otaSendStatus(OTA_STATUS_ERROR);
        return;
      }
      otaFirmwareSize = (uint8_t)val[1] | ((uint8_t)val[2] << 8) |
                        ((uint8_t)val[3] << 16) | ((uint8_t)val[4] << 24);
      Serial.printf("[OTA] START size=%u\n", otaFirmwareSize);

      otaPartition = esp_ota_get_next_update_partition(NULL);
      if (!otaPartition)
      {
        Serial.println("[OTA] No OTA partition");
        otaSendStatus(OTA_STATUS_ERROR);
        return;
      }

      esp_err_t err = esp_ota_begin(otaPartition, otaFirmwareSize, &otaHandle);
      if (err != ESP_OK)
      {
        Serial.printf("[OTA] esp_ota_begin failed: %s\n", esp_err_to_name(err));
        otaSendStatus(OTA_STATUS_ERROR);
        return;
      }

      otaReceived = 0;
      otaLastPct = 0;
      otaFinalize = false;
      otaInProgress = true;
      if (sensorTaskHandle) vTaskSuspend(sensorTaskHandle);
      Serial.println("[OTA] Ready to receive");
      otaSendStatus(OTA_STATUS_READY);
    }
    else if (cmd == OTA_CMD_STOP)
    {
      Serial.println("[OTA] Cancelled");
      if (otaInProgress)
      {
        esp_ota_abort(otaHandle);
        otaInProgress = false;
        if (sensorTaskHandle) vTaskResume(sensorTaskHandle);
      }
      otaSendStatus(OTA_STATUS_OK);
    }
  }
};

// ================= OTA Data Callback =================
class OTADataCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pChar) override
  {
    if (!otaInProgress) return;

    std::string val = pChar->getValue();
    if (val.empty()) return;

    esp_err_t err = esp_ota_write(otaHandle, val.data(), val.size());
    if (err != ESP_OK)
    {
      Serial.printf("[OTA] write failed: %s\n", esp_err_to_name(err));
      esp_ota_abort(otaHandle);
      otaInProgress = false;
      if (sensorTaskHandle) vTaskResume(sensorTaskHandle);
      otaSendStatus(OTA_STATUS_ERROR);
      return;
    }

    otaReceived += val.size();

    if (otaFirmwareSize > 0)
    {
      uint8_t pct = (uint8_t)((otaReceived * 100) / otaFirmwareSize);
      if (pct >= otaLastPct + 10)
      {
        Serial.printf("[OTA] %u%%\n", pct);
        otaSendStatus(OTA_STATUS_PROGRESS, pct);
        otaLastPct = pct;
      }
      // All data received -> finalize in separate task
      if (otaReceived >= otaFirmwareSize && !otaFinalize)
      {
        Serial.printf("[OTA] All data received! Free heap: %u\n", ESP.getFreeHeap());
        otaFinalize = true;
        if (otaLastPct < 100)
        {
          Serial.println("[OTA] 100%");
          otaSendStatus(OTA_STATUS_PROGRESS, 100);
          otaLastPct = 100;
        }
        BaseType_t ret = xTaskCreate(otaFinalizeTask, "OTA_Final", 8192, NULL, 5, NULL);
        if (ret != pdPASS)
        {
          Serial.printf("[OTA] xTaskCreate failed! ret=%d, trying direct...\n", ret);
          // Fallback: run directly (risky but better than stuck)
          otaFinalizeTask(NULL);
        }
      }
    }
  }
};

// ================= Setup =================
void setupOtaService(BLEServer *pServer, TaskHandle_t sensorTask)
{
  sensorTaskHandle = sensorTask;

  if (!pServer)
  {
    Serial.println("[OTA] BLE Server not ready!");
    return;
  }

  BLEService *otaService = pServer->createService(OTA_SERVICE_UUID);

  otaControlChar = otaService->createCharacteristic(
      OTA_CONTROL_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  otaControlChar->addDescriptor(new BLE2902());
  otaControlChar->setCallbacks(new OTAControlCallback());

  BLECharacteristic *otaDataChar = otaService->createCharacteristic(
      OTA_DATA_UUID,
      BLECharacteristic::PROPERTY_WRITE_NR);
  otaDataChar->setCallbacks(new OTADataCallback());

  otaService->start();

  pServer->getAdvertising()->addServiceUUID(OTA_SERVICE_UUID);
  pServer->getAdvertising()->start();

  Serial.println("[OTA] Service ready");
}
