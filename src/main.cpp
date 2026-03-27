#include <Arduino.h>
#include <BleSerial.h>
#include "esp_partition.h"
#define DEBUG
#define RXD_RO 5
#define TXD_DI 6
#define RS485_EN 7

HardwareSerial modbus(1);
BleSerial ble;
bool isConnected;

// ================= CRC =================
uint16_t modbusCRC(uint8_t *buf, int len)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++)
  {
    crc ^= buf[pos];
    for (int i = 0; i < 8; i++)
    {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void rs485Tx() { digitalWrite(RS485_EN, HIGH); }
void rs485Rx() { digitalWrite(RS485_EN, LOW); }
void printHex(uint8_t *buf, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    if (buf[i] < 0x10)
      Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// ============== READ 7 REGISTERS ==============
bool readSoilAll(uint16_t *data, uint32_t &tRead)
{
  Serial.println("READ");
  uint8_t req[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07};

  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  uint8_t resp[32];
  int idx = 0;

  uint32_t t0 = millis();

  rs485Tx();
  modbus.write(req, 8);
  modbus.flush();
  delayMicroseconds(300);
  rs485Rx();

  uint32_t t = millis();

  while (idx < 19 && millis() - t < 500)
  {
    if (modbus.available())
    {

      auto b = modbus.read();
      Serial.println(b, HEX);
      if (idx == 0 && b != 0x01)
        continue;
      if (idx == 1 && b != 0x03)
      {
        idx = 0;
        continue;
      }
      resp[idx++] = b;
    }
  }

  tRead = millis() - t0;

  Serial.println("SYSYS2");
  if (idx < 19)
    return false;
  Serial.println("SYSYS3");

  uint16_t crcCalc = modbusCRC(resp, 17);
  uint16_t crcRecv = resp[17] | (resp[18] << 8);
  if (crcCalc != crcRecv)
    return false;

  // Parse data
  for (int i = 0; i < 7; i++)
  {
    data[i] = (resp[3 + i * 2] << 8) | resp[4 + i * 2];
  }
  return true;
}

float ema(float prev, float cur, float a = 0.2)
{
  return prev * (1.0f - a) + cur * a;
}
float map3(float x,
           float x0, float y0,
           float x1, float y1,
           float x2, float y2)
{
  if (x <= x1)
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

void _bleTask(void *parameter)
{
  while (true)
  {
    //   if (!isConnected)
    //   {
    //     vTaskDelay(10 / portTICK_PERIOD_MS);
    //     continue;
    //   }

    uint16_t d[7];
    uint32_t t;
    if (readSoilAll(d, t))
    {
      float moisture = d[0] / 10.0;

      int16_t tempRaw = (int16_t)d[1]; // signed
      float temperature = tempRaw / 10.0;

      uint16_t ec = d[2];
      float ph = d[3] / 10.0;

      uint16_t nitrogen = d[4];
      uint16_t phosphorus = d[5];
      uint16_t potassium = d[6];

      bool noSoil =
          (moisture < 5) ||
          (ec < 30);
      if (noSoil)
      {
        Serial.println("No Soil");

        ble.printf("M:%.1f\n", 0.0);
        ble.printf("T:%.1f\n", temperature);
        ble.printf("EC:%u\n", 0);
        ble.printf("pH:%.1f\n", ph);
        ble.printf("N:%.1f\n", 0.0);
        ble.printf("P:%.1f\n", 0.0);
        ble.printf("K:%.1f\n", 0.0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }

      ble.printf("M_raw:%.1f\n", moisture);
      ble.printf("T_raw:%.1f\n", temperature);
      ble.printf("EC_raw:%u\n", ec);
      ble.printf("pH_raw:%.1f\n", ph);
      ble.printf("N_raw:%.1f\n", nitrogen);
      ble.printf("P_raw:%.1f\n", phosphorus);
      ble.printf("K_raw:%.1f\n", potassium);
      // N
      float N_base = map3(nitrogen,
                          5, 20,
                          10, 100,
                          15, 220);
      float kN =
          constrain(1.0 * constrain(1.0 - (moisture - 60) * 0.005, 0.7, 1.1) * constrain(1.0 + (temperature - 25) * 0.01, 0.8, 1.2) * constrain(1.0 - ec / 2000.0, 0.75, 1.0),
                    0.6, 1.2);

      float N_ppm = constrain(N_base * kN, 0, 300);

      // P
      float P_base = map3(phosphorus,
                          10, 20,
                          40, 80,
                          80, 150);

      float kP =
          constrain(1.0 * constrain(1.0 - (moisture - 70) * 0.003, 0.8, 1.1) * constrain(1.0 + (temperature - 25) * 0.005, 0.9, 1.1),
                    0.8, 1.1);

      float P_ppm = constrain(P_base * kP, 0, 200);

      // K
      float K_base = map3(potassium,
                          10, 40,
                          40, 120,
                          80, 300);

      float kK =
          constrain(1.0 * constrain(1.0 - ec / 1500.0, 0.7, 1.0) * constrain(1.0 + (temperature - 25) * 0.005, 0.9, 1.1),
                    0.7, 1.1);

      float K_ppm = constrain(K_base * kK, 0, 400);

#ifdef DEBUG
      Serial.printf("M:%.1f\n", moisture);
      Serial.printf("T:%.1f\n", temperature);
      Serial.printf("EC:%u\n", ec);
      Serial.printf("pH:%.1f\n", ph);
      Serial.printf("N:%.1f\n", N_ppm);
      Serial.printf("P:%.1f\n", P_ppm);
      Serial.printf("K:%.1f\n", K_ppm);
#endif
      ble.printf("M:%.1f\n", moisture);
      ble.printf("T:%.1f\n", temperature);
      ble.printf("EC:%u\n", ec);
      ble.printf("pH:%.1f\n", ph);
      ble.printf("N:%.1f\n", N_ppm);
      ble.printf("P:%.1f\n", P_ppm);
      ble.printf("K:%.1f\n", K_ppm);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void printPartitions()
{
  Serial.println("=== Partition Table ===");
  esp_partition_iterator_t it =
      esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);

  while (it != NULL)
  {
    const esp_partition_t *p = esp_partition_get(it);
    Serial.printf("%-12s type=0x%02X subtype=0x%02X addr=0x%06X size=0x%06X (%u KB)\n",
                  p->label, p->type, p->subtype, p->address, p->size, p->size / 1024);
    it = esp_partition_next(it);
  }
}

// ================= SETUP =================
void setup()
{

  Serial.begin(115200);
  Serial.println("Starting...");
  modbus.begin(4800, SERIAL_8N1, RXD_RO, TXD_DI);
  Serial.println("Modbus setup done");
  pinMode(RS485_EN, OUTPUT);
  rs485Rx();
  Serial.println("Modbus setup done");

  ble.begin("AgriBeacon Soil");
  Serial.println("BLE setup done");
  ble.setConnectCallback([](bool connected)
                         { isConnected = connected; });
  xTaskCreatePinnedToCore(
      _bleTask,
      "BleTask",
      9216,
      NULL,
      1,
      NULL,
      PRO_CPU_NUM);
  Serial.println("Setup done");
  printPartitions();
}

// ================= LOOP =================
void loop()
{
  // static bool lastState = HIGH;
  // static unsigned long lastDebounceTime = 0;
  // bool reading = digitalRead(0);
  // if ((millis() - lastDebounceTime) > 50)
  // {
  //   if (lastState == LOW && reading == HIGH)
  //   {
  //     esp_restart();
  //   }
  // }
  // if (reading != lastState)
  // {
  //   lastDebounceTime = millis();
  // }
  // lastState = reading;
}
