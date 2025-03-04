#include "AHT20.h"
#include "serial_proto.h"
#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2CSgp40.h>
#include <VOCGasIndexAlgorithm.h>
#include <Wire.h>

#define VERSION "v1.0.2M"

#define SENSECAP                                                               \
  "\n\
   _____                      _________    ____         \n\
  / ___/___  ____  ________  / ____/   |  / __ \\       \n\
  \\__ \\/ _ \\/ __ \\/ ___/ _ \\/ /   / /| | / /_/ /   \n\
 ___/ /  __/ / / (__  )  __/ /___/ ___ |/ ____/         \n\
/____/\\___/_/ /_/____/\\___/\\____/_/  |_/_/           \n\
--------------------------------------------------------\n\
 Version: %s \n\
--------------------------------------------------------\n\
"

AHT20 AHT;
SensirionI2CSgp40 sgp40;
SensirionI2cScd4x scd4x;
VOCGasIndexAlgorithm voc_algorithm;

const int BUFFER_SIZE =
    1024; // Buffer size for NMEA sentences (generous for Multi-GNSS)
char nmeaBuffer[BUFFER_SIZE];
int bufferIndex = 0;

bool active = false;
uint32_t collectInterval = 5000;

// sensor data send to  esp32
void sensor_data_send(meshtastic_MessageType type, float data) {
  meshtastic_InterdeviceMessage myPacket =
      meshtastic_InterdeviceMessage_init_default;
  myPacket.data.sensor.type = type;
  myPacket.data.sensor.data.float_value = data;
  mt_send_uplink(myPacket);
}

void printSerialNumber(uint64_t value) {
  Serial.print("Serial: 0x");
  Serial.print(value < 0x1000000000000000 ? "0" : "");
  Serial.print(value < 0x100000000000000 ? "0" : "");
  Serial.print(value < 0x10000000000000 ? "0" : "");
  Serial.print(value < 0x1000000000000 ? "0" : "");
  Serial.print(value < 0x100000000000 ? "0" : "");
  Serial.print(value < 0x10000000000 ? "0" : "");
  Serial.print(value < 0x1000000000 ? "0" : "");
  Serial.print(value < 0x100000000 ? "0" : "");
  Serial.print(value < 0x10000000 ? "0" : "");
  Serial.print(value < 0x1000000 ? "0" : "");
  Serial.print(value < 0x100000 ? "0" : "");
  Serial.print(value < 0x10000 ? "0" : "");
  Serial.print(value < 0x1000 ? "0" : "");
  Serial.print(value < 0x100 ? "0" : "");
  Serial.print(value < 0x10 ? "0" : "");
  Serial.print(value, HEX);
  Serial.println();
}

float temperature = 0.0;
float humidity = 0.0;

uint16_t defaultCompenstaionRh = 0x8000;
uint16_t defaultCompenstaionT = 0x6666;

uint16_t compensationRh = defaultCompenstaionRh;
uint16_t compensationT = defaultCompenstaionT;

uint32_t sent = 0;

/************************ aht  temp & humidity ****************************/

void sensor_aht_init(void) { AHT.begin(); }

void sensor_aht_get(void) {

  float humi, temp;

  int ret = AHT.getSensor(&humi, &temp);

  if (ret) {
    // GET DATA OK
    Serial.print("humidity: ");
    Serial.print(humi * 100);
    Serial.print("%\t temerature: ");
    Serial.println(temp);
    temperature = temp;
    humidity = humi * 100;
    compensationT = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
    compensationRh = static_cast<uint16_t>(humidity * 65535 / 100);
  } else {
    // GET DATA FAIL
    Serial.println("GET DATA FROM AHT20 FAIL");
    compensationRh = defaultCompenstaionRh;
    compensationT = defaultCompenstaionT;
  }

  if (ret) {
    sensor_data_send(meshtastic_MessageType_AHT20_TEMP, temperature);
    sensor_data_send(meshtastic_MessageType_AHT20_HUMIDITY, humidity);
  }
}

/************************ sgp40 tvoc  ****************************/

void sensor_sgp40_init(void) {
  uint16_t error;
  char errorMessage[256];

  sgp40.begin(Wire);

  uint16_t serialNumber[3];
  uint8_t serialNumberSize = 3;

  error = sgp40.getSerialNumber(serialNumber, serialNumberSize);

  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("SerialNumber:");
    Serial.print("0x");
    for (size_t i = 0; i < serialNumberSize; i++) {
      uint16_t value = serialNumber[i];
      Serial.print(value < 4096 ? "0" : "");
      Serial.print(value < 256 ? "0" : "");
      Serial.print(value < 16 ? "0" : "");
      Serial.print(value, HEX);
    }
    Serial.println();
  }

  uint16_t testResult;
  error = sgp40.executeSelfTest(testResult);
  if (error) {
    Serial.print("Error trying to execute executeSelfTest(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (testResult != 0xD400) {
    Serial.print("executeSelfTest failed with error: ");
    Serial.println(testResult);
  }
}

void sensor_sgp40_get(void) {
  uint16_t error;
  char errorMessage[256];
  uint16_t defaultRh = 0x8000;
  uint16_t defaultT = 0x6666;
  uint16_t srawVoc = 0;

  Serial.print("sensor sgp40: ");

  error = sgp40.measureRawSignal(compensationRh, compensationT, srawVoc);
  if (error) {
    Serial.print("Error trying to execute measureRawSignal(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("SRAW_VOC:");
    Serial.println(srawVoc);
  }

  if (!error) {
    int32_t voc_index = voc_algorithm.process(srawVoc);
    Serial.print("VOC Index: ");
    Serial.println(voc_index);

    sensor_data_send(meshtastic_MessageType_TVOC_INDEX, (float)voc_index);
  }
}

/************************ scd4x  co2 ****************************/

void sensor_scd4x_init(void) {
  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire, 0x62);

  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  uint64_t serial;

  error = scd4x.getSerialNumber(serial);
  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    printSerialNumber(serial);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  // scd4x.powerDown();
}

void sensor_scd4x_get(void) {
  uint16_t error;
  char errorMessage[256];

  Serial.print("sensor scd4x: ");
  // Read Measurement
  uint16_t co2;
  float temperature;
  float humidity;
  error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error) {
    Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (co2 == 0) {
    Serial.println("Invalid sample detected, skipping.");
  } else {
    Serial.print("Co2:");
    Serial.print(co2);
    Serial.print("\t");
    Serial.print("Temperature:");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print("Humidity:");
    Serial.println(humidity);
  }

  if (!error) {
    sensor_data_send(meshtastic_MessageType_SCD41_CO2, (float)co2);
    sensor_data_send(meshtastic_MessageType_SCD41_TEMP, temperature);
    sensor_data_send(meshtastic_MessageType_SCD41_HUMIDITY, humidity);
  }
}

void sensor_power_on(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
  active = true;
  delay(100);
  sensor_aht_init();
  sensor_sgp40_init();
  sensor_scd4x_init();
  sent = 0;
}

void sensor_power_off(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  active = false;
}

/************************ beep ****************************/

#define Buzzer 19 // Buzzer GPIO

uint32_t buzz_off = 0;

void beep_init(void) { pinMode(Buzzer, OUTPUT); }
void beep_off(void) { digitalWrite(Buzzer, LOW); }
void beep_on(uint32_t duration) {
  analogWrite(Buzzer, 127);
  buzz_off = millis() + duration;
}

/************************ grove  ****************************/

// Don't send this for now, we need the 2nd port for the GPS

// void grove_adc_get(void) {
//   String dataString = "";
//   int adc0 = analogRead(26);
//   dataString += String(adc0);
//   dataString += ',';
//   int adc1 = analogRead(27);
//   dataString += String(adc1);
//   Serial.print("grove adc: ");
//   Serial.println(dataString);
// }

/************************ recv cmd from esp32  ****************************/

static bool shutdown_flag = false;

void onPacketReceived(meshtastic_SensorData sensor) {
  switch (sensor.type) {
  case meshtastic_MessageType_POWER_ON: {
    Serial.println("cmd power on");
    sensor_power_on();
    break;
  }
  case meshtastic_MessageType_SHUTDOWN: {
    Serial.println("cmd shutdown");
    shutdown_flag = true;
    sensor_power_off();
    break;
  }
  case meshtastic_MessageType_BEEP_ON: {
    Serial.println("cmd beep on");
    beep_on(sensor.data.uint32_value);
    break;
  }
  case meshtastic_MessageType_BEEP_OFF: {
    Serial.println("cmd beep off");
    beep_off();
    break;
  }
  case meshtastic_MessageType_COLLECT_INTERVAL: {
    collectInterval = sensor.data.uint32_value;
    Serial.println("cmd collect interval " + String(collectInterval));
    break;
  }

  default:
    break;
  }
}

void onNmeaReceived(char *nmea) {
  // pass this on to the GPS chip
  Serial2.print(nmea);
}

/************************ setup & loop ****************************/

int cnt = 0;

void setup() {
  // The Virtual USB Serial
  Serial.begin(115200);

  // this is the device link to the ESP32-S3 CPU
  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);

  // use pin 26 and 27 for the GPS Chip serial link
  Serial2.setRX(27);
  Serial2.setTX(26);
  Serial2.begin(9600);

  // I2C is on 20/21
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  mt_set_sensor_callback(onPacketReceived);
  mt_set_nmea_callback(onNmeaReceived);

  int32_t index_offset;
  int32_t learning_time_offset_hours;
  int32_t learning_time_gain_hours;
  int32_t gating_max_duration_minutes;
  int32_t std_initial;
  int32_t gain_factor;
  voc_algorithm.get_tuning_parameters(
      index_offset, learning_time_offset_hours, learning_time_gain_hours,
      gating_max_duration_minutes, std_initial, gain_factor);

  Serial.println("\nVOC Gas Index Algorithm parameters");
  Serial.print("Index offset:\t");
  Serial.println(index_offset);
  Serial.print("Learing time offset hours:\t");
  Serial.println(learning_time_offset_hours);
  Serial.print("Learing time gain hours:\t");
  Serial.println(learning_time_gain_hours);
  Serial.print("Gating max duration minutes:\t");
  Serial.println(gating_max_duration_minutes);
  Serial.print("Std inital:\t");
  Serial.println(std_initial);
  Serial.print("Gain factor:\t");
  Serial.println(gain_factor);

  beep_init();
  delay(500);
  beep_on(50);

  Serial.printf(SENSECAP, VERSION);
}

void loop() {

  if ((sent == 0 || (millis() - sent > collectInterval)) && (active)) {
    Serial.printf("\r\n\r\n--------- start measure %d-------\r\n", cnt);
    cnt++;
    sensor_aht_get();
    sensor_sgp40_get();
    sensor_scd4x_get();
    // Don't send this for now, we need the 2nd port for the GPS
    // grove_adc_get();
    sent = millis();
  }

  if (buzz_off > 0 && millis() > buzz_off) {
    beep_off();
    buzz_off = 0;
  }

  mt_loop();

  // read GPS data lines into buffer and send it to the ESP32
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n') {
      nmeaBuffer[bufferIndex] = '\0';
      Serial.print(nmeaBuffer);
      meshtastic_InterdeviceMessage myPacket =
          meshtastic_InterdeviceMessage_init_default;
      strncpy(myPacket.data.nmea, nmeaBuffer, bufferIndex);
      mt_send_uplink(myPacket);
      bufferIndex = 0;
      break;
    } else {
      if (bufferIndex < BUFFER_SIZE - 1) {
        nmeaBuffer[bufferIndex++] = c;
      } else {
        bufferIndex = 0; // Reset in case of buffer overflow
      }
    }
  }
}
