#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "serial_proto.h"
#include "buzzer.h"

#define VERSION "v2.0.0M"

// use pin 26 and 27 for the GPS Chip serial link
SerialPIO softSerial2(26, 27, 1024);

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

// Buffer size for NMEA sentences (generous for Multi-GNSS)
const int BUFFER_SIZE = 1024;
char nmeaBuffer[BUFFER_SIZE];
int bufferIndex = 0;

bool sd_init_flag = 0;
const int SD_CS_PIN = 13;

// used/free statistics, computed in the background: the FAT scan behind
// usedBytes takes seconds on FAT16/32 cards and must not stall the
// request path (the UI on the other end waits synchronously)
static bool sd_stats_pending = false;
static bool sd_stats_valid = false;
static uint64_t sd_used_bytes = 0;
static uint64_t sd_free_bytes = 0;

// (Re-)initialize the SD card on demand. The Indicator's slot has no
// card-detect line, so retry rate-limited whenever a file operation
// arrives; this makes inserting a card after boot just work.
bool sd_ensure(void) {
  static uint32_t last_attempt = 0;
  if (sd_init_flag)
    return true;
  if (last_attempt != 0 && millis() - last_attempt < 2000)
    return false;
  last_attempt = millis();
  if (!SD.begin(SD_CS_PIN, 16000000, SPI1)) {
    Serial.println("Card failed, or not present");
    return false;
  }
  Serial.println("card initialized.");
  sd_init_flag = 1;
  sd_stats_valid = false;
  sd_stats_pending = true;
  return true;
}

// runs from loop(), outside the request path
void sd_update_stats(void) {
  if (!sd_stats_pending || !sd_init_flag)
    return;
  sd_stats_pending = false;
  FSInfo info;
  if (SDFS.info(info)) {
    sd_used_bytes = info.usedBytes;
    sd_free_bytes = info.totalBytes - info.usedBytes;
    sd_stats_valid = true;
  }
}

// false while the background scan has not completed yet
bool sd_used_free(uint64_t *used, uint64_t *free_bytes) {
  if (!sd_stats_valid)
    return false;
  *used = sd_used_bytes;
  *free_bytes = sd_free_bytes;
  return true;
}

// The built-in sensor needs to be powered on
void sensor_power_on(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
}

/************************ beep ****************************/

void onNmeaReceived(char *nmea) {
  // pass this on to the GPS chip
  softSerial2.print(nmea);
}

/************************ setup & loop ****************************/

void setup() {
  // The Virtual USB Serial for debug logging
  Serial.begin(115200);

  sensor_power_on();

  // this is the device link to the ESP32-S3 CPU
  // FIFO must hold a full request frame (~4.7KB for chunked writes) plus
  // whatever arrives while a response transmit blocks (~23ms at 2M baud)
  Serial1.setFIFOSize(8192);
  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(2000000);

  softSerial2.begin(9600);

  // I2C is on 20/21
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  // SD Card slot is on SPI1
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  sd_ensure();

  mt_set_nmea_callback(onNmeaReceived);

  beep_init();
  delay(500);

  Serial.printf(SENSECAP, VERSION);
  beep_on(50);
}

void loop() {

  if (buzz_off > 0 && millis() > buzz_off) {
    beep_off();
    buzz_off = 0;
  }

  sd_update_stats();
  mt_loop();

  // read GPS data lines into buffer and send it to the ESP32
  while (softSerial2.available()) {
    char c = softSerial2.read();

    if (c == '\n') {
      nmeaBuffer[bufferIndex] = '\0';
      Serial.println(nmeaBuffer);
      // static: ~4.6KB struct, too large for the core stack
      static meshtastic_InterdeviceMessage myPacket;
      memset(&myPacket, 0, sizeof(myPacket));
      myPacket.which_data = meshtastic_InterdeviceMessage_nmea_tag;
      myPacket.data.nmea[0] = '\0'; // Ensure the string is null
      strncpy(myPacket.data.nmea, nmeaBuffer, bufferIndex);
      myPacket.data.nmea[bufferIndex] = '\0'; // Explicitly null-terminate the string  
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
