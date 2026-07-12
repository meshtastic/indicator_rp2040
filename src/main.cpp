#include "buzzer.h"
#include "serial_proto.h"
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <pico/mutex.h>

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

const int SD_CS_PIN = 13;

// All SD card access is arbitrated between the two cores with this mutex.
// Core1 owns mounting, card-removal recovery and the stats scan, all of
// which can block for seconds (SD.begin spins its full timeout on an empty
// slot, the FAT scan behind usedBytes walks the whole FAT). Core0 claims
// the mutex per request with a try-lock and reports "SD busy" instead of
// stalling the link, so GPS forwarding and other requests keep flowing.
auto_init_mutex(sd_mutex);
static volatile bool sd_mounted = false;
static volatile bool sd_recheck = false;
// bumped on every unmount so serial_proto drops file handles that cannot
// survive a remount
volatile uint32_t sd_generation = 0;

// used/free statistics from the core1 scan
static volatile bool sd_stats_pending = false;
static bool sd_stats_valid = false;
static uint64_t sd_used_bytes = 0;
static uint64_t sd_free_bytes = 0;
// card identity, cached at mount time so get_sd_info never touches the card
static meshtastic_SdCardInfo sd_info_cache;

// ---- core0 interface (used by serial_proto.cpp) ----

// Claim exclusive card access for one request. SD_CLAIM_BUSY means core1
// is mounting or scanning right now; the caller reports that instead of
// waiting for it.
int sd_claim(void) {
  if (!mutex_try_enter(&sd_mutex, NULL))
    return SD_CLAIM_BUSY;
  if (!sd_mounted) {
    mutex_exit(&sd_mutex);
    return SD_CLAIM_NOCARD;
  }
  return SD_CLAIM_OK;
}

void sd_release(void) { mutex_exit(&sd_mutex); }

// A card-level failure was observed on a mounted card: assume it was
// pulled and have core1 recover with a clean remount
void sd_mark_dead(void) { sd_recheck = true; }

// A write changed the filesystem; refresh the stats (rate-limited on core1)
void sd_note_write(void) { sd_stats_pending = true; }

// Snapshot for get_sd_info, answered entirely from cached state so the
// response never waits on the card
void sd_get_info(meshtastic_SdCardInfo *out) {
  memset(out, 0, sizeof(*out));
  if (!sd_mounted)
    return;
  *out = sd_info_cache; // stable while mounted
  out->present = true;
  // used/free stay zero while the scan is still running (mutex held by
  // core1) or has not completed yet
  if (mutex_try_enter(&sd_mutex, NULL)) {
    if (sd_stats_valid) {
      out->used_bytes = sd_used_bytes;
      out->free_bytes = sd_free_bytes;
    }
    mutex_exit(&sd_mutex);
  }
}

// ---- core1: mount, recovery and stats scan ----

// caller holds sd_mutex, card is freshly mounted
static void sd_cache_card_info(void) {
  memset(&sd_info_cache, 0, sizeof(sd_info_cache));
  sd_info_cache.card_size = SD.size64();
  switch (SD.fatType()) {
  case 16:
    sd_info_cache.fat_type = meshtastic_SdCardInfo_FatType_FAT16;
    break;
  case 32:
    sd_info_cache.fat_type = meshtastic_SdCardInfo_FatType_FAT32;
    break;
  case 64:
    sd_info_cache.fat_type = meshtastic_SdCardInfo_FatType_EXFAT;
    break;
  default:
    sd_info_cache.fat_type = meshtastic_SdCardInfo_FatType_UNKNOWN_FAT;
  }
  // SdFat card types: 1 = SD1, 2 = SD2, 3 = SDHC/SDXC (differ by capacity)
  switch (SD.type()) {
  case 1:
  case 2:
    sd_info_cache.card_type = meshtastic_SdCardInfo_CardType_SD;
    break;
  case 3:
    sd_info_cache.card_type =
        sd_info_cache.card_size > 32ull * 1000 * 1000 * 1000
            ? meshtastic_SdCardInfo_CardType_SDXC
            : meshtastic_SdCardInfo_CardType_SDHC;
    break;
  default:
    sd_info_cache.card_type = meshtastic_SdCardInfo_CardType_UNKNOWN_CARD;
  }
}

void setup1() {
  // SD card slot is on SPI1, used exclusively under sd_mutex
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
}

void loop1() {
  static uint32_t last_mount = 0;
  static uint32_t last_scan = 0;

  if (sd_recheck) {
    mutex_enter_blocking(&sd_mutex);
    sd_mounted = false;
    sd_stats_valid = false;
    sd_generation++;
    SD.end(false);
    mutex_exit(&sd_mutex);
    sd_recheck = false;
    last_mount = 0; // retry immediately
    Serial.println("SD card lost, remounting");
  }

  if (!sd_mounted) {
    // The Indicator's slot has no card-detect line, so retry rate-limited;
    // this makes inserting (or re-inserting) a card after boot just work
    if (last_mount == 0 || millis() - last_mount >= 2000) {
      bool ok;
      mutex_enter_blocking(&sd_mutex);
      if ((ok = SD.begin(SD_CS_PIN, 16000000, SPI1))) {
        sd_cache_card_info();
        sd_stats_valid = false;
        sd_stats_pending = true;
      }
      mutex_exit(&sd_mutex);
      if (ok) {
        // published after the info cache is fully written
        sd_mounted = true;
        Serial.println("card initialized.");
      }
      // stamped after the attempt: begin() spins for its full timeout
      // when the slot is empty
      last_mount = millis();
    }
  } else if (sd_stats_pending) {
    // initial scan right after mount, write-triggered refreshes at most
    // once a minute (each scan blocks file requests for its duration)
    uint32_t interval = sd_stats_valid ? 60000 : 0;
    if (last_scan == 0 || millis() - last_scan >= interval) {
      FSInfo info;
      mutex_enter_blocking(&sd_mutex);
      bool ok = SDFS.info(info);
      if (ok) {
        sd_used_bytes = info.usedBytes;
        sd_free_bytes = info.totalBytes - info.usedBytes;
        sd_stats_valid = true;
        sd_stats_pending = false;
      }
      mutex_exit(&sd_mutex);
      if (!ok)
        sd_recheck = true; // info failing on a mounted card = card gone
      last_scan = millis();
    }
  }
  delay(20);
}

// The built-in sensor needs to be powered on
void sensor_power_on(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
}

/************************ beep ****************************/

// Downlink NMEA (ESP32 -> GPS chip) is buffered and drained a few bytes
// per loop() pass: SerialPIO::write blocks at 9600 baud once its small
// hardware FIFO is full, so printing a whole sentence inline would stall
// the loop for tens of ms to a second per message
static uint8_t gps_tx_ring[2048];
static size_t gps_tx_head = 0, gps_tx_tail = 0;

void onNmeaReceived(char *nmea) {
  for (const char *p = nmea; *p; p++) {
    size_t next = (gps_tx_head + 1) % sizeof(gps_tx_ring);
    if (next == gps_tx_tail)
      break; // ring full: drop the rest, the NMEA stream self-heals
    gps_tx_ring[gps_tx_head] = *p;
    gps_tx_head = next;
  }
}

static void gps_tx_drain(void) {
  while (gps_tx_head != gps_tx_tail && softSerial2.availableForWrite() > 0) {
    softSerial2.write(gps_tx_ring[gps_tx_tail]);
    gps_tx_tail = (gps_tx_tail + 1) % sizeof(gps_tx_ring);
  }
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
  // a hung bus must not stall the executor, reset it on timeout
  Wire.setTimeout(50, true);

  // the SD card lives on core1, see setup1()/loop1()

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

  gps_tx_drain();
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
      myPacket.data.nmea[bufferIndex] =
          '\0'; // Explicitly null-terminate the string
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
