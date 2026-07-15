#include "buzzer.h"
#include "serial_proto.h"
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <SdFat.h>
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
// a mount attempt is running (or has not been tried yet): the card is not
// usable right now, but whether one is present is not decided either
static volatile bool sd_mounting = true;
// core1 should re-init the card to see whether it is still in the slot
static volatile bool sd_verify_pending = false;
// The card was ejected on request: it stays released (no automatic remount)
// so it can be pulled safely, until a mount is asked for.
static volatile bool sd_ejected = false;
static volatile bool sd_eject_pending = false;
static volatile bool sd_mount_requested = false;
static volatile bool sd_format_requested = false;
// a card answers in the slot but has no filesystem we can mount
static volatile bool sd_unformatted = false;
// bumped on every unmount so serial_proto drops file handles that cannot
// survive a remount
volatile uint32_t sd_generation = 0;

// last time core0 touched the card; core1 keeps its multi-second
// maintenance off the card while requests are still flowing
static volatile uint32_t sd_last_access = 0;

// used/free statistics from the core1 scan
static volatile bool sd_stats_pending = false;
static volatile uint32_t stats_pending_since = 0;
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
  if (sd_ejected)
    return SD_CLAIM_NOCARD; // released on request, do not touch it
  // Checked before the try-lock: core1 holds the mutex for the whole
  // (2 second) mount attempt, and answering BUSY for an empty slot would
  // make it look like a card that is merely busy. While a mount attempt is
  // actually running, though, the outcome is not known yet: BUSY tells the
  // requester to come back rather than to give up on the card.
  if (!sd_mounted)
    return sd_mounting ? SD_CLAIM_BUSY : SD_CLAIM_NOCARD;
  if (!mutex_try_enter(&sd_mutex, NULL))
    return SD_CLAIM_BUSY;
  if (!sd_mounted) {
    mutex_exit(&sd_mutex);
    return sd_mounting ? SD_CLAIM_BUSY : SD_CLAIM_NOCARD;
  }
  sd_last_access = millis(); // keeps core1 off the card while requests flow
  return SD_CLAIM_OK;
}

void sd_release(void) {
  sd_last_access = millis();
  mutex_exit(&sd_mutex);
}

// Something looked wrong with the card, or nothing has been asked of it for
// a while: have core1 verify it is still there. Cheap and rate limited, so
// callers can ask on any suspicious failure.
void sd_request_verify(void) { sd_verify_pending = true; }

// Flush the card and release it so it can be pulled without corrupting it.
// It stays released until sd_request_mount().
void sd_request_eject(void) {
  sd_ejected = true; // reported as gone right away, core1 tears it down
  sd_eject_pending = true;
}

// Mount whatever is in the slot, also after an eject. Mounting a card that is
// already mounted is a no-op: it must not report itself busy, because only the
// mount path below clears that again.
void sd_request_mount(void) {
  sd_ejected = false;
  if (sd_mounted)
    return;
  sd_mounting = true;        // reported as busy until core1 knows better
  sd_mount_requested = true; // without waiting out the retry interval
}

// Put a fresh filesystem on the card. Takes seconds and destroys everything
// on it; the card is mounted again afterwards.
void sd_request_format(void) {
  sd_ejected = false;
  sd_mounting = true;
  sd_format_requested = true;
}

// A write changed the filesystem by this many bytes (negative when it freed
// them). The full scan behind used/free walks the whole FAT and holds the
// card for seconds, which would starve the map tiles the writes belong to,
// so keep the numbers current by accounting instead of rescanning. Cluster
// slack is ignored: this is a UI readout, not a budget.
// Called from a file operation, i.e. with the card mutex already held (it is
// not recursive, so this must not take it again).
void sd_account_bytes(int64_t delta) {
  if (!sd_stats_valid || delta == 0)
    return;
  if (delta > 0) {
    uint64_t d = (uint64_t)delta;
    sd_used_bytes += d;
    sd_free_bytes = sd_free_bytes > d ? sd_free_bytes - d : 0;
  } else {
    uint64_t d = (uint64_t)(-delta);
    sd_used_bytes = sd_used_bytes > d ? sd_used_bytes - d : 0;
    sd_free_bytes += d;
  }
}

// Snapshot for get_sd_info, answered entirely from cached state so the
// response never waits on the card
void sd_get_info(meshtastic_SdCardInfo *out) {
  memset(out, 0, sizeof(*out));
  if (sd_ejected)
    return; // released on request: gone as far as anyone else is concerned
  if (!sd_mounted) {
    // asked before anything else: a mounted card is never busy, whatever a
    // stale flag might say, and a caller that waits out a busy card forever
    // is a frozen screen
    if (sd_mounting) {
      // whether a card is present is not decided yet, so asking again beats
      // reporting an empty slot
      out->busy = true;
    } else if (sd_unformatted) {
      // a card is in there, it just has nothing we can mount
      out->unformatted = true;
    }
    return;
  }
  // Never waits for the card: core1 can hold the mutex for seconds. The
  // identity cache is only rewritten while unmounted, so a generation that
  // did not change across the copy proves it did not come from two cards.
  uint32_t generation = sd_generation;
  *out = sd_info_cache;
  if (generation != sd_generation || !sd_mounted) {
    memset(out, 0, sizeof(*out));
    out->busy = true; // remounting right now
    return;
  }
  out->present = true;
  if (sd_stats_valid) {
    out->used_bytes = sd_used_bytes;
    out->free_bytes = sd_free_bytes;
    out->stats_valid = true;
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

// The card sits on SPI1, and every raw card access has to say so: a config
// without a port falls back to SPI0, whose default pins carry the serial link
// to the main firmware (16/17), so it would tear the link down.
static SdSpiConfig sd_spi_config(void) {
  return SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);
}

// Does a card answer in the slot at all, filesystem or not? SD.begin() cannot
// tell an empty slot from a card without a filesystem, but the card responds
// to the SPI init either way. Caller holds the mutex, card must be unmounted.
static bool sd_card_responds(void) {
  SdCardFactory factory;
  SdCard *card = factory.newCard(sd_spi_config());
  return card && !card->errorCode();
}

// Put a fresh FAT on the card. Not SDFS.format(): that one builds its card
// config without a SPI port, so it would run on SPI0 and re-mux the pins of
// the interdevice link. Caller holds the mutex, card must be unmounted.
static bool sd_format_card(void) {
  SdCardFactory factory;
  SdCard *card = factory.newCard(sd_spi_config());
  if (!card || card->errorCode())
    return false;
  static uint8_t sector[512]; // too big for the core stack
  FatFormatter formatter;
  return formatter.format(card, sector, nullptr);
}

// Is the card still in the slot? SdFat answers type()/fatType() and even a
// root directory lookup from state cached at mount time, so the only way to
// ask the card itself is to initialize it again: that clocks CMD0/ACMD41
// over SPI and fails when the slot is empty. Caller must be core1.
static bool sd_verify_card(void) {
  uint64_t was = sd_info_cache.card_size;
  bool ok;
  mutex_enter_blocking(&sd_mutex);
  // the handles and cached sectors belong to the volume we are about to
  // tear down, whatever the outcome
  sd_close_cached_files();
  sd_generation++;
  SD.end(false);
  ok = SD.begin(SD_CS_PIN, 16000000, SPI1);
  if (ok) {
    sd_cache_card_info();
    // the card stays mounted across a verify, so core0 may be reading the
    // identity cache while it is rewritten here: bumping again makes such a
    // copy visibly straddle two generations and be discarded
    sd_generation++;
    if (sd_info_cache.card_size != was) {
      // a different card: the free space we know belongs to the old one
      sd_stats_valid = false;
      sd_stats_pending = true;
      stats_pending_since = millis();
      Serial.println("SD card replaced");
    }
  } else {
    sd_mounted = false;
    sd_stats_valid = false;
  }
  mutex_exit(&sd_mutex);
  if (!ok)
    Serial.println("SD card removed");
  return ok;
}

void loop1() {
  static uint32_t last_mount = 0;
  static uint32_t last_verify = 0;

  if (sd_mount_requested) {
    sd_mount_requested = false;
    last_mount = 0; // try right away, whatever the retry interval says
  }

  // only the mount attempt below can clear this, so a mounted card must never
  // be left reporting itself busy
  if (sd_mounted && sd_mounting)
    sd_mounting = false;

  // Ejected on request: flush and release the card, then leave the slot alone
  // so it can be pulled. Only a mount request brings it back.
  if (sd_eject_pending) {
    sd_eject_pending = false;
    if (sd_mounted) {
      mutex_enter_blocking(&sd_mutex);
      sd_mounted = false;
      sd_stats_valid = false;
      sd_generation++;
      sd_close_cached_files(); // flushes what is still open
      SD.end(false);
      mutex_exit(&sd_mutex);
    }
    Serial.println("SD card ejected, safe to remove");
  }

  // The slot has no card-detect line, so a card that is pulled while nothing
  // is being read goes unnoticed until something fails. Ask the card itself,
  // but only while core0 is not using it: a verify re-initializes the card,
  // which drops the read cache and costs a few ms (and a full 2 second
  // timeout once, when the card is really gone).
  if (sd_mounted && !sd_ejected &&
      (sd_verify_pending || millis() - last_verify >= 5000) &&
      millis() - sd_last_access >= 1000) {
    sd_verify_pending = false;
    last_verify = millis();
    if (!sd_verify_card())
      last_mount = 0; // look for a new card right away
  }

  // Formatting needs the card unmounted, and destroys everything on it. Only
  // ever on request, never as a reaction to a card that fails to mount.
  if (sd_format_requested) {
    sd_format_requested = false;
    mutex_enter_blocking(&sd_mutex);
    if (sd_mounted) {
      sd_mounted = false;
      sd_generation++;
      sd_close_cached_files();
      SD.end(false);
    }
    sd_stats_valid = false;
    bool ok = sd_format_card();
    mutex_exit(&sd_mutex);
    Serial.println(ok ? "SD card formatted" : "SD card format failed");
    sd_unformatted = !ok;
    last_mount = 0; // mount the fresh filesystem
  }

  if (!sd_mounted && !sd_ejected) {
    // The Indicator's slot has no card-detect line, so retry rate-limited;
    // this makes inserting (or re-inserting) a card after boot just work
    if (last_mount == 0 || millis() - last_mount >= 2000) {
      bool ok;
      sd_mounting = true; // requests get BUSY, not "no card", until we know
      mutex_enter_blocking(&sd_mutex);
      if ((ok = SD.begin(SD_CS_PIN, 16000000, SPI1))) {
        sd_cache_card_info();
        sd_stats_valid = false;
        sd_stats_pending = true;
        stats_pending_since = millis();
        sd_unformatted = false;
      } else {
        // mount failed: a card that answers but carries no filesystem is not
        // an empty slot, and the user can format it
        sd_unformatted = sd_card_responds();
      }
      mutex_exit(&sd_mutex);
      if (ok) {
        // published after the info cache is fully written
        sd_mounted = true;
        sd_last_access = millis(); // the idle window starts here
        Serial.println("card initialized.");
      } else if (sd_unformatted) {
        Serial.println("SD card has no filesystem");
      }
      sd_mounting = false;
      // stamped after the attempt: begin() spins for its full timeout
      // when the slot is empty
      last_mount = millis();
    }
  } else if (sd_mounted && sd_stats_pending) {
    // The free space scan walks the whole FAT (SdFat keeps no cached cluster
    // count) and holds the card for seconds, so it runs once per mount and
    // only when core0 has not touched the card for a while: serving map
    // tiles beats knowing the free space. Later writes keep the numbers
    // current by accounting (sd_account_bytes), so no rescan follows them.
    //
    // A steady stream of tile reads would keep the card busy forever, so
    // stop waiting for an idle window after a while and just take it: the
    // requester retries the BUSY responses that costs.
    bool waited_long_enough = millis() - stats_pending_since >= 30000;
    if (millis() - sd_last_access >= 3000 || waited_long_enough) {
      FSInfo info;
      mutex_enter_blocking(&sd_mutex);
      SDFS.info(info);
      // info() reports success even when the FAT walk failed, and then
      // returns a used count that wraps past the card size
      bool ok = info.totalBytes > 0 && info.usedBytes <= info.totalBytes;
      if (ok) {
        sd_used_bytes = info.usedBytes;
        sd_free_bytes = info.totalBytes - info.usedBytes;
        sd_stats_valid = true;
        sd_stats_pending = false;
      } else {
        sd_close_cached_files();
      }
      mutex_exit(&sd_mutex);
      if (!ok)
        sd_verify_pending = true; // a FAT walk only fails when the card is gone
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
  // SerialPIO::availableForWrite() returns 8 minus the fill level of a
  // 4 deep TX FIFO, so it never reaches 0 and cannot be used to detect a
  // full FIFO. write() blocks once the FIFO is full, so hand it a few
  // bytes per pass and let the next pass continue: at 9600 baud a full
  // ring would otherwise block the loop for two seconds.
  const int fifo_depth = 4;
  int free_slots = softSerial2.availableForWrite() - (8 - fifo_depth);
  while (gps_tx_head != gps_tx_tail && free_slots-- > 0) {
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

  // Announce ourselves: the main firmware may have finished its own boot
  // (and its peripheral scan) while we were still starting, and it has no
  // other way of noticing that we came up or rebooted.
  mt_send_hello();

  // This MCU is the sole path to GPS, sensors and SD card; a wedged
  // peripheral driver must reboot it rather than take them down for good.
  // The request loop on core0 feeds the watchdog, core1 may block freely.
  rp2040.wdt_begin(4000);
}

void loop() {

  // watchdog: this loop must never stall, all blocking work is on core1
  rp2040.wdt_reset();

  if (buzz_off != 0 && (int32_t)(millis() - buzz_off) >= 0) {
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
