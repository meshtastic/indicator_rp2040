#pragma once

#include "meshtastic/interdevice.pb.h"
#include <Arduino.h>
#include <pb_decode.h>
#include <pb_encode.h>

// Magic number at the start of all MT packets
#define MT_MAGIC_0 0x94
#define MT_MAGIC_1 0xc3

// The header is the magic number plus a 16-bit payload-length field
#define MT_HEADER_SIZE 4

bool mt_send_uplink(const meshtastic_InterdeviceMessage &message);
// unsolicited ping at boot, so the main firmware notices us coming (back) up
bool mt_send_hello(void);
void mt_set_nmea_callback(void (*callback)(char *nmea));
void mt_loop();

// SD card access arbitration between the cores, implemented in main.cpp:
// core1 owns mounting and the background stats scan, core0 claims the card
// per request and fails fast instead of stalling the link
enum { SD_CLAIM_OK = 0, SD_CLAIM_NOCARD, SD_CLAIM_BUSY };
int sd_claim(void);
void sd_release(void);
// a card operation looked wrong: have core1 check whether the card is still
// there (SdFat answers from cached state, only a re-init asks the card)
void sd_request_verify(void);
// release the card so it can be pulled safely, mount it again, or wipe it
void sd_request_eject(void);
void sd_request_mount(void);
void sd_request_format(void);
// drops cached file handles; called with the card mutex held, right before
// core1 unmounts (implemented next to the caches, in serial_proto.cpp)
void sd_close_cached_files(void);
// a write changed the filesystem by this many bytes (negative when it freed
// them); keeps used/free current without another full FAT walk
void sd_account_bytes(int64_t delta);
void sd_get_info(meshtastic_SdCardInfo *out);
// bumped on every unmount, open file handles do not survive a remount
extern volatile uint32_t sd_generation;