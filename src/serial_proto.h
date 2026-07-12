#pragma once

#ifndef SERIAL_PROTO_H
#define SERIAL_PROTO_H

#include "meshtastic/interdevice.pb.h"
#include <Arduino.h>
#include <pb_decode.h>
#include <pb_encode.h>

// Magic number at the start of all MT packets
#define MT_MAGIC_0 0x94
#define MT_MAGIC_1 0xc3

// The header is the magic number plus a 16-bit payload-length field
#define MT_HEADER_SIZE 4

// Wait this many msec if there's nothing new on the channel
#define NO_NEWS_PAUSE 25

bool mt_send_uplink(const meshtastic_InterdeviceMessage &message);
void mt_set_nmea_callback(void (*callback)(char *nmea));
void mt_loop();

// SD card access arbitration between the cores, implemented in main.cpp:
// core1 owns mounting and the background stats scan, core0 claims the card
// per request and fails fast instead of stalling the link
enum { SD_CLAIM_OK = 0, SD_CLAIM_NOCARD, SD_CLAIM_BUSY };
int sd_claim(void);
void sd_release(void);
void sd_mark_dead(void);
void sd_note_write(void);
void sd_get_info(meshtastic_SdCardInfo *out);
// bumped on every unmount, open file handles do not survive a remount
extern volatile uint32_t sd_generation;

#endif