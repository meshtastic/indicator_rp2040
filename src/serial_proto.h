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

bool mt_send_uplink(meshtastic_InterdeviceMessage message);

// Set the callback function that gets called when the node receives a sensor
// control message.
void mt_set_sensor_callback(void (*callback)(meshtastic_SensorData sensor));

// Set callback function that gets called when nmea message is received
void mt_set_nmea_callback(void (*callback)(char *nmea));

void mt_loop();

#endif