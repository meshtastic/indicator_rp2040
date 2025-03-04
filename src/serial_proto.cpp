#include "serial_proto.h"

// The buffer used for protobuf encoding/decoding. Since there's only one, and
// it's global, we have to make sure we're only ever doing one encoding or
// decoding at a time.

#define PB_BUFSIZE meshtastic_InterdeviceMessage_size + MT_HEADER_SIZE

pb_byte_t pb_rx_buf[PB_BUFSIZE];
size_t pb_rx_size = 0; // Number of bytes currently in the buffer

pb_byte_t pb_tx_buf[PB_BUFSIZE];

void (*sensor_callback)(meshtastic_SensorData sensor) = NULL;
void (*nmea_callback)(char *nmea) = NULL;

bool mt_send(const char *buf, size_t len) {
  size_t wrote = Serial1.write(buf, len);
  if (wrote == len)
    return true;
  return false;
}

// Parse a packet that came in, and handle it. Return true if we were able to
// parse it.
bool mt_handle_packet(size_t payload_len) {
  meshtastic_InterdeviceMessage message =
      meshtastic_InterdeviceMessage_init_zero;

  // Decode the protobuf and shift forward any remaining bytes in the buffer
  // (which, if present, belong to the packet that we're going to process on the
  // next loop)
  pb_istream_t stream =
      pb_istream_from_buffer(pb_rx_buf + MT_HEADER_SIZE, payload_len);
  bool status =
      pb_decode(&stream, meshtastic_InterdeviceMessage_fields, &message);
  memmove(pb_rx_buf, pb_rx_buf + MT_HEADER_SIZE + payload_len,
          PB_BUFSIZE - MT_HEADER_SIZE - payload_len);
  pb_rx_size -= MT_HEADER_SIZE + payload_len;

  if (!status) {
    Serial.println("Decoding failed");
    return false;
  }

  switch (message.which_data) {
  case meshtastic_InterdeviceMessage_sensor_tag:
    if (sensor_callback != NULL)
      sensor_callback(message.data.sensor);
    return true;
    break;
  case meshtastic_InterdeviceMessage_nmea_tag:
    if (nmea_callback != NULL)
      nmea_callback(message.data.nmea);
    return true;
    break;
  default:
    // the other messages really only flow downstream
    Serial.println("Got a message of unexpected type");
    return false;
  }
}

void mt_check_packet() {
  if (pb_rx_size < MT_HEADER_SIZE) {
    // We don't even have a header yet
    delay(NO_NEWS_PAUSE);
    return;
  }

  if (pb_rx_buf[0] != MT_MAGIC_0 || pb_rx_buf[1] != MT_MAGIC_1) {
    Serial.println("Got bad magic");
    memset(pb_rx_buf, 0, PB_BUFSIZE);
    pb_rx_size = 0;
    return;
  }

  uint16_t payload_len = pb_rx_buf[2] << 8 | pb_rx_buf[3];
  if (payload_len > PB_BUFSIZE) {
    Serial.println("Got packet claiming to be ridiculous length");
    return;
  }

  if ((size_t)(payload_len + MT_HEADER_SIZE) > pb_rx_size) {
    delay(NO_NEWS_PAUSE);
    return;
  }

  // We have a complete packet, handle it
  mt_handle_packet(payload_len);
}

size_t mt_serial_check(char *buf, size_t space_left) {
  size_t bytes_read = 0;
  while (Serial1.available()) {
    char c = Serial1.read();
    *buf++ = c;
    if (++bytes_read >= space_left) {
      Serial.println("Serial overflow");
      break;
    }
  }
  return bytes_read;
}

bool mt_send_uplink(meshtastic_InterdeviceMessage message) {
  pb_tx_buf[0] = MT_MAGIC_0;
  pb_tx_buf[1] = MT_MAGIC_1;

  pb_ostream_t stream =
      pb_ostream_from_buffer(pb_tx_buf + MT_HEADER_SIZE, PB_BUFSIZE);
  if (!pb_encode(&stream, meshtastic_InterdeviceMessage_fields, &message)) {
    Serial.println("pb_encode failed");
    return false;
  }

  // Store the payload length in the header
  pb_tx_buf[2] = stream.bytes_written / 256;
  pb_tx_buf[3] = stream.bytes_written % 256;

  bool rv =
      mt_send((const char *)pb_tx_buf, MT_HEADER_SIZE + stream.bytes_written);

  return rv;
}

void mt_loop() {
  size_t bytes_read = 0;

  // See if there are any more bytes to add to our buffer.
  size_t space_left = PB_BUFSIZE - pb_rx_size;

  bytes_read = mt_serial_check((char *)pb_rx_buf + pb_rx_size, space_left);

  pb_rx_size += bytes_read;
  mt_check_packet();
}

void mt_set_sensor_callback(void (*callback)(meshtastic_SensorData sensor)) {
  sensor_callback = callback;
}

void mt_set_nmea_callback(void (*callback)(char *nmea)) {
  nmea_callback = callback;
}