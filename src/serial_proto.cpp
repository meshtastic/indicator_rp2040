#include "serial_proto.h"
#include "buzzer.h"
#include <SD.h>
#include <Wire.h>

// One-entry read cache: map tiles are fetched in sequential 4K chunks, so
// keeping the file open between chunks avoids a FAT open + seek per chunk
static File read_cache;
static char read_cache_path[256] = "";

// Directory entry count from the last full walk, so follow-up pages of the
// same listing don't have to walk to the end of the directory again
static char count_cache_path[256] = "";
static uint32_t count_cache_total = 0;

static void invalidate_read_cache() {
  if (read_cache)
    read_cache.close();
  read_cache_path[0] = '\0';
}

// call before any card access: file handles do not survive a remount
static void check_sd_generation() {
  static uint32_t seen_generation = 0;
  if (seen_generation != sd_generation) {
    invalidate_read_cache();
    count_cache_path[0] = '\0';
    seen_generation = sd_generation;
  }
}

// The buffer used for protobuf encoding/decoding. Since there's only one, and
// it's global, we have to make sure we're only ever doing one encoding or
// decoding at a time.

#define PB_BUFSIZE meshtastic_InterdeviceMessage_size + MT_HEADER_SIZE

pb_byte_t pb_rx_buf[PB_BUFSIZE];
size_t pb_rx_size = 0; // Number of bytes currently in the buffer

pb_byte_t pb_tx_buf[PB_BUFSIZE];

// Statically allocated message structs: with 4KB file chunks an
// InterdeviceMessage is ~4.6KB, far too large for the 8KB core stack.
// The loop is single threaded, so one of each suffices.
static meshtastic_InterdeviceMessage rx_message;
static meshtastic_InterdeviceMessage tx_response;

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
  meshtastic_InterdeviceMessage &message = rx_message;
  memset(&message, 0, sizeof(message));

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
  case meshtastic_InterdeviceMessage_nmea_tag:
    if (nmea_callback != NULL)
      nmea_callback(message.data.nmea);
    return true;

  case meshtastic_InterdeviceMessage_beep_tag:
    // Handle the beep command
    if (message.data.beep > 0) {
      beep_on(message.data.beep);
    } else {
      beep_off();
    }
    return true;
  case meshtastic_InterdeviceMessage_i2c_transaction_tag: {
    // Execute a tunneled I2C transaction: an optional write followed by an
    // optional read with repeated start
    meshtastic_I2CTransaction transaction = message.data.i2c_transaction;
    meshtastic_InterdeviceMessage &response = tx_response;
    memset(&response, 0, sizeof(response));
    response.id = message.id; // correlate with the request
    response.which_data = meshtastic_InterdeviceMessage_i2c_result_tag;
    meshtastic_I2CResult *result = &response.data.i2c_result;
    result->status = meshtastic_I2CResult_Status_OK;

    if (transaction.write_data.size > 0 || transaction.read_len == 0) {
      Wire.beginTransmission((uint8_t)transaction.address);
      Wire.write(transaction.write_data.bytes, transaction.write_data.size);
      // keep the bus claimed (repeated start) when a read follows
      switch (Wire.endTransmission(transaction.read_len == 0)) {
      case 0:
        break;
      case 2:
        result->status = meshtastic_I2CResult_Status_NACK_ADDRESS;
        break;
      case 3:
        result->status = meshtastic_I2CResult_Status_NACK_DATA;
        break;
      default:
        result->status = meshtastic_I2CResult_Status_ERROR;
      }
    }

    if (result->status == meshtastic_I2CResult_Status_OK &&
        transaction.read_len > 0) {
      size_t want = transaction.read_len;
      if (want > sizeof(result->read_data.bytes))
        want = sizeof(result->read_data.bytes);
      size_t got = Wire.requestFrom((uint8_t)transaction.address, want);
      if (got == 0)
        result->status = meshtastic_I2CResult_Status_NACK_ADDRESS;
      while (Wire.available() && result->read_data.size < got)
        result->read_data.bytes[result->read_data.size++] = Wire.read();
    }
    return mt_send_uplink(response);
  }

  case meshtastic_InterdeviceMessage_file_transfer_tag: {
    // SD card file operations. GET is a ranged chunk read. Writes are
    // sequential: POST starts a new file (offset 0), PUT appends a chunk
    // at the current end of the file.
    meshtastic_FileTransfer &request = message.data.file_transfer;
    meshtastic_InterdeviceMessage &response = tx_response;
    memset(&response, 0, sizeof(response));
    response.id = message.id; // correlate with the request
    response.which_data = meshtastic_InterdeviceMessage_file_transfer_tag;
    meshtastic_FileTransfer *out = &response.data.file_transfer;
    out->operation = request.operation;
    strncpy(out->filepath, request.filepath, sizeof(out->filepath) - 1);
    out->offset = request.offset;
    out->success = false;

    int claim = sd_claim();
    if (claim != SD_CLAIM_OK) {
      strncpy(out->message, claim == SD_CLAIM_BUSY ? "SD busy" : "no SD card",
              sizeof(out->message) - 1);
      return mt_send_uplink(response);
    }
    check_sd_generation();

    switch (request.operation) {
    case meshtastic_FileOperation_GET: {
      if (!read_cache || strcmp(read_cache_path, request.filepath) != 0) {
        invalidate_read_cache();
        read_cache = SD.open(request.filepath, FILE_READ);
        if (!read_cache) {
          strncpy(out->message, "open failed", sizeof(out->message) - 1);
          break;
        }
        strncpy(read_cache_path, request.filepath, sizeof(read_cache_path) - 1);
      }
      out->file_size = read_cache.size();
      size_t want = request.length;
      if (want == 0 || want > sizeof(out->filedata.bytes))
        want = sizeof(out->filedata.bytes);
      if (request.offset > 0xFFFFFFFFull) {
        // the FS layer seeks with 32 bit positions
        strncpy(out->message, "offset too large", sizeof(out->message) - 1);
        break;
      }
      if (!read_cache.seek((uint32_t)request.offset)) {
        strncpy(out->message, "seek failed", sizeof(out->message) - 1);
        sd_mark_dead(); // seek on an open handle only fails at card level
        break;
      }
      int got = read_cache.read(out->filedata.bytes, want);
      if (got < 0) {
        strncpy(out->message, "read failed", sizeof(out->message) - 1);
        sd_mark_dead();
        break;
      }
      out->filedata.size = got;
      out->success = true;
      break;
    }

    case meshtastic_FileOperation_POST:
    case meshtastic_FileOperation_PUT: {
      invalidate_read_cache();
      if (request.operation == meshtastic_FileOperation_POST &&
          SD.exists(request.filepath))
        SD.remove(request.filepath);
      File f = SD.open(request.filepath, FILE_WRITE); // append mode, creates
      if (!f) {
        strncpy(out->message, "open failed", sizeof(out->message) - 1);
        // creating a file on a healthy card does not fail; assume the
        // card was pulled and let core1 verify with a remount
        sd_mark_dead();
        break;
      }
      if (f.size() != request.offset) {
        snprintf(out->message, sizeof(out->message),
                 "bad offset %u, file size %u", (unsigned)request.offset,
                 (unsigned)f.size());
        out->file_size = f.size(); // lets the client resync its offset
        f.close();
        break;
      }
      size_t wrote = f.write(request.filedata.bytes, request.filedata.size);
      out->file_size = f.size();
      f.close();
      out->success = (wrote == request.filedata.size);
      if (!out->success) {
        strncpy(out->message, "write failed", sizeof(out->message) - 1);
        sd_mark_dead();
      } else {
        sd_note_write();
      }
      count_cache_path[0] = '\0';
      break;
    }

    case meshtastic_FileOperation_DELETE:
      invalidate_read_cache();
      out->success = SD.remove(request.filepath);
      if (!out->success)
        strncpy(out->message, "remove failed", sizeof(out->message) - 1);
      else
        sd_note_write();
      count_cache_path[0] = '\0';
      break;

    default:
      strncpy(out->message, "unknown operation", sizeof(out->message) - 1);
    }
    sd_release();
    return mt_send_uplink(response);
  }

  case meshtastic_InterdeviceMessage_directory_listing_tag: {
    // Paged directory listing, subdirectories get a trailing slash
    meshtastic_DirectoryListing &request = message.data.directory_listing;
    meshtastic_InterdeviceMessage &response = tx_response;
    memset(&response, 0, sizeof(response));
    response.id = message.id; // correlate with the request
    response.which_data = meshtastic_InterdeviceMessage_directory_listing_tag;
    meshtastic_DirectoryListing *out = &response.data.directory_listing;
    strncpy(out->directory, request.directory, sizeof(out->directory) - 1);
    out->offset = request.offset;
    out->success = false;

    int claim = sd_claim();
    if (claim != SD_CLAIM_OK) {
      strncpy(out->message, claim == SD_CLAIM_BUSY ? "SD busy" : "no SD card",
              sizeof(out->message) - 1);
      return mt_send_uplink(response);
    }
    check_sd_generation();

    File dir = SD.open(request.directory);
    if (!dir || !dir.isDirectory()) {
      strncpy(out->message, "not a directory", sizeof(out->message) - 1);
    } else {
      const size_t max_names =
          sizeof(out->filenames) / sizeof(out->filenames[0]);
      // Walking to the end of the directory just to count the entries is
      // expensive (a full LFN-parsing walk per page); do it once on the
      // first page and reuse the count for the follow-up pages
      bool have_total = request.offset > 0 &&
                        strcmp(count_cache_path, request.directory) == 0;
      uint32_t index = 0;
      while (true) {
        File entry = dir.openNextFile();
        if (!entry)
          break;
        if (index >= request.offset && out->filenames_count < max_names) {
          snprintf(out->filenames[out->filenames_count],
                   sizeof(out->filenames[0]), "%s%s", entry.name(),
                   entry.isDirectory() ? "/" : "");
          out->filenames_count++;
        }
        index++;
        entry.close();
        if (have_total && out->filenames_count >= max_names)
          break; // page full, the total is already known
      }
      if (have_total) {
        out->total_count = count_cache_total;
      } else {
        out->total_count = index;
        strncpy(count_cache_path, request.directory,
                sizeof(count_cache_path) - 1);
        count_cache_total = index;
      }
      out->success = true;
    }
    if (dir)
      dir.close();
    sd_release();
    return mt_send_uplink(response);
  }

  case meshtastic_InterdeviceMessage_get_sd_info_tag: {
    // SD card statistics, answered entirely from state cached at mount
    // time so the response never waits on the card (this also serves as
    // the readiness probe target for older ESP32 firmware)
    meshtastic_InterdeviceMessage &response = tx_response;
    memset(&response, 0, sizeof(response));
    response.id = message.id; // correlate with the request
    response.which_data = meshtastic_InterdeviceMessage_sd_info_tag;
    sd_get_info(&response.data.sd_info);
    return mt_send_uplink(response);
  }

  case meshtastic_InterdeviceMessage_ping_tag: {
    // liveness probe, must work with nothing attached to this MCU
    meshtastic_InterdeviceMessage &response = tx_response;
    memset(&response, 0, sizeof(response));
    response.id = message.id; // correlate with the request
    response.which_data = meshtastic_InterdeviceMessage_pong_tag;
    response.data.pong = true;
    return mt_send_uplink(response);
  }

  case meshtastic_InterdeviceMessage_i2c_scan_tag: {
    // Scan the local bus and report all responding addresses
    meshtastic_InterdeviceMessage &response = tx_response;
    memset(&response, 0, sizeof(response));
    response.id = message.id; // correlate with the request
    response.which_data = meshtastic_InterdeviceMessage_i2c_scan_result_tag;
    for (uint8_t addr = 8; addr < 120; addr++) {
      if (response.data.i2c_scan_result.size >=
          sizeof(response.data.i2c_scan_result.bytes))
        break;
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0)
        response.data.i2c_scan_result
            .bytes[response.data.i2c_scan_result.size++] = addr;
    }
    return mt_send_uplink(response);
  }

  default:
    // the other messages really only flow downstream
    Serial.println("Got a message of unexpected type");
    return false;
  }
}

void mt_check_packet() {
  if (pb_rx_size < MT_HEADER_SIZE) {
    // We don't even have a header yet
    return;
  }

  if (pb_rx_buf[0] != MT_MAGIC_0 || pb_rx_buf[1] != MT_MAGIC_1) {
    Serial.println("Got bad magic");
    memset(pb_rx_buf, 0, PB_BUFSIZE);
    pb_rx_size = 0;
    return;
  }

  uint16_t payload_len = pb_rx_buf[2] << 8 | pb_rx_buf[3];
  if ((size_t)payload_len + MT_HEADER_SIZE > PB_BUFSIZE) {
    // oversized frame can never complete, resync on the next magic
    Serial.println("Got packet claiming to be ridiculous length");
    memset(pb_rx_buf, 0, PB_BUFSIZE);
    pb_rx_size = 0;
    return;
  }

  if ((size_t)(payload_len + MT_HEADER_SIZE) > pb_rx_size) {
    // Packet not complete yet
    return;
  }

  // We have a complete packet, handle it
  mt_handle_packet(payload_len);
}

size_t mt_serial_check(char *buf, size_t space_left) {
  size_t bytes_read = 0;
  while (bytes_read < space_left && Serial1.available()) {
    buf[bytes_read++] = Serial1.read();
  }
  return bytes_read;
}

bool mt_send_uplink(const meshtastic_InterdeviceMessage &message) {
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

void mt_set_nmea_callback(void (*callback)(char *nmea)) {
  nmea_callback = callback;
}