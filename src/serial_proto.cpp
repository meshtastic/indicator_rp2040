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

// Called by core1 (holding the mutex, before it unmounts) so the handle is
// closed against the volume it belongs to. Closing it after the remount
// would flush the old volume's sector cache onto the new card.
void sd_close_cached_files(void) {
  invalidate_read_cache();
  count_cache_path[0] = '\0';
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

#define PB_BUFSIZE (meshtastic_InterdeviceMessage_size + MT_HEADER_SIZE)

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

// Unsolicited ping at boot: tells the main firmware we are (back) up and
// which protocol version we speak, without it having to poll for us.
bool mt_send_hello(void) {
  meshtastic_InterdeviceMessage &response = tx_response;
  memset(&response, 0, sizeof(response));
  response.which_data = meshtastic_InterdeviceMessage_ping_tag;
  response.data.ping =
      meshtastic_InterdeviceVersion_INTERDEVICE_VERSION_CURRENT;
  return mt_send_uplink(response);
}

// The requester fails fast on a nack instead of burning its timeout.
// id 0 when the request could not even be decoded.
static bool mt_send_nack(uint32_t id) {
  meshtastic_InterdeviceMessage &response = tx_response;
  memset(&response, 0, sizeof(response));
  response.id = id;
  response.which_data = meshtastic_InterdeviceMessage_nack_tag;
  response.data.nack = true;
  return mt_send_uplink(response);
}

// The core reports any failed non-empty write as a generic error; a
// zero-length probe (bit-banged, always classifies) tells an absent
// device apart from a data NACK or bus fault
static bool i2c_device_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Size of an existing file, 0 when it is not one. Used to keep the used/free
// accounting straight across writes and deletes.
static uint64_t file_size_of(const char *path) {
  File f = SD.open(path, FILE_READ);
  if (!f)
    return 0;
  uint64_t size = f.isDirectory() ? 0 : (uint64_t)f.size();
  f.close();
  return size;
}

// An operation on a path can fail because the path is wrong or because the
// card is gone; only the card itself can tell us apart. Reading the root
// directory touches the card, so a mounted card that cannot produce it has
// been pulled. Called only on failure paths.
static bool card_still_there(void) {
  if (SD.exists("/"))
    return true;
  Serial.println("SD card stopped responding");
  sd_mark_dead();
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
    mt_send_nack(0);
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
    meshtastic_I2CTransaction &transaction = message.data.i2c_transaction;
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
        result->status = i2c_device_present((uint8_t)transaction.address)
                             ? meshtastic_I2CResult_Status_NACK_DATA
                             : meshtastic_I2CResult_Status_NACK_ADDRESS;
      }
    }

    if (result->status == meshtastic_I2CResult_Status_OK &&
        transaction.read_len > 0) {
      size_t want = transaction.read_len;
      if (want > sizeof(result->read_data.bytes))
        want = sizeof(result->read_data.bytes);
      size_t got = Wire.requestFrom((uint8_t)transaction.address, want);
      if (got == 0)
        // a failed read can also be a bus timeout with the device present
        result->status = i2c_device_present((uint8_t)transaction.address)
                             ? meshtastic_I2CResult_Status_ERROR
                             : meshtastic_I2CResult_Status_NACK_ADDRESS;
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

    int claim = sd_claim();
    if (claim != SD_CLAIM_OK) {
      out->status = claim == SD_CLAIM_BUSY ? meshtastic_FileStatus_FILE_BUSY
                                           : meshtastic_FileStatus_FILE_NO_CARD;
      return mt_send_uplink(response);
    }
    check_sd_generation();

    switch (request.operation) {
    case meshtastic_FileOperation_GET: {
      if (!read_cache || strcmp(read_cache_path, request.filepath) != 0) {
        invalidate_read_cache();
        read_cache = SD.open(request.filepath, FILE_READ);
        if (!read_cache) {
          // a pulled card fails exactly like a missing file, and map tiles
          // are probed constantly: this is the path that must notice
          out->status = card_still_there()
                            ? meshtastic_FileStatus_FILE_NOT_FOUND
                            : meshtastic_FileStatus_FILE_NO_CARD;
          break;
        }
        if (read_cache.isDirectory()) {
          // reading one returns raw directory entries, not file content
          invalidate_read_cache();
          out->status = meshtastic_FileStatus_FILE_NOT_A_FILE;
          break;
        }
        strncpy(read_cache_path, request.filepath, sizeof(read_cache_path) - 1);
      }
      out->file_size = read_cache.size();
      size_t want = request.length;
      if (want == 0 || want > sizeof(out->filedata.bytes))
        want = sizeof(out->filedata.bytes);
      // the FS layer seeks with 32 bit positions, and seeking past the end
      // is a request error, not a card failure
      if (request.offset > 0xFFFFFFFFull ||
          request.offset > (uint64_t)read_cache.size()) {
        out->status = meshtastic_FileStatus_FILE_OFFSET_CONFLICT;
        break;
      }
      if (!read_cache.seek((uint32_t)request.offset)) {
        out->status = meshtastic_FileStatus_FILE_IO_ERROR;
        sd_mark_dead(); // in range, so the card itself failed
        break;
      }
      int got = read_cache.read(out->filedata.bytes, want);
      if (got < 0) {
        out->status = meshtastic_FileStatus_FILE_IO_ERROR;
        sd_mark_dead();
        break;
      }
      out->filedata.size = got;
      out->status = meshtastic_FileStatus_FILE_OK;
      break;
    }

    case meshtastic_FileOperation_POST:
    case meshtastic_FileOperation_PUT: {
      invalidate_read_cache();
      if (request.operation == meshtastic_FileOperation_POST) {
        if (request.offset != 0) {
          // a POST starts a new file; truncating for a chunk that is not
          // the first one would destroy what is already there
          out->status = meshtastic_FileStatus_FILE_OFFSET_CONFLICT;
          break;
        }
        if (SD.exists(request.filepath)) {
          sd_account_bytes(-(int64_t)file_size_of(request.filepath));
          SD.remove(request.filepath);
        }
      }
      File f = SD.open(request.filepath, FILE_WRITE); // append mode, creates
      if (!f) {
        // a full card, a bad path and a pulled card all fail here
        out->status = card_still_there() ? meshtastic_FileStatus_FILE_IO_ERROR
                                         : meshtastic_FileStatus_FILE_NO_CARD;
        strncpy(out->message, "open failed", sizeof(out->message) - 1);
        break;
      }
      if (f.size() != request.offset) {
        out->status = meshtastic_FileStatus_FILE_OFFSET_CONFLICT;
        out->file_size = f.size(); // lets the writer resync its offset
        f.close();
        break;
      }
      size_t wrote = f.write(request.filedata.bytes, request.filedata.size);
      out->file_size = f.size();
      f.close();
      sd_account_bytes((int64_t)wrote);
      if (wrote == request.filedata.size) {
        out->status = meshtastic_FileStatus_FILE_OK;
      } else {
        // short write: a full card (do not unmount, it would thrash between
        // remounts) or a card that went away
        out->status = card_still_there() ? meshtastic_FileStatus_FILE_IO_ERROR
                                         : meshtastic_FileStatus_FILE_NO_CARD;
        strncpy(out->message, "write failed", sizeof(out->message) - 1);
      }
      count_cache_path[0] = '\0';
      break;
    }

    case meshtastic_FileOperation_DELETE: {
      invalidate_read_cache();
      uint64_t freed = file_size_of(request.filepath);
      // idempotent: a file that is already gone is the requested outcome,
      // so a retried delete after a lost response reports OK
      if (SD.remove(request.filepath) || !SD.exists(request.filepath)) {
        out->status = meshtastic_FileStatus_FILE_OK;
        sd_account_bytes(-(int64_t)freed);
      } else {
        out->status = card_still_there() ? meshtastic_FileStatus_FILE_IO_ERROR
                                         : meshtastic_FileStatus_FILE_NO_CARD;
        strncpy(out->message, "remove failed", sizeof(out->message) - 1);
      }
      count_cache_path[0] = '\0';
      break;
    }

    default:
      out->status = meshtastic_FileStatus_FILE_IO_ERROR;
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

    int claim = sd_claim();
    if (claim != SD_CLAIM_OK) {
      out->status = claim == SD_CLAIM_BUSY ? meshtastic_FileStatus_FILE_BUSY
                                           : meshtastic_FileStatus_FILE_NO_CARD;
      return mt_send_uplink(response);
    }
    check_sd_generation();

    File dir = SD.open(request.directory);
    bool is_dir = dir && dir.isDirectory();
    if (dir)
      dir.close();
    if (!is_dir) {
      out->status =
          SD.exists(request.directory)
              ? meshtastic_FileStatus_FILE_NOT_A_FILE
              : (card_still_there() ? meshtastic_FileStatus_FILE_NOT_FOUND
                                    : meshtastic_FileStatus_FILE_NO_CARD);
    } else {
      const size_t max_names =
          sizeof(out->filenames) / sizeof(out->filenames[0]);
      // Iterate with the Dir API: openNextFile() re-opens every entry by
      // full path, which turns a directory walk into an O(n^2) scan and can
      // block core0 for minutes on a directory full of map tiles.
      Dir d = SDFS.openDir(request.directory);
      // Counting the entries means walking the whole directory, so do it
      // once on the first page and reuse the count for the follow-up pages
      bool have_total = request.offset > 0 &&
                        strcmp(count_cache_path, request.directory) == 0;
      uint32_t index = 0;
      while (d.next()) {
        if (index >= request.offset && out->filenames_count < max_names) {
          snprintf(out->filenames[out->filenames_count],
                   sizeof(out->filenames[0]), "%s%s", d.fileName().c_str(),
                   d.isDirectory() ? "/" : "");
          out->filenames_count++;
        }
        index++;
        if (have_total && out->filenames_count >= max_names)
          break;            // page full, the total is already known
        rp2040.wdt_reset(); // a big directory still takes a while
      }
      if (have_total) {
        out->total_count = count_cache_total;
      } else {
        out->total_count = index;
        strncpy(count_cache_path, request.directory,
                sizeof(count_cache_path) - 1);
        count_cache_total = index;
      }
      out->status = meshtastic_FileStatus_FILE_OK;
    }
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
    // Liveness probe and version handshake, must work with nothing
    // attached to this MCU. Always answer with the version we speak: the
    // main firmware compares it and shuts the bridge down on a mismatch.
    static uint32_t logged_peer_version = UINT32_MAX;
    if (message.data.ping !=
            meshtastic_InterdeviceVersion_INTERDEVICE_VERSION_CURRENT &&
        message.data.ping != logged_peer_version) {
      // Only when the version we see changes: the peer probes repeatedly
      // while the link is down, and a USB CDC write blocks for up to a
      // second when a host is attached but not reading. Reflashing the peer
      // to another (still wrong) version does get logged again.
      logged_peer_version = message.data.ping;
      Serial.printf(
          "Peer speaks interdevice protocol v%u, we speak v%u\r\n",
          (unsigned)message.data.ping,
          (unsigned)meshtastic_InterdeviceVersion_INTERDEVICE_VERSION_CURRENT);
    }
    meshtastic_InterdeviceMessage &response = tx_response;
    memset(&response, 0, sizeof(response));
    response.id = message.id; // correlate with the request
    response.which_data = meshtastic_InterdeviceMessage_pong_tag;
    response.data.pong =
        meshtastic_InterdeviceVersion_INTERDEVICE_VERSION_CURRENT;
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
      // a hung bus times out per address, which adds up past the watchdog
      rp2040.wdt_reset();
    }
    // a stuck-low SDA acks every probe; report a fault as an empty scan
    // rather than 112 phantom devices
    if (response.data.i2c_scan_result.size >= 100) {
      Serial.println("I2C scan: bus fault, every address responded");
      response.data.i2c_scan_result.size = 0;
    }
    return mt_send_uplink(response);
  }

  case meshtastic_InterdeviceMessage_pong_tag:
  case meshtastic_InterdeviceMessage_nack_tag:
    // never react to these, a nacked nack would ping-pong forever
    return true;

  default:
    // the other messages really only flow downstream
    Serial.println("Got a message of unexpected type");
    mt_send_nack(message.id);
    return false;
  }
}

// Distance to the next byte that could start a frame. Skips the corrupt
// prefix while keeping anything that may be a frame queued behind it (a
// trailing lone MT_MAGIC_0 counts, its successor has not arrived yet).
static size_t scan_magic(const pb_byte_t *buf, size_t len) {
  for (size_t i = 1; i < len; i++) {
    if (buf[i] == MT_MAGIC_0 && (i + 1 == len || buf[i + 1] == MT_MAGIC_1))
      return i;
  }
  return len;
}

void mt_check_packet() {
  // process everything buffered; one pass can deliver several frames
  while (pb_rx_size >= MT_HEADER_SIZE) {
    size_t payload_len = (size_t)(pb_rx_buf[2] << 8 | pb_rx_buf[3]);
    if (pb_rx_buf[0] != MT_MAGIC_0 || pb_rx_buf[1] != MT_MAGIC_1 ||
        payload_len + MT_HEADER_SIZE > PB_BUFSIZE) {
      // Corrupt or false header: resync on the next magic instead of
      // flushing, one bad byte must not cost the frames behind it
      size_t skip = scan_magic(pb_rx_buf, pb_rx_size);
      Serial.printf("Bad frame header, dropping %u bytes\r\n", (unsigned)skip);
      memmove(pb_rx_buf, pb_rx_buf + skip, pb_rx_size - skip);
      pb_rx_size -= skip;
      continue;
    }

    if (payload_len + MT_HEADER_SIZE > pb_rx_size)
      return; // frame not complete yet

    mt_handle_packet(payload_len);
  }
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

  pb_ostream_t stream = pb_ostream_from_buffer(pb_tx_buf + MT_HEADER_SIZE,
                                               PB_BUFSIZE - MT_HEADER_SIZE);
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

  // byte loss is otherwise invisible: it surfaces as a resync much later
  if (Serial1.overflow())
    Serial.println("Serial1 RX overflow, frames lost");

  // See if there are any more bytes to add to our buffer.
  size_t space_left = PB_BUFSIZE - pb_rx_size;

  bytes_read = mt_serial_check((char *)pb_rx_buf + pb_rx_size, space_left);

  pb_rx_size += bytes_read;
  mt_check_packet();
}

void mt_set_nmea_callback(void (*callback)(char *nmea)) {
  nmea_callback = callback;
}