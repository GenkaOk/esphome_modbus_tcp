#include "modbustcp.h"
#include <new>
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"

namespace esphome::modbustcp {

static const char *const TAG = "modbustcp";

void ModbusTCP::setup() {
    client_ = new (std::nothrow) AsyncClient();
    if (client_ == nullptr) {
      // Without a client this component cannot do anything. Mark it failed instead of crashing:
      // ESPHome then skips it and the rest of the firmware keeps running.
      ESP_LOGE(TAG, "Failed to allocate AsyncClient, component disabled");
      this->mark_failed();
      return;
    }

    client_->onConnect([](void* arg, AsyncClient* c) {
      auto* self = (ModbusTCP*)arg;
      ESP_LOGI("tcp", "Connected to %s:%d", self->host_.c_str(), self->port_);
      self->connected_ = true;
    }, this);

    client_->onDisconnect([](void* arg, AsyncClient* c) {
      auto* self = (ModbusTCP*)arg;
      ESP_LOGW("tcp", "Disconnected, will retry...");
      self->connected_ = false;
      // Start the 5 s backoff at the moment of the disconnect. Leaving last_attempt_ at 0 made
      // loop() see "more than 5 s since the last attempt" on the very next iteration and reconnect
      // immediately, so a server that drops the new connection right away (e.g. a gateway still
      // holding the stale one) turned into a tight connect/disconnect loop that also churned
      // through LWIP sockets and could take the WiFi connection down with it.
      self->last_attempt_ = millis();
      // This runs in the async_tcp task: only raise a flag. loop() drops whatever is left from the
      // dead connection — a truncated frame there would glue itself to the first response of the
      // next connection and desynchronise the parser.
      self->reset_requested_.store(true);
    }, this);

    client_->onError([](void* arg, AsyncClient* c, int8_t err) {
      auto* self = (ModbusTCP*)arg;
      ESP_LOGE("tcp", "Error: %s", c->errorToString(err));
      self->connected_ = false;
    }, this);

    client_->onData([](void* arg, AsyncClient* c, void *data, size_t len) {
      auto* self = (ModbusTCP*)arg;
      self->handle_message(reinterpret_cast<uint8_t*>(data), len);
    }, this);
  }
  

      void ModbusTCP::connect() {
    if (client_ == nullptr || client_->connecting()) {
      return;
    }
    if (!client_->connect(host_.c_str(), port_)) {
      ESP_LOGW("tcp", "Connection failed, will retry...");
    }
  }

  void ModbusTCP::send_message(const uint8_t *send_byte, size_t len) {
    if (client_ == nullptr) {
      return;
    }
    if (connected_ && client_->canSend()) {
      // Send exactly the frame that was built. Deriving the length from the MBAP field instead
      // silently truncated write-multiple frames.
      client_->write(reinterpret_cast<const char*>(send_byte), len);

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
      // Building the hex dump allocates, so keep it out of the code path entirely when the level
      // is not compiled in.
      std::string res1;
      char buf1[5];
      for (size_t i = 12; i < len; i++) {
        sprintf(buf1, "%02X", send_byte[i]);
        res1 += buf1;
        res1 += ":";
      }
      ESP_LOGD("modbus_tcp", ">>> %02X%02X %02X%02X %02X%02X %02X %02X %02X%02X %02X%02X %s",
                   send_byte[0], send_byte[1], send_byte[2], send_byte[3], send_byte[4], send_byte[5],
                   send_byte[6], send_byte[7], send_byte[8], send_byte[9], send_byte[10], send_byte[11], res1.c_str());
#endif

      } else {
      ESP_LOGW("modbus_tcp", "Cannot send, not connected");
    }
  }

  // Called from the async_tcp task. Staging only — the bytes are parsed from loop(), together with
  // everything else that touches shared state. A single onData call is not guaranteed to line up
  // with frame boundaries (TCP can fragment a frame across calls, or coalesce several frames), so
  // loop() reassembles them from rx_buffer_.
  void ModbusTCP::handle_message(const uint8_t *data, size_t len) {
    if (len == 0 || data == nullptr) {
      return;
    }
    this->rx_stage_push_(data, len);
  }

  void ModbusTCP::rx_stage_push_(const uint8_t *data, size_t len) {
    uint32_t head = this->rx_stage_head_.load(std::memory_order_relaxed);
    uint32_t tail = this->rx_stage_tail_.load(std::memory_order_acquire);
    size_t free_bytes = RX_STAGE_SIZE - static_cast<size_t>(head - tail);
    if (len > free_bytes) {
      // Dropping a partial chunk would leave the parser stuck on an impossible length, so drop the
      // whole chunk and let loop() discard whatever it had buffered.
      this->rx_stage_dropped_.fetch_add(static_cast<uint32_t>(len), std::memory_order_relaxed);
      return;
    }
    for (size_t i = 0; i < len; i++) {
      this->rx_stage_[(head + i) & (RX_STAGE_SIZE - 1)] = data[i];
    }
    this->rx_stage_head_.store(head + static_cast<uint32_t>(len), std::memory_order_release);
  }

  size_t ModbusTCP::rx_stage_pop_(uint8_t *out, size_t max) {
    uint32_t head = this->rx_stage_head_.load(std::memory_order_acquire);
    uint32_t tail = this->rx_stage_tail_.load(std::memory_order_relaxed);
    size_t available = static_cast<size_t>(head - tail);
    size_t n = available < max ? available : max;
    for (size_t i = 0; i < n; i++) {
      out[i] = this->rx_stage_[(tail + i) & (RX_STAGE_SIZE - 1)];
    }
    this->rx_stage_tail_.store(tail + static_cast<uint32_t>(n), std::memory_order_release);
    return n;
  }

  // Extracts and dispatches every complete Modbus TCP ADU currently sitting in rx_buffer_,
  // leaving any trailing partial frame in the buffer for the next call.
  void ModbusTCP::process_buffer_() {
    static const size_t MBAP_HEADER_LEN = 7;  // transaction id(2) + protocol id(2) + length(2) + unit id(1)
    static const size_t MAX_FRAME_LEN = 260;  // Modbus TCP ADUs are capped well below this

    while (true) {
      if (rx_buffer_.size() < MBAP_HEADER_LEN) {
        return;  // not enough for a header yet
      }

      uint16_t length_field = (uint16_t(rx_buffer_[4]) << 8) | rx_buffer_[5];
      size_t frame_len = 6 + length_field;  // total ADU length, per the MBAP length field

      if (length_field == 0 || frame_len > MAX_FRAME_LEN) {
        ESP_LOGW(TAG, "Implausible Modbus TCP frame length %zu, discarding %zu buffered bytes", frame_len,
                 rx_buffer_.size());
        rx_buffer_.clear();
        return;
      }

      if (rx_buffer_.size() < frame_len) {
        return;  // frame not fully received yet
      }

      handle_frame_(rx_buffer_.data(), frame_len);
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_len);
    }
  }

  void ModbusTCP::handle_frame_(const uint8_t *byte, size_t frame_len) {
    // Smallest frame that can be decoded: MBAP header (7) + function code (1).
    if (frame_len < 8) {
      ESP_LOGW(TAG, "Modbus frame too short (%zu bytes), discarding", frame_len);
      return;
    }

    uint16_t transaction_id = (uint16_t(byte[0]) << 8) | byte[1];

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    std::string res;
    char buf[5];
    for (size_t i = 8; i < frame_len; i++) {
      sprintf(buf, "%02X", byte[i]);
      res += buf;
      res += ":";
    }

    ESP_LOGD("modbus_tcp", " <<< %02X%02X %02X%02X %02X%02X %02X %02X %s ", byte[0], byte[1], byte[2], byte[3],
             byte[4], byte[5], byte[6], byte[7], res.c_str());
#endif

    if (transaction_id != expected_transaction_id_) {
      // Late/stray response (e.g. arrived after the controller already gave up and moved on to
      // the next command) or a response meant for another client multiplexed through a proxy.
      // Decoding it as if it belonged to the current pending command would silently hand
      // garbage values to whichever sensor is currently waiting. Drop it instead.
      ESP_LOGW(TAG, "Ignoring Modbus response with unexpected transaction id 0x%04X (expected 0x%04X)",
               transaction_id, expected_transaction_id_);
      return;
    }

    uint8_t unit_id = byte[6];
    uint8_t function_code = byte[7] & 0x7F;

    if ((byte[7] & 0x80) == 0x80) {
      uint8_t exception_code = frame_len > 8 ? byte[8] : 0;
      ESP_LOGE(TAG, "Error: ");
      switch (exception_code) {
        case 0x01: ESP_LOGE(TAG, "Failure Code 0x01 ILLEGAL FUNCTION"); break;
        case 0x02: ESP_LOGE(TAG, "Failure Code 0x02 ILLEGAL DATA ADDRESS"); break;
        case 0x03: ESP_LOGE(TAG, "Failure Code 0x03 ILLEGAL DATA VALUE"); break;
        case 0x04: ESP_LOGE(TAG, "Failure Code 0x04 SERVER FAILURE"); break;
        case 0x05: ESP_LOGE(TAG, "Failure Code 0x05 ACKNOWLEDGE"); break;
        case 0x06: ESP_LOGE(TAG, "Failure Code 0x06 SERVER BUSY"); break;
      }
      // Hand the exception to the device instead of swallowing it: a legal "illegal data address"
      // used to be indistinguishable from a dead device, so the command was retried until the
      // controller declared it offline.
      for (auto *device : this->devices_) {
        if (device->address_ == unit_id) {
          device->on_modbus_error(function_code, exception_code);
        }
      }
      waiting_for_response = 0;
      return;
    }

    // Only read responses (FC 01-04) carry a leading byte count. Write ACKs (FC 05/06/0F/10) echo
    // the address (and the written value) straight after the function code — treating their byte 8
    // as a byte count dropped every ACK for addresses >= 1024 and handed an empty/garbage payload
    // to the write handler for smaller ones.
    bool has_byte_count = function_code >= 0x01 && function_code <= 0x04;
    size_t data_offset = has_byte_count ? 9 : 8;

    if (frame_len < data_offset) {
      ESP_LOGW(TAG, "Modbus response too short (%zu bytes) for function code 0x%02X, discarding", frame_len,
               function_code);
      return;
    }

    size_t data_len;
    if (has_byte_count) {
      data_len = byte[8];
      if (data_offset + data_len > frame_len) {
        ESP_LOGW(TAG, "Modbus response byte count (%zu) exceeds frame length (%zu), discarding", data_len, frame_len);
        return;
      }
    } else {
      data_len = frame_len - data_offset;
    }

    std::vector<uint8_t> data(byte + data_offset, byte + data_offset + data_len);

    // Deliver to the controller that owns this unit id. Without the filter every controller on the
    // bus consumed every response and popped a command it had not sent.
    bool matched = false;
    for (auto *device : this->devices_) {
      if (device->address_ == unit_id) {
        device->on_modbus_data(data);
        matched = true;
      }
    }
    if (!matched) {
      // Some gateways answer with a unit id that differs from the request (e.g. 0 or 0xFF). Keep
      // the old broadcast behaviour for them so that sensors still update.
      ESP_LOGD(TAG, "No device configured for unit id %u, delivering response to all devices", unit_id);
      for (auto *device : this->devices_) {
        device->on_modbus_data(data);
      }
    }

    // Cleared only after the frame has been handed over (and the command popped), so the main loop
    // cannot resend the command that just got its answer. Doing it here instead of relying on the
    // send_wait_time_ timeout lets the next command go out without paying a full 250 ms per request.
    waiting_for_response = 0;
  }

  void ModbusTCP::on_shutdown() {
    if (client_) {
      client_->close();
    }
  }

    

void ModbusTCP::loop() {
  if (this->is_failed()) {
    return;
  }

  // Applied here because onDisconnect runs in the async_tcp task and must not touch the parser
  // state: a truncated frame from the dead connection would glue itself to the first response of
  // the next one, and no response can arrive for the command we were waiting on.
  if (this->reset_requested_.exchange(false)) {
    this->rx_buffer_.clear();
    this->waiting_for_response = 0;
    this->rx_stage_tail_.store(this->rx_stage_head_.load(std::memory_order_acquire), std::memory_order_release);
  }

  uint32_t dropped = this->rx_stage_dropped_.exchange(0, std::memory_order_relaxed);
  if (dropped > 0) {
    ESP_LOGW(TAG, "Receive staging buffer full, dropped %u bytes and resynchronising", dropped);
    this->rx_buffer_.clear();
  }

  uint8_t chunk[128];
  bool received = false;
  size_t n;
  while ((n = this->rx_stage_pop_(chunk, sizeof(chunk))) > 0) {
    this->rx_buffer_.insert(this->rx_buffer_.end(), chunk, chunk + n);
    received = true;
  }
  if (received) {
    this->process_buffer_();
  }

  // Only (re)connect when there is an actual network. After a reboot this component's loop runs
  // before WiFi has an IP; attempts made there fail anyway and each one burns an LWIP socket.
  if (!connected_ && network::is_connected()) {
      uint32_t nowtcp = millis();
      if (nowtcp - last_attempt_ > 5000) {
        last_attempt_ = nowtcp;
        ESP_LOGW("tcp", "Reconnecting...");
        connect();
      }
    }
  const uint32_t now = App.get_loop_component_start_time();
  

 if (now - this->last_send_ > send_wait_time_) {
      if (waiting_for_response > 0) {
        //ESP_LOGD(TAG, "Stop waiting for response from %d", waiting_for_response);
      }
      waiting_for_response = 0;
    }
  


  }
void ModbusTCP::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus_TCP:");
  ESP_LOGCONFIG(TAG, "  Client: %s \n"
                     "  Port: %d \n"
                     "  Send Wait Time: %d ms\n",
                         host_.c_str(), port_, this->send_wait_time_);
}

float ModbusTCP::get_setup_priority() const { return setup_priority::AFTER_WIFI - 1.0f; }

void ModbusTCP::send(uint8_t address, uint8_t function_code, uint16_t start_address, uint16_t number_of_entities, size_t payload_len, const uint8_t *payload) {
  static const size_t MAX_VALUES = 128;
 const uint32_t now = App.get_loop_component_start_time();
  // Only check max number of registers for standard function codes
  // Some devices use non standard codes like 0x43
  if (number_of_entities > MAX_VALUES && function_code <= 0x10) {
    ESP_LOGE(TAG, "send too many values %d max=%zu", number_of_entities, MAX_VALUES);
    return;
  }
    
  size_t MAX_FRAME_SIZE = 256;
  uint8_t data[MAX_FRAME_SIZE];
  size_t pos = 0;
      data[pos++] = Transaction_Identifier >> 8;
      data[pos++] = Transaction_Identifier >> 0;
      data[pos++] = 0x00;
      data[pos++] = 0x00;
      data[pos++] = 0x00;
      data[pos++] = 0x00;  // MBAP length low byte; filled in below once the frame is complete
      data[pos++] = address;
      data[pos++] = function_code;
      data[pos++] = start_address >> 8;
      data[pos++] = start_address >> 0;

      if (function_code != 0x05 && function_code != 0x06) {
      data[pos++] = number_of_entities >> 8;
      data[pos++] = number_of_entities >> 0;
      }
   if (payload != nullptr) {
     if (function_code == 0x10 || function_code == 0x0f) {           // Write multiple
      data[pos++] = payload_len;                                     // Byte count is required for write
    } else {
      payload_len = 2;  // Write single register or coil
    }

    if (payload_len + pos > MAX_FRAME_SIZE) {  // Check if send more as 256 bytes
      ESP_LOGE(TAG, "Payload too large to send: %zu bytes", payload_len);
      return;
    }
    for (size_t i = 0; i < payload_len; i++) {
      data[pos++] = payload[i];
    }
  }
       
       
       expected_transaction_id_ = Transaction_Identifier;
       Transaction_Identifier++;
  // MBAP length field = unit id + PDU = everything after the first six bytes. Deriving it from the
  // frame we actually built is what makes write-multiple work: the hand-computed 0x04 + payload_len
  // used to be three bytes short, so the byte count and the payload were never transmitted.
  data[5] = pos - 6;
  send_message(data, pos);

waiting_for_response = address;
last_send_ = millis();
}


void ModbusTCP::send_raw(const std::vector<uint8_t> &payload) {
  if (payload.empty() || client_ == nullptr) {
    return;
  }

 // this->write_array(payload);
  client_->write(reinterpret_cast<const char*>(payload.data()), payload.size());
  //this->client.clear();
  waiting_for_response = payload[0];
  ESP_LOGV(TAG, "Modbus write raw: %s", format_hex_pretty(payload).c_str());
  last_send_ = millis();
}


}  // namespace esphome::modbustcp
