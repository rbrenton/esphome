#include "alpha_hwr.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <lwip/sockets.h>  //gives ntohl

#ifdef USE_ESP32

namespace esphome {
namespace alpha_hwr {

static const char *const TAG = "alpha_hwr";


std::vector<KnownCommand> known_commands_ = {
  {"FLOW_HEAD", {39, 7, 231, 248, 10, 3, 93, 1, 33, 82, 31}, "Alpha3 Flow/Head request"},
  {"POWER", {39, 7, 231, 248, 10, 3, 87, 0, 69, 138, 205}, "Alpha3 Power request"},
  {"TEMP_TEST", {39, 7, 231, 248, 10, 3, 84, 0, 69, 138, 205}, "Temperature test command"},
  // Add more as discovered
};

void ProtocolCapture::log_packet(const std::string& direction, const uint8_t* data, size_t len, const std::string& notes) {
  if (!enabled || len == 0) return;

  PacketEntry entry;
  entry.timestamp = millis();
  entry.direction = direction;
  entry.data.assign(data, data + len);
  entry.notes = notes;
  packets.push_back(entry);

  // Limit log size to prevent memory issues
  if (packets.size() > 100) {
    packets.erase(packets.begin());
  }
}

void Alpha_HWR::dump_config() {
  ESP_LOGCONFIG(TAG, "ALPHA_HWR");
  LOG_SENSOR(" ", "Flow", this->flow_sensor_);
  LOG_SENSOR(" ", "Head", this->head_sensor_);
  LOG_SENSOR(" ", "Power", this->power_sensor_);
  LOG_SENSOR(" ", "Current", this->current_sensor_);
  LOG_SENSOR(" ", "Speed", this->speed_sensor_);
  LOG_SENSOR(" ", "Voltage", this->voltage_sensor_);
}

void Alpha_HWR::setup() {}

void Alpha_HWR::extract_publish_sensor_value_(const uint8_t *response, int16_t length, int16_t response_offset,
                                           int16_t value_offset, sensor::Sensor *sensor, float factor) {
  if (sensor == nullptr)
    return;
  // we need to handle cases where a value is split over two packets
  const int16_t value_length = 4;  // 32bit float
  // offset inside current response packet
  auto rel_offset = value_offset - response_offset;
  if (rel_offset <= -value_length)
    return;  // aready passed the value completly
  if (rel_offset >= length)
    return;  // value not in this packet

  auto start_offset = std::max(0, rel_offset);
  auto end_offset = std::min((int16_t) (rel_offset + value_length), length);
  auto copy_length = end_offset - start_offset;
  auto buffer_offset = std::max(-rel_offset, 0);
  std::memcpy(this->buffer_ + buffer_offset, response + start_offset, copy_length);

  if (rel_offset + value_length <= length) {
    // we have the whole value
    void *buffer = this->buffer_;                          // to prevent warnings when casting the pointer
    *((int32_t *) buffer) = ntohl(*((int32_t *) buffer));  // values are big endian
    float fvalue = *((float *) buffer);
    sensor->publish_state(fvalue * factor);
  }
}

bool Alpha_HWR::is_current_response_type_(const uint8_t *response_type) {
  return !std::memcmp(this->response_type_, response_type, GENI_RESPONSE_TYPE_LENGTH);
}

void Alpha_HWR::handle_geni_response_(const uint8_t *response, uint16_t length) {
  // Log EVERYTHING that comes in
  this->log_raw_protocol_data("RX", response, length, "GENI_RESPONSE");
  
  if (this->response_offset_ >= this->response_length_) {
    ESP_LOGD(TAG, "[%s] GENI response begin", this->parent_->address_str().c_str());
    if (length < GENI_RESPONSE_HEADER_LENGTH) {
      ESP_LOGW(TAG, "[%s] response too short", this->parent_->address_str().c_str());
      return;
    }
    if (response[0] != 36 || response[2] != 248 || response[3] != 231 || response[4] != 10) {
      ESP_LOGW(TAG, "[%s] response bytes %d %d %d %d %d don't match GENI HEADER", this->parent_->address_str().c_str(),
               response[0], response[1], response[2], response[3], response[4]);
      return;
    }
    this->response_length_ = response[1] - GENI_RESPONSE_HEADER_LENGTH + 2;
    this->response_offset_ = -GENI_RESPONSE_HEADER_LENGTH;
    std::memcpy(this->response_type_, response + 5, GENI_RESPONSE_TYPE_LENGTH);
    
    // Enhanced response type logging
    this->log_raw_protocol_data("RX", this->response_type_, GENI_RESPONSE_TYPE_LENGTH, "RESPONSE_TYPE");
  }

  auto extract_publish_sensor_value = [response, length, this](int16_t value_offset, sensor::Sensor *sensor, float factor) {
    this->extract_publish_sensor_value_(response, length, this->response_offset_, value_offset, sensor, factor);
  };

  if (this->is_current_response_type_(GENI_RESPONSE_TYPE_FLOW_HEAD)) {
    ESP_LOGD(TAG, "[%s] FLOW HEAD Response", this->parent_->address_str().c_str());
    extract_publish_sensor_value(GENI_RESPONSE_FLOW_OFFSET, this->flow_sensor_, 3600.0F);
    extract_publish_sensor_value(GENI_RESPONSE_HEAD_OFFSET, this->head_sensor_, .0001F);
  } else if (this->is_current_response_type_(GENI_RESPONSE_TYPE_POWER)) {
    ESP_LOGD(TAG, "[%s] POWER Response", this->parent_->address_str().c_str());
    extract_publish_sensor_value(GENI_RESPONSE_POWER_OFFSET, this->power_sensor_, 1.0F);
    extract_publish_sensor_value(GENI_RESPONSE_CURRENT_OFFSET, this->current_sensor_, 1.0F);
    extract_publish_sensor_value(GENI_RESPONSE_MOTOR_SPEED_OFFSET, this->speed_sensor_, 1.0F);
    extract_publish_sensor_value(GENI_RESPONSE_VOLTAGE_AC_OFFSET, this->voltage_sensor_, 1.0F);
  } else {
    if (this->response_type_[0] == 48) {
      ESP_LOGI(TAG, "[%s] HWR STATUS Response (Type 48)", this->parent_->address_str().c_str());
      this->analyze_response_type_48(this->response_type_, GENI_RESPONSE_TYPE_LENGTH);
    } else {
      ESP_LOGW(TAG, "Unknown response type: %d", this->response_type_[0]);
    }
  }

  this->response_offset_ += length;
}

void Alpha_HWR::export_protocol_capture() {
  ESP_LOGI(TAG, "=== PROTOCOL CAPTURE EXPORT ===");
  ESP_LOGI(TAG, "Total packets: %d", this->protocol_capture_.packets.size());
  ESP_LOGI(TAG, "Format: TIMESTAMP DIRECTION LENGTH DATA [NOTES]");
  ESP_LOGI(TAG, "");

  for (const auto& packet : this->protocol_capture_.packets) {
    std::string hex_data;
    for (size_t i = 0; i < packet.data.size(); i++) {
      char hex_buf[4];
      snprintf(hex_buf, sizeof(hex_buf), "%02X", packet.data[i]);
      hex_data += hex_buf;
      if (i < packet.data.size() - 1) hex_data += " ";
    }

    ESP_LOGI(TAG, "%d %s %d %s %s",
             packet.timestamp,
             packet.direction.c_str(),
             packet.data.size(),
             hex_data.c_str(),
             packet.notes.c_str());
  }

  ESP_LOGI(TAG, "=== END PROTOCOL CAPTURE ===");
}

void Alpha_HWR::test_discovery_commands() {
  if (!this->protocol_discovery_mode_) return;
  
  uint32_t now = millis();
  if (now - this->last_discovery_command_ < 10000) return;  // 10 second intervals
  
  ESP_LOGI(TAG, "=== DISCOVERY PHASE %d ===", this->discovery_phase_);
  
  switch (this->discovery_phase_) {
    case 0: {
      ESP_LOGI(TAG, "Testing: Status/Info command");
      uint8_t cmd[] = {39, 7, 231, 248, 10, 3, 77, 0, 69, 138, 205};
      this->log_protocol_data("CMD_STATUS", cmd, sizeof(cmd));
      this->send_request_(cmd, sizeof(cmd));
      break;
    }
    case 1: {
      ESP_LOGI(TAG, "Testing: Temperature command (hypothetical)");
      uint8_t cmd[] = {39, 7, 231, 248, 10, 3, 84, 0, 69, 138, 205};
      this->log_protocol_data("CMD_TEMP", cmd, sizeof(cmd));
      this->send_request_(cmd, sizeof(cmd));
      break;
    }
    default:
      ESP_LOGI(TAG, "Discovery complete. Continuing with normal operation...");
      this->protocol_discovery_mode_ = false;
      return;
  }
  
  this->discovery_phase_++;
  this->last_discovery_command_ = now;
}

void Alpha_HWR::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        this->response_offset_ = 0;
        this->response_length_ = 0;
        ESP_LOGI(TAG, "[%s] connection open", this->parent_->address_str().c_str());
      }
      break;
    }
    case ESP_GATTC_CONNECT_EVT: {
      if (std::memcmp(param->connect.remote_bda, this->parent_->get_remote_bda(), 6) != 0)
        return;
      auto ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
      if (ret) {
        ESP_LOGW(TAG, "esp_ble_set_encryption failed, status=%x", ret);
      }
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      this->node_state = espbt::ClientState::IDLE;
      if (this->flow_sensor_ != nullptr)
        this->flow_sensor_->publish_state(NAN);
      if (this->head_sensor_ != nullptr)
        this->head_sensor_->publish_state(NAN);
      if (this->power_sensor_ != nullptr)
        this->power_sensor_->publish_state(NAN);
      if (this->current_sensor_ != nullptr)
        this->current_sensor_->publish_state(NAN);
      if (this->speed_sensor_ != nullptr)
        this->speed_sensor_->publish_state(NAN);
      if (this->speed_sensor_ != nullptr)
        this->voltage_sensor_->publish_state(NAN);
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      auto *chr = this->parent_->get_characteristic(ALPHA_HWR_GENI_SERVICE_UUID, ALPHA_HWR_GENI_CHARACTERISTIC_UUID);
      if (chr == nullptr) {
        ESP_LOGE(TAG, "[%s] No GENI service found at device, not an Alpha HRW..?", this->parent_->address_str().c_str());
        break;
      }
      auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                      chr->handle);
      if (status) {
        ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed, status=%d", status);
      }
      this->geni_handle_ = chr->handle;
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      this->node_state = espbt::ClientState::ESTABLISHED;
      this->update();
      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.handle == this->geni_handle_) {
        this->handle_geni_response_(param->notify.value, param->notify.value_len);
      }
      break;
    }
    default:
      break;
  }
}

void Alpha_HWR::send_request_(uint8_t *request, size_t len) {
  auto status =
      esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->geni_handle_, len,
                               request, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status)
    ESP_LOGW(TAG, "[%s] esp_ble_gattc_write_char failed, status=%d", this->parent_->address_str().c_str(), status);
}

void Alpha_HWR::update() {
  if (this->node_state != espbt::ClientState::ESTABLISHED) {
    ESP_LOGW(TAG, "[%s] Cannot poll, not connected", this->parent_->address_str().c_str());
    return;
  }

  // Run discovery commands first (if enabled)
  if (this->protocol_discovery_mode_ && this->discovery_phase_ < 3) {
    this->test_discovery_commands();
    return;  // Skip normal polling during discovery
  }

  if (this->flow_sensor_ != nullptr || this->head_sensor_ != nullptr) {
    uint8_t geni_request_flow_head[] = {39, 7, 231, 248, 10, 3, 93, 1, 33, 82, 31};
    this->send_request_(geni_request_flow_head, sizeof(geni_request_flow_head));
    delay(25);  // need to wait between requests
  }
  if (this->power_sensor_ != nullptr || this->current_sensor_ != nullptr || this->speed_sensor_ != nullptr ||
      this->voltage_sensor_ != nullptr) {
    uint8_t geni_request_power[] = {39, 7, 231, 248, 10, 3, 87, 0, 69, 138, 205};
    this->send_request_(geni_request_power, sizeof(geni_request_power));
    delay(25);  // need to wait between requests
  }
}
// Specific implementation for handling Response Type 48
// Add this to your alpha_hwr.cpp file

void Alpha_HWR::parse_hwr_response_48(const uint8_t* data, size_t len) {
  ESP_LOGI(TAG, "=== PARSING HWR RESPONSE TYPE 48 ===");

  // Current known payload: [48, 0, 1, 0, 3, 0, 0, 41]
  // Let's systematically decode this

  if (len < 8) {
    ESP_LOGW(TAG, "Response type 48 too short: %d bytes", len);
    return;
  }

  // Byte 0: Response type (48)
  uint8_t response_type = data[0];
  ESP_LOGI(TAG, "Response type: %d", response_type);

  // Byte 1: Status/Mode indicator (currently 0)
  uint8_t status_mode = data[1];
  ESP_LOGI(TAG, "Status/Mode: %d", status_mode);

  // Bytes 2-3: Could be temperature, flow, or other sensor reading
  uint16_t value_1 = (data[3] << 8) | data[2];  // Little-endian
  uint16_t value_1_be = (data[2] << 8) | data[3]; // Big-endian
  ESP_LOGI(TAG, "Value 1 (LE): %d, (BE): %d", value_1, value_1_be);

  // Bytes 4-5: Another potential sensor value
  uint16_t value_2 = (data[5] << 8) | data[4];  // Little-endian
  uint16_t value_2_be = (data[4] << 8) | data[5]; // Big-endian
  ESP_LOGI(TAG, "Value 2 (LE): %d, (BE): %d", value_2, value_2_be);

  // Byte 6: Reserved/status
  uint8_t reserved = data[6];
  ESP_LOGI(TAG, "Reserved/Status: %d", reserved);

  // Byte 7: Checksum/terminator (currently 41)
  uint8_t checksum = data[7];
  ESP_LOGI(TAG, "Checksum/Terminator: %d", checksum);

  // Hypothesis testing for HWR-specific data:

  // 1. Temperature hypothesis (common in HWR systems)
  if (value_1 > 0 && value_1 < 1000) {  // Reasonable temp range in 0.1°C
    float temp_1 = value_1 / 10.0f;
    ESP_LOGI(TAG, "HYPOTHESIS: Temperature = %.1f°C", temp_1);
    if (this->temperature_sensor_ != nullptr) {
      this->temperature_sensor_->publish_state(temp_1);
    }
  }

  // 2. Flow/Power hypothesis
  if (value_1 > 1000) {  // Could be flow in ml/min or power in mW
    ESP_LOGI(TAG, "HYPOTHESIS: Flow/Power reading = %d", value_1);
  }

  // 3. Status bits hypothesis
  ESP_LOGI(TAG, "Status analysis:");
  ESP_LOGI(TAG, "  Pump running: %s", (status_mode & 0x01) ? "YES" : "NO");
  ESP_LOGI(TAG, "  Error state: %s", (status_mode & 0x02) ? "YES" : "NO");
  ESP_LOGI(TAG, "  Schedule active: %s", (status_mode & 0x04) ? "YES" : "NO");
  ESP_LOGI(TAG, "  Hot water demand: %s", (status_mode & 0x08) ? "YES" : "NO");

  // 4. Try to identify if this is a response to a specific command
  ESP_LOGI(TAG, "Command correlation:");
  if (!this->protocol_log_.empty()) {
    auto& last_entry = this->protocol_log_.back();
    ESP_LOGI(TAG, "  Time since last command: %d ms", millis() - last_entry.timestamp);

    // Pattern matching based on command sent
    if (last_entry.command.size() >= 7) {
      uint8_t cmd_type = last_entry.command[6];  // Command byte from GENI protocol
      ESP_LOGI(TAG, "  Last command type: 0x%02X", cmd_type);

      switch (cmd_type) {
        case 0x5D:  // Flow/Head request
          ESP_LOGI(TAG, "  This might be flow/head response");
          break;
        case 0x57:  // Power request
          ESP_LOGI(TAG, "  This might be power response");
          break;
        case 0x54:  // Temperature request (hypothetical)
          ESP_LOGI(TAG, "  This might be temperature response");
          break;
      }
    }
  }

  ESP_LOGI(TAG, "=== END PARSING TYPE 48 ===");
}

// Method to save protocol log to persistent storage (optional)
void Alpha_HWR::dump_protocol_log() {
  ESP_LOGI(TAG, "=== PROTOCOL LOG DUMP ===");
  ESP_LOGI(TAG, "Total entries: %d", this->protocol_log_.size());

  for (size_t i = 0; i < this->protocol_log_.size(); i++) {
    const auto& entry = this->protocol_log_[i];
    ESP_LOGI(TAG, "Entry %d: time=%d, cmd_len=%d, resp_len=%d, type=%d",
             i, entry.timestamp, entry.command.size(), entry.response.size(), entry.response_type);

    if (!entry.command.empty()) {
      this->log_protocol_data("CMD", entry.command.data(), entry.command.size());
    }
    if (!entry.response.empty()) {
      this->log_protocol_data("RESP", entry.response.data(), entry.response.size());
    }
  }

  ESP_LOGI(TAG, "=== END PROTOCOL LOG ===");
}
void Alpha_HWR::send_test_command(uint8_t cmd_type) {
  uint8_t cmd[] = {39, 7, 231, 248, 10, 3, cmd_type, 0, 69, 138, 205};
  
  ESP_LOGI(TAG, "Sending test command: 0x%02X", cmd_type);
  this->log_protocol_data("TEST_CMD", cmd, sizeof(cmd));
  
  // Store command for correlation
  if (this->protocol_discovery_mode_) {
    ProtocolLogEntry entry;
    entry.timestamp = millis();
    entry.command.assign(cmd, cmd + sizeof(cmd));
    entry.notes = "Manual test command";
    this->protocol_log_.push_back(entry);
  }
  
  this->send_request_(cmd, sizeof(cmd));
}
void Alpha_HWR::analyze_response_type_48(const uint8_t* data, size_t len) {
  ESP_LOGI(TAG, "=== ANALYZING HWR RESPONSE TYPE 48 ===");
  ESP_LOGI(TAG, "Response type array (first 8 bytes):");
  this->log_protocol_data("TYPE_48", data, std::min(len, (size_t)8));
  
  // Call your existing detailed parser
  this->parse_hwr_response_48(data, len);
  
  ESP_LOGI(TAG, "=== END HWR ANALYSIS ===");
}
void Alpha_HWR::log_protocol_data(const char* prefix, const uint8_t* data, size_t len) {
  if (len == 0) return;

  std::string hex_string;
  std::string dec_string;
  hex_string.reserve(len * 3);
  dec_string.reserve(len * 4);

  for (size_t i = 0; i < len; i++) {
    if (i > 0) {
      hex_string += " ";
      dec_string += " ";
    }
    char hex_buf[4];
    char dec_buf[8];
    snprintf(hex_buf, sizeof(hex_buf), "%02X", data[i]);
    snprintf(dec_buf, sizeof(dec_buf), "%d", data[i]);
    hex_string += hex_buf;
    dec_string += dec_buf;
  }

  ESP_LOGI(TAG, "%s HEX: [%s]", prefix, hex_string.c_str());
  ESP_LOGI(TAG, "%s DEC: [%s]", prefix, dec_string.c_str());
  ESP_LOGI(TAG, "%s LEN: %d", prefix, len);
}

void Alpha_HWR::log_raw_protocol_data(const char* direction, const uint8_t* data, size_t len, const char* context) {
  if (len == 0) return;

  // Log to protocol capture system
  this->protocol_capture_.log_packet(direction, data, len, context);

  // Enhanced console logging with timestamp and context
  uint32_t timestamp = millis();
  ESP_LOGI(TAG, "=== RAW PROTOCOL [%s] [%s] ===", direction, context);
  ESP_LOGI(TAG, "Timestamp: %d ms", timestamp);
  ESP_LOGI(TAG, "Length: %d bytes", len);

  // Hex dump in 16-byte rows (like hexdump -C)
  for (size_t i = 0; i < len; i += 16) {
    std::string hex_line, ascii_line;
    char addr_buf[16];
    snprintf(addr_buf, sizeof(addr_buf), "%04x:", (unsigned int)i);

    // Hex bytes
    for (size_t j = 0; j < 16; j++) {
      if (i + j < len) {
        char hex_buf[4];
        snprintf(hex_buf, sizeof(hex_buf), " %02x", data[i + j]);
        hex_line += hex_buf;

        // ASCII representation
        uint8_t byte = data[i + j];
        ascii_line += (byte >= 32 && byte <= 126) ? (char)byte : '.';
      } else {
        hex_line += "   ";
        ascii_line += " ";
      }
    }

    ESP_LOGI(TAG, "%s%s |%s|", addr_buf, hex_line.c_str(), ascii_line.c_str());
  }

  // One-line format for easy copy/paste
  std::string hex_oneline;
  for (size_t i = 0; i < len; i++) {
    char hex_buf[4];
    snprintf(hex_buf, sizeof(hex_buf), "%02X", data[i]);
    hex_oneline += hex_buf;
    if (i < len - 1) hex_oneline += " ";
  }
  ESP_LOGI(TAG, "HEX: [%s]", hex_oneline.c_str());
  ESP_LOGI(TAG, "=== END RAW PROTOCOL ===");
}

void Alpha_HWR::send_raw_command(const std::string& hex_string) {
  // Parse hex string "27 07 E7 F8 0A 03 54 00 45 8A CD"
  std::vector<uint8_t> bytes;
  std::string hex_clean = hex_string;

  // Remove any non-hex characters
  std::string cleaned;
  for (char c : hex_clean) {
    if (std::isxdigit(c)) {
      cleaned += c;
    }
  }

  // Parse pairs of hex digits
  for (size_t i = 0; i < cleaned.length(); i += 2) {
    if (i + 1 < cleaned.length()) {
      std::string hex_pair = cleaned.substr(i, 2);
      uint8_t byte = (uint8_t)std::strtoul(hex_pair.c_str(), nullptr, 16);
      bytes.push_back(byte);
    }
  }

  if (bytes.empty()) {
    ESP_LOGW(TAG, "Invalid hex string: %s", hex_string.c_str());
    return;
  }

  ESP_LOGI(TAG, "=== SENDING RAW COMMAND ===");
  ESP_LOGI(TAG, "Input: %s", hex_string.c_str());
  this->log_raw_protocol_data("TX", bytes.data(), bytes.size(), "RAW_COMMAND");

  this->send_request_(bytes.data(), bytes.size());
}


void Alpha_HWR::send_known_command(const std::string& name) {
  for (const auto& cmd : this->known_commands_) {
    if (cmd.name == name) {
      ESP_LOGI(TAG, "Sending known command: %s (%s)", name.c_str(), cmd.description.c_str());
      this->log_raw_protocol_data("TX", cmd.data.data(), cmd.data.size(), cmd.name.c_str());
      this->send_request_((uint8_t*)cmd.data.data(), cmd.data.size());
      return;
    }
  }
  ESP_LOGW(TAG, "Unknown command: %s", name.c_str());
}

// 7. SYSTEMATIC COMMAND SWEEPER
void Alpha_HWR::sweep_command_range(uint8_t start_cmd, uint8_t end_cmd, uint8_t param) {
  static uint8_t current_cmd = start_cmd;
  static uint32_t last_sweep_time = 0;
  
  if (millis() - last_sweep_time < 3000) return;  // 3 second intervals
  
  if (current_cmd <= end_cmd) {
    ESP_LOGI(TAG, "=== COMMAND SWEEP: 0x%02X (param=%d) ===", current_cmd, param);
    uint8_t cmd[] = {39, 7, 231, 248, 10, 3, current_cmd, param, 69, 138, 205};
    
    char context[32];
    snprintf(context, sizeof(context), "SWEEP_0x%02X", current_cmd);
    this->log_raw_protocol_data("TX", cmd, sizeof(cmd), context);
    this->send_request_(cmd, sizeof(cmd));
    
    current_cmd++;
    last_sweep_time = millis();
  } else {
    ESP_LOGI(TAG, "Command sweep complete (0x%02X to 0x%02X)", start_cmd, end_cmd);
    current_cmd = start_cmd;  // Reset for next sweep
  }
}

// 8. PROTOCOL ANALYSIS TOOLS
void Alpha_HWR::analyze_response_patterns() {
  ESP_LOGI(TAG, "=== RESPONSE PATTERN ANALYSIS ===");
  
  std::map<uint8_t, int> response_type_counts;
  std::map<std::string, int> pattern_counts;
  
  for (const auto& packet : this->protocol_capture_.packets) {
    if (packet.direction == "RX" && !packet.data.empty()) {
      response_type_counts[packet.data[0]]++;
      
      // Create pattern string for first few bytes
      std::string pattern;
      for (size_t i = 0; i < std::min((size_t)4, packet.data.size()); i++) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%02X", packet.data[i]);
        pattern += buf;
        if (i < 3) pattern += " ";
      }
      pattern_counts[pattern]++;
    }
  }
  
  ESP_LOGI(TAG, "Response type frequencies:");
  for (const auto& pair : response_type_counts) {
    ESP_LOGI(TAG, "  Type %d: %d occurrences", pair.first, pair.second);
  }
  
  ESP_LOGI(TAG, "Common response patterns:");
  for (const auto& pair : pattern_counts) {
    ESP_LOGI(TAG, "  [%s]: %d occurrences", pair.first.c_str(), pair.second);
  }
  
  ESP_LOGI(TAG, "=== END ANALYSIS ===");
}

}  // namespace alpha_hwr
}  // namespace esphome

#endif
