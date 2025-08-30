#include "alpha_hwr.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <lwip/sockets.h>  //gives ntohl

#ifdef USE_ESP32

namespace esphome {
namespace alpha_hwr {

static const char *const TAG = "alpha_hwr";

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
  if (this->response_offset_ >= this->response_length_) {
    ESP_LOGD(TAG, "[%s] GENI response begin", this->parent_->address_str().c_str());
    if (length < GENI_RESPONSE_HEADER_LENGTH) {
      ESP_LOGW(TAG, "[%s] response to short", this->parent_->address_str().c_str());
      return;
    }
    if (response[0] != 36 || response[2] != 248 || response[3] != 231 || response[4] != 10) {
      ESP_LOGW(TAG, "[%s] response bytes %d %d %d %d %d don't match GENI HEADER", this->parent_->address_str().c_str(),
               response[0], response[1], response[2], response[3], response[4]);
      return;
    }
    this->response_length_ = response[1] - GENI_RESPONSE_HEADER_LENGTH + 2;  // maybe 2 byte checksum
    this->response_offset_ = -GENI_RESPONSE_HEADER_LENGTH;
    std::memcpy(this->response_type_, response + 5, GENI_RESPONSE_TYPE_LENGTH);
  }

  auto extract_publish_sensor_value = [response, length, this](int16_t value_offset, sensor::Sensor *sensor,
                                                               float factor) {
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
    ESP_LOGW(TAG, "unkown GENI response Type %d %d %d %d %d %d %d %d", this->response_type_[0], this->response_type_[1],
             this->response_type_[2], this->response_type_[3], this->response_type_[4], this->response_type_[5],
             this->response_type_[6], this->response_type_[7]);
  }
  this->response_offset_ += length;
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

}  // namespace alpha_hwr
}  // namespace esphome

#endif
