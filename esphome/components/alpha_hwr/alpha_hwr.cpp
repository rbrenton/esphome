#include "alpha_hwr.h"
#include "esphome/core/log.h"
#include <cctype>

#ifdef USE_ESP32

namespace esphome {
namespace alpha_hwr {

static const char *const TAG = "alpha_hwr";

void Alpha_HWR::dump_config() {
  ESP_LOGCONFIG(TAG, "Alpha HWR Protocol Discovery");
  ESP_LOGCONFIG(TAG, "  MAC: %s", this->parent_->address_str().c_str());
}

void Alpha_HWR::setup() {
  ESP_LOGI(TAG, "Protocol discovery ready");
}

void Alpha_HWR::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, 
                                    esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT:
      ESP_LOGI(TAG, "Connected to %s", this->parent_->address_str().c_str());
      break;
      
    case ESP_GATTC_CONNECT_EVT:
      if (std::memcmp(param->connect.remote_bda, this->parent_->get_remote_bda(), 6) == 0) {
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
      }
      break;
      
    case ESP_GATTC_DISCONNECT_EVT:
      ESP_LOGI(TAG, "Disconnected from %s", this->parent_->address_str().c_str());
      this->node_state = espbt::ClientState::IDLE;
      break;
      
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      auto *chr = this->parent_->get_characteristic(ALPHA_HWR_GENI_SERVICE_UUID, 
                                                    ALPHA_HWR_GENI_CHARACTERISTIC_UUID);
      if (!chr) {
        ESP_LOGE(TAG, "GENI service not found");
        break;
      }
      
      this->geni_handle_ = chr->handle;
      auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), 
                                                      this->parent_->get_remote_bda(), 
                                                      chr->handle);
      if (status) {
        ESP_LOGW(TAG, "Failed to register for notifications: %d", status);
      }
      break;
    }
    
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
      ESP_LOGI(TAG, "Ready for communication");
      this->node_state = espbt::ClientState::ESTABLISHED;
      break;
      
    case ESP_GATTC_NOTIFY_EVT:
      if (param->notify.handle == this->geni_handle_) {
        this->log_packet("RX", param->notify.value, param->notify.value_len);
      }
      break;
      
    default:
      break;
  }
}

void Alpha_HWR::log_packet(const char* direction, const uint8_t* data, size_t len) {
  if (len == 0) return;
  
  // Build hex string
  std::string hex;
  for (size_t i = 0; i < len; i++) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X", data[i]);
    hex += buf;
    if (i < len - 1) hex += " ";
  }
  
  // Log with timestamp
  ESP_LOGI(TAG, "%s [%3d]: %s", direction, len, hex.c_str());
  
  // ASCII representation for readable bytes
  std::string ascii;
  for (size_t i = 0; i < len; i++) {
    ascii += (data[i] >= 32 && data[i] <= 126) ? (char)data[i] : '.';
  }
  ESP_LOGD(TAG, "%s ASCII: %s", direction, ascii.c_str());
}

void Alpha_HWR::send_request(const uint8_t* data, size_t len) {
  if (this->node_state != espbt::ClientState::ESTABLISHED) {
    ESP_LOGW(TAG, "Not connected");
    return;
  }
  
  this->log_packet("TX", data, len);
  
  auto status = esp_ble_gattc_write_char(this->parent_->get_gattc_if(), 
                                         this->parent_->get_conn_id(), 
                                         this->geni_handle_, 
                                         len, 
                                         (uint8_t*)data, 
                                         ESP_GATT_WRITE_TYPE_NO_RSP, 
                                         ESP_GATT_AUTH_REQ_NONE);
  if (status) {
    ESP_LOGW(TAG, "Write failed: %d", status);
  }
}

std::vector<uint8_t> Alpha_HWR::parse_hex(const std::string& hex) {
  std::vector<uint8_t> bytes;
  std::string clean;
  
  // Remove non-hex characters
  for (char c : hex) {
    if (std::isxdigit(c)) {
      clean += c;
    }
  }
  
  // Parse hex pairs
  for (size_t i = 0; i + 1 < clean.length(); i += 2) {
    uint8_t byte = std::strtoul(clean.substr(i, 2).c_str(), nullptr, 16);
    bytes.push_back(byte);
  }
  
  return bytes;
}

void Alpha_HWR::send_hex_command(const std::string& hex) {
  auto bytes = this->parse_hex(hex);
  if (!bytes.empty()) {
    this->send_request(bytes.data(), bytes.size());
  } else {
    ESP_LOGW(TAG, "Invalid hex: %s", hex.c_str());
  }
}

}  // namespace alpha_hwr
}  // namespace esphome

#endif
