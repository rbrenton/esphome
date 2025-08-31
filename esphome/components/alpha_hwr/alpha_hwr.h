#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include <vector>
#include <string>

#ifdef USE_ESP32

#include <esp_gattc_api.h>

namespace esphome {
namespace alpha_hwr {

namespace espbt = esphome::esp32_ble_tracker;

static const espbt::ESPBTUUID ALPHA_HWR_GENI_SERVICE_UUID = espbt::ESPBTUUID::from_uint16(0xfe5d);
static const espbt::ESPBTUUID ALPHA_HWR_GENI_CHARACTERISTIC_UUID =
    espbt::ESPBTUUID::from_raw({static_cast<char>(0xa9), 0x7b, static_cast<char>(0xb8), static_cast<char>(0x85), 0x0,
                                0x1a, 0x28, static_cast<char>(0xaa), 0x2a, 0x43, 0x6e, 0x3, static_cast<char>(0xd1),
                                static_cast<char>(0xff), static_cast<char>(0x9c), static_cast<char>(0x85)});

class Alpha_HWR : public esphome::ble_client::BLEClientNode, public Component {
 public:
  void setup() override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void dump_config() override;
  
  // Public method for sending commands
  void send_hex_command(const std::string& hex);

 protected:
  uint16_t geni_handle_{0};
  
  void log_packet(const char* direction, const uint8_t* data, size_t len);
  void send_request(const uint8_t* data, size_t len);
  std::vector<uint8_t> parse_hex(const std::string& hex);
};

}  // namespace alpha_hwr
}  // namespace esphome

#endif
