#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>

#ifdef USE_ESP32

#include <esp_gattc_api.h>

namespace esphome {
namespace alpha_hwr {

namespace espbt = esphome::esp32_ble_tracker;

class ProtocolCapture {
public:
  struct PacketEntry {
    uint32_t timestamp;
    std::string direction;  // "TX" or "RX"
    std::vector<uint8_t> data;
    std::string notes;
  };

  std::vector<PacketEntry> packets;
  bool enabled = true;

  void log_packet(const std::string& direction, const uint8_t* data, size_t len, const std::string& notes = "");
};

struct KnownCommand {
  std::string name;
  std::vector<uint8_t> data;
  std::string description;
};

static const espbt::ESPBTUUID ALPHA_HWR_GENI_SERVICE_UUID = espbt::ESPBTUUID::from_uint16(0xfe5d);
static const espbt::ESPBTUUID ALPHA_HWR_GENI_CHARACTERISTIC_UUID =
    espbt::ESPBTUUID::from_raw({static_cast<char>(0xa9), 0x7b, static_cast<char>(0xb8), static_cast<char>(0x85), 0x0,
                                0x1a, 0x28, static_cast<char>(0xaa), 0x2a, 0x43, 0x6e, 0x3, static_cast<char>(0xd1),
                                static_cast<char>(0xff), static_cast<char>(0x9c), static_cast<char>(0x85)});
static const int16_t GENI_RESPONSE_HEADER_LENGTH = 13;
static const size_t GENI_RESPONSE_TYPE_LENGTH = 8;

static const uint8_t GENI_RESPONSE_TYPE_FLOW_HEAD[GENI_RESPONSE_TYPE_LENGTH] = {31, 0, 1, 48, 1, 0, 0, 24};
static const int16_t GENI_RESPONSE_FLOW_OFFSET = 0;
static const int16_t GENI_RESPONSE_HEAD_OFFSET = 4;

static const uint8_t GENI_RESPONSE_TYPE_POWER[GENI_RESPONSE_TYPE_LENGTH] = {44, 0, 1, 0, 1, 0, 0, 37};
static const int16_t GENI_RESPONSE_VOLTAGE_AC_OFFSET = 0;
static const int16_t GENI_RESPONSE_VOLTAGE_DC_OFFSET = 4;
static const int16_t GENI_RESPONSE_CURRENT_OFFSET = 8;
static const int16_t GENI_RESPONSE_POWER_OFFSET = 12;
static const int16_t GENI_RESPONSE_MOTOR_POWER_OFFSET = 16;  // not sure
static const int16_t GENI_RESPONSE_MOTOR_SPEED_OFFSET = 20;

class Alpha_HWR : public esphome::ble_client::BLEClientNode, public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void dump_config() override;

  void set_flow_sensor(sensor::Sensor *sensor) { this->flow_sensor_ = sensor; }
  void set_head_sensor(sensor::Sensor *sensor) { this->head_sensor_ = sensor; }
  void set_power_sensor(sensor::Sensor *sensor) { this->power_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { this->current_sensor_ = sensor; }
  void set_speed_sensor(sensor::Sensor *sensor) { this->speed_sensor_ = sensor; }
  void set_voltage_sensor(sensor::Sensor *sensor) { this->voltage_sensor_ = sensor; }
  void set_temperature_sensor(sensor::Sensor *sensor) { this->temperature_sensor_ = sensor; }
  void set_energy_sensor(sensor::Sensor *sensor) { this->energy_sensor_ = sensor; }

  void send_raw_command(const std::string& hex_string);
  void send_known_command(const std::string& name);
  void export_protocol_capture();
  //void clear_protocol_capture();
  void sweep_command_range(uint8_t start_cmd, uint8_t end_cmd, uint8_t param = 0);
  void analyze_response_patterns();
  //void set_debug_mode(bool enabled);
  //void set_capture_enabled(bool enabled);

  void dump_protocol_log();
  void send_test_command(uint8_t cmd_type);

 protected:
  sensor::Sensor *flow_sensor_{nullptr};
  sensor::Sensor *head_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};

  uint16_t geni_handle_;
  int16_t response_length_;
  int16_t response_offset_;
  uint8_t response_type_[GENI_RESPONSE_TYPE_LENGTH];
  uint8_t buffer_[4];

  struct ProtocolLogEntry {
    uint32_t timestamp;
    std::vector<uint8_t> command;
    std::vector<uint8_t> response;
    uint8_t response_type;
    std::string notes;
  };

  std::vector<ProtocolLogEntry> protocol_log_;
  uint8_t discovery_phase_ = 0;
  uint32_t last_discovery_command_ = 0;
  bool protocol_discovery_mode_ = true;

  ProtocolCapture protocol_capture_;
  std::vector<KnownCommand> known_commands_;
  //bool debug_mode_ = true;

  void extract_publish_sensor_value_(const uint8_t *response, int16_t length, int16_t response_offset,
                                     int16_t value_offset, sensor::Sensor *sensor, float factor);
  void handle_geni_response_(const uint8_t *response, uint16_t length);
  void send_request_(uint8_t *request, size_t len);
  bool is_current_response_type_(const uint8_t *response_type);

  void log_raw_protocol_data(const char* direction, const uint8_t* data, size_t len, const char* context);
  void initialize_known_commands();

  // Discovery methods
  void log_protocol_data(const char* prefix, const uint8_t* data, size_t len);
  void analyze_response_type_48(const uint8_t* data, size_t len);
  void test_discovery_commands();
  void parse_hwr_response_48(const uint8_t* data, size_t len);
};
}  // namespace alpha_hwr
}  // namespace esphome

#endif
