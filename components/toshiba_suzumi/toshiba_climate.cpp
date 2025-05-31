#include "toshiba_climate.h"
#include "toshiba_climate_mode.h"
#include "esphome/core/log.h"

namespace esphome {
namespace toshiba_suzumi {

using namespace esphome::climate;

static const int RECEIVE_TIMEOUT = 200;
static const int COMMAND_DELAY = 100;

/**
 * Checksum is calculated from all bytes excluding start byte.
 * It's (256 - (sum % 256)).
 */
uint8_t checksum(std::vector<uint8_t> data, uint8_t length) {
  uint8_t sum = 0;
  for (size_t i = 1; i < length; i++) {
    sum += data[i];
  }
  return 256 - sum;
}

/**
 * Send the command to UART interface.
 */
void ToshibaClimateUart::send_to_uart(ToshibaCommand command) {
  this->last_command_timestamp_ = millis();
  ESP_LOGV(TAG, "Sending: [%s]", format_hex_pretty(command.payload).c_str());
  this->write_array(command.payload);
}

/**
 * Send starting handshake to initialize communication with the unit.
 */
void ToshibaClimateUart::start_handshake() {
  ESP_LOGCONFIG(TAG, "Sending handshake...");
  enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::HANDSHAKE, .payload = HANDSHAKE[0]});
  enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::HANDSHAKE, .payload = HANDSHAKE[1]});
  enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::HANDSHAKE, .payload = HANDSHAKE[2]});
  enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::HANDSHAKE, .payload = HANDSHAKE[3]});
  enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::HANDSHAKE, .payload = HANDSHAKE[4]});
  enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::HANDSHAKE, .payload = HANDSHAKE[5]});
  enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::DELAY, .delay = 2000});
  enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::HANDSHAKE, .payload = AFTER_HANDSHAKE[0]});
  enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::HANDSHAKE, .payload = AFTER_HANDSHAKE[1]});
}

/**
 * Handle data in RX buffer, validate message for content and checksum.
 * Since we know the format only of some messages (expected length), unknown messages
 * are ended via RECIEVE timeout.
 */
bool ToshibaClimateUart::validate_message_() {
  uint8_t at = this->rx_message_.size() - 1;
  auto *data = &this->rx_message_[0];
  uint8_t new_byte = data[at];

  // Byte 0: HEADER (always 0x02)
  if (at == 0)
    return new_byte == 0x02;

  // always get first three bytes
  if (at < 2) {
    return true;
  }

  // Byte 3
  if (data[2] != 0x03) {
    // Normal commands starts with 0x02 0x00 0x03 and have length between 15-17 bytes.
    // however there are some special unknown handshake commands which has non-standard replies.
    // Since we don't know their format, we can't validate them.
    return true;
  }

  if (at <= 5) {
    // no validation for these fields
    return true;
  }

  // Byte 7: LENGTH
  uint8_t length = 6 + data[6] + 1;  // prefix + data + checksum

  // wait until all data is read
  if (at < length)
    return true;

  // last byte: CHECKSUM
  uint8_t rx_checksum = new_byte;
  uint8_t calc_checksum = checksum(this->rx_message_, at);

  if (rx_checksum != calc_checksum) {
    ESP_LOGW(TAG, "Received invalid message checksum %02X!=%02X DATA=[%s]", rx_checksum, calc_checksum,
             format_hex_pretty(data, length).c_str());
    return false;
  }

  // valid message
  ESP_LOGV(TAG, "Received: DATA=[%s]", format_hex_pretty(data, length).c_str());
  this->parseResponse(this->rx_message_);

  // return false to reset rx buffer
  return false;
}

void ToshibaClimateUart::enqueue_command_(const ToshibaCommand &command) {
  this->command_queue_.push_back(command);
  this->process_command_queue_();
}

void ToshibaClimateUart::sendCmd(ToshibaCommandType cmd, uint8_t value) {
  std::vector<uint8_t> payload = {2, 0, 3, 16, 0, 0, 7, 1, 48, 1, 0, 2};
  payload.push_back(static_cast<uint8_t>(cmd));
  payload.push_back(value);
  payload.push_back(checksum(payload, payload.size()));
  ESP_LOGD(TAG, "Sending ToshibaCommand: %d, value: %d, checksum: %d", cmd, value, payload[14]);
  this->enqueue_command_(ToshibaCommand{.cmd = cmd, .payload = std::vector<uint8_t>{payload}});
}

void ToshibaClimateUart::requestData(ToshibaCommandType cmd) {
  std::vector<uint8_t> payload = {2, 0, 3, 16, 0, 0, 6, 1, 48, 1, 0, 1};
  payload.push_back(static_cast<uint8_t>(cmd));
  payload.push_back(checksum(payload, payload.size()));
  ESP_LOGI(TAG, "Requesting data from sensor %d, checksum: %d", payload[12], payload[13]);
  this->enqueue_command_(ToshibaCommand{.cmd = cmd, .payload = std::vector<uint8_t>{payload}});
}

void ToshibaClimateUart::getInitData() {
  ESP_LOGD(TAG, "Requesting initial data from AC unit");
  this->requestData(ToshibaCommandType::POWER_STATE);
  this->requestData(ToshibaCommandType::MODE);
  this->requestData(ToshibaCommandType::TARGET_TEMP);
  this->requestData(ToshibaCommandType::FAN);
  this->requestData(ToshibaCommandType::POWER_SEL);
  this->requestData(ToshibaCommandType::SWING);
  this->requestData(ToshibaCommandType::ROOM_TEMP);
  this->requestData(ToshibaCommandType::OUTDOOR_TEMP);
  this->requestData(ToshibaCommandType::SPECIAL_MODE);
}

void ToshibaClimateUart::getExtendedInitData() {
  ESP_LOGD(TAG, "Requesting extended data from AC unit");
  // Request new feature states
  this->requestData(ToshibaCommandType::AIR_QUALITY);
  this->requestData(ToshibaCommandType::DISPLAY_BRIGHTNESS);
  this->requestData(ToshibaCommandType::DRY_LEVEL);
  this->requestData(ToshibaCommandType::I_FEEL);
  this->requestData(ToshibaCommandType::TURBO_MODE);
  this->requestData(ToshibaCommandType::BEEP_CONTROL);
  this->requestData(ToshibaCommandType::AUTO_RESTART);
  this->requestData(ToshibaCommandType::ON_TIMER);
  this->requestData(ToshibaCommandType::OFF_TIMER);
  this->requestData(ToshibaCommandType::SLEEP_TIMER);
  this->requestData(ToshibaCommandType::FILTER_TIME);
  this->requestData(ToshibaCommandType::OPERATING_HOURS);
  this->requestData(ToshibaCommandType::ERROR_CODE);
}

void ToshibaClimateUart::setup() {
  // establish communication
  this->start_handshake();
  // load initial sensor data from the unit
  this->getInitData();
  // load extended feature data
  this->getExtendedInitData();

  if (this->wifi_led_disabled_) {
    // Disable Wifi LED
    this->sendCmd(ToshibaCommandType::WIFI_LED, 128);
  }
}

/**
 * Detect RX timeout and send next command in the queue to the unit.
 */
void ToshibaClimateUart::process_command_queue_() {
  uint32_t now = millis();
  uint32_t cmdDelay = now - this->last_command_timestamp_;

  // when we have not processed message and timeout since last received byte has expired,
  // we likely won't receive any more data and there is nothing we can do with the message as it's
  // format is was not recognized by validate_message_ function.
  // Nothing to do - drop the message to free up communication and allow to send next command.
  if (now - this->last_rx_char_timestamp_ > RECEIVE_TIMEOUT) {
    this->rx_message_.clear();
  }

  // Add communication health logging
  if (cmdDelay > 5000 && !this->command_queue_.empty()) {
    ESP_LOGW(TAG, "Communication slow - %d commands queued", this->command_queue_.size());
  }

  // when there is no RX message and there is a command to send
  if (cmdDelay > COMMAND_DELAY && !this->command_queue_.empty() && this->rx_message_.empty()) {
    auto newCommand = this->command_queue_.front();
    if (newCommand.cmd == ToshibaCommandType::DELAY && cmdDelay < newCommand.delay) {
      // delay command did not finished yet
      return;
    }
    this->send_to_uart(this->command_queue_.front());
    this->command_queue_.erase(this->command_queue_.begin());
  }
}

/**
 * Handle received byte from UART
 */
void ToshibaClimateUart::handle_rx_byte_(uint8_t c) {
  this->rx_message_.push_back(c);
  if (!validate_message_()) {
    this->rx_message_.clear();
  } else {
    this->last_rx_char_timestamp_ = millis();
  }
}

void ToshibaClimateUart::loop() {
  while (available()) {
    uint8_t c;
    this->read_byte(&c);
    this->handle_rx_byte_(c);
  }
  this->process_command_queue_();
}

void ToshibaClimateUart::parseResponse(std::vector<uint8_t> rawData) {
  uint8_t length = rawData.size();
  ToshibaCommandType sensor;
  uint8_t value;

  switch (length) {
    case 15:  // response to requestData with the actual value of sensor/setting
      sensor = static_cast<ToshibaCommandType>(rawData[12]);
      value = rawData[13];
      break;
    case 16:  // probably ACK for issued command
      ESP_LOGD(TAG, "Received message with length: %d and value %s", length, format_hex_pretty(rawData).c_str());
      return;
    case 17:  // response to requestData with the actual value of sensor/setting
      sensor = static_cast<ToshibaCommandType>(rawData[14]);
      value = rawData[15];
      break;
    default:
      ESP_LOGW(TAG, "Received unknown message with length: %d and value %s", length,
               format_hex_pretty(rawData).c_str());
      return;
  }
  
  switch (sensor) {
    case ToshibaCommandType::TARGET_TEMP:
      ESP_LOGI(TAG, "Received target temp: %d", value);
      if (this->special_mode_ == SPECIAL_MODE::EIGHT_DEG) {
        // if special mode is EIGHT_DEG, shift the target temperature by SPECIAL_TEMP_OFFSET
        value -= SPECIAL_TEMP_OFFSET;
        ESP_LOGI(TAG, "Note: Special Mode \"%s\" is active, shifting target temp to %d", SPECIAL_MODE_EIGHT_DEG.c_str(), value);
      }
      this->target_temperature = value;
      break;
      
    case ToshibaCommandType::FAN: {
      FAN fan_enum = static_cast<FAN>(value);
      if (fan_enum == FAN::FAN_AUTO) {
        ESP_LOGI(TAG, "Fan: AUTO (Variable Speed)");
        this->set_fan_mode_(CLIMATE_FAN_AUTO);
      } else if (fan_enum == FAN::FAN_QUIET) {
        ESP_LOGI(TAG, "Fan: QUIET (~300 RPM)");
        this->set_fan_mode_(CLIMATE_FAN_QUIET);
      } else if (fan_enum == FAN::FAN_LOW) {
        ESP_LOGI(TAG, "Fan: LOW (~500 RPM)");
        this->set_fan_mode_(CLIMATE_FAN_LOW);
      } else if (fan_enum == FAN::FAN_MEDIUM) {
        ESP_LOGI(TAG, "Fan: MEDIUM (~750 RPM)");
        this->set_fan_mode_(CLIMATE_FAN_MEDIUM);
      } else if (fan_enum == FAN::FAN_HIGH) {
        ESP_LOGI(TAG, "Fan: HIGH (~1000 RPM)");
        this->set_fan_mode_(CLIMATE_FAN_HIGH);
      } else {
        auto fanMode = IntToCustomFanMode(fan_enum);
        ESP_LOGI(TAG, "Received fan mode: %s", fanMode.c_str());
        this->set_custom_fan_mode_(fanMode);
      }
      break;
    }
    
    case ToshibaCommandType::SWING: {
      auto swingMode = IntToClimateSwingMode(static_cast<SWING>(value));
      ESP_LOGI(TAG, "Received swing mode: %s", climate_swing_mode_to_string(swingMode));
      this->swing_mode = swingMode;
      break;
    }
    
    case ToshibaCommandType::MODE: {
      auto mode = IntToClimateMode(static_cast<MODE>(value));
      ESP_LOGI(TAG, "Received AC mode: %s", climate_mode_to_string(mode));
      if (this->power_state_ == STATE::ON) {
        this->mode = mode;
      }
      break;
    }
    
    case ToshibaCommandType::ROOM_TEMP:
      ESP_LOGI(TAG, "Received room temp: %d °C", value);
      this->current_temperature = value;
      break;
      
    case ToshibaCommandType::OUTDOOR_TEMP:
      if (outdoor_temp_sensor_ != nullptr) {
        ESP_LOGI(TAG, "Received outdoor temp: %d °C", (int8_t) value);
        outdoor_temp_sensor_->publish_state((int8_t) value);
      }
      break;
      
    case ToshibaCommandType::POWER_SEL: {
      auto pwr_level = IntToPowerLevel(static_cast<PWR_LEVEL>(value));
      ESP_LOGI(TAG, "Power Level: %s (Estimated: %dW)", pwr_level.c_str(), 
               value == 100 ? 2500 : value == 75 ? 1875 : 1250);
      if (pwr_select_ != nullptr) {
        pwr_select_->publish_state(pwr_level);
      }
      break;
    }
    
    case ToshibaCommandType::POWER_STATE: {
      auto climateState = static_cast<STATE>(value);
      ESP_LOGI(TAG, "Received AC unit power state: %s", climate_state_to_string(climateState));
      if (climateState == STATE::OFF) {
        // AC unit was just powered off, set mode to OFF
        this->mode = climate::CLIMATE_MODE_OFF;
      } else if (this->mode == climate::CLIMATE_MODE_OFF && climateState == STATE::ON) {
        // AC unit was just powered on, query unit for it's MODE
        this->requestData(ToshibaCommandType::MODE);
      }
      this->power_state_ = climateState;
      break;
    }
    
    case ToshibaCommandType::SPECIAL_MODE: {
      this->special_mode_ = static_cast<SPECIAL_MODE>(value);
      auto special_mode = IntToSpecialMode(this->special_mode_.value());
      
      // Add mode-specific info
      if (this->special_mode_ == SPECIAL_MODE::EIGHT_DEG) {
        ESP_LOGI(TAG, "Special mode: %s (Frost Protection: 5-13°C)", special_mode.c_str());
      } else if (this->special_mode_ == SPECIAL_MODE::ECO) {
        ESP_LOGI(TAG, "Special mode: %s (Energy Saving Active)", special_mode.c_str());
      } else {
        ESP_LOGI(TAG, "Special mode: %s", special_mode.c_str());
      }
      
      if (special_mode_select_ != nullptr) {
        special_mode_select_->publish_state(special_mode);
      }
      break;
    }

    // New feature responses
    case ToshibaCommandType::AIR_QUALITY: {
      this->air_quality_mode_ = static_cast<AIR_QUALITY_MODE>(value);
      auto air_quality_mode = IntToAirQualityMode(this->air_quality_mode_.value());
      ESP_LOGI(TAG, "Received air quality mode: %s", air_quality_mode.c_str());
      if (air_quality_select_ != nullptr) {
        air_quality_select_->publish_state(air_quality_mode);
      }
      break;
    }

    case ToshibaCommandType::DISPLAY_BRIGHTNESS: {
      this->display_brightness_ = static_cast<DISPLAY_BRIGHTNESS>(value);
      auto brightness = IntToDisplayBrightness(this->display_brightness_.value());
      ESP_LOGI(TAG, "Received display brightness: %s", brightness.c_str());
      if (display_brightness_select_ != nullptr) {
        display_brightness_select_->publish_state(brightness);
      }
      break;
    }

    case ToshibaCommandType::DRY_LEVEL: {
      this->dry_level_ = static_cast<DRY_LEVEL>(value);
      auto dry_level = IntToDryLevel(this->dry_level_.value());
      ESP_LOGI(TAG, "Received dry level: %s", dry_level.c_str());
      if (dry_level_select_ != nullptr) {
        dry_level_select_->publish_state(dry_level);
      }
      break;
    }

    case ToshibaCommandType::I_FEEL: {
      this->i_feel_enabled_ = (value == 65);
      ESP_LOGI(TAG, "Received I Feel status: %s", this->i_feel_enabled_ ? "ON" : "OFF");
      if (i_feel_switch_ != nullptr) {
        i_feel_switch_->publish_state(this->i_feel_enabled_);
      }
      break;
    }

    case ToshibaCommandType::TURBO_MODE: {
      this->turbo_mode_enabled_ = (value == 65);
      ESP_LOGI(TAG, "Received Turbo mode status: %s", this->turbo_mode_enabled_ ? "ON" : "OFF");
      if (turbo_switch_ != nullptr) {
        turbo_switch_->publish_state(this->turbo_mode_enabled_);
      }
      break;
    }

    case ToshibaCommandType::BEEP_CONTROL: {
      this->beep_enabled_ = (value == 65);
      ESP_LOGI(TAG, "Received Beep status: %s", this->beep_enabled_ ? "ON" : "OFF");
      if (beep_switch_ != nullptr) {
        beep_switch_->publish_state(this->beep_enabled_);
      }
      break;
    }

    case ToshibaCommandType::AUTO_RESTART: {
      this->auto_restart_enabled_ = (value == 65);
      ESP_LOGI(TAG, "Received Auto Restart status: %s", this->auto_restart_enabled_ ? "ON" : "OFF");
      if (auto_restart_switch_ != nullptr) {
        auto_restart_switch_->publish_state(this->auto_restart_enabled_);
      }
      break;
    }

    case ToshibaCommandType::ON_TIMER: {
      this->on_timer_.hours = value;
      this->on_timer_.enabled = (value > 0);
      ESP_LOGI(TAG, "Received On Timer: %d hours (%s)", value, this->on_timer_.enabled ? "ON" : "OFF");
      if (on_timer_number_ != nullptr) {
        on_timer_number_->publish_state(value);
      }
      this->update_timer_status();
      break;
    }

    case ToshibaCommandType::OFF_TIMER: {
      this->off_timer_.hours = value;
      this->off_timer_.enabled = (value > 0);
      ESP_LOGI(TAG, "Received Off Timer: %d hours (%s)", value, this->off_timer_.enabled ? "ON" : "OFF");
      if (off_timer_number_ != nullptr) {
        off_timer_number_->publish_state(value);
      }
      this->update_timer_status();
      break;
    }

    case ToshibaCommandType::SLEEP_TIMER: {
      this->sleep_timer_.hours = value;
      this->sleep_timer_.enabled = (value > 0);
      ESP_LOGI(TAG, "Received Sleep Timer: %d hours (%s)", value, this->sleep_timer_.enabled ? "ON" : "OFF");
      if (sleep_timer_number_ != nullptr) {
        sleep_timer_number_->publish_state(value);
      }
      this->update_timer_status();
      break;
    }

    case ToshibaCommandType::FILTER_TIME: {
      ESP_LOGI(TAG, "Received filter time: %d hours", value * 10); // Assuming 10-hour increments
      if (filter_time_sensor_ != nullptr) {
        filter_time_sensor_->publish_state(value * 10);
      }
      break;
    }

    case ToshibaCommandType::OPERATING_HOURS: {
      ESP_LOGI(TAG, "Received operating hours: %d hours", value * 10); // Assuming 10-hour increments
      if (operating_hours_sensor_ != nullptr) {
        operating_hours_sensor_->publish_state(value * 10);
      }
      break;
    }

    case ToshibaCommandType::ERROR_CODE: {
      std::string error_msg = "E" + std::to_string(value);
      if (value == 0) {
        error_msg = "No Error";
      }
      ESP_LOGI(TAG, "Received error code: %s", error_msg.c_str());
      if (error_code_sensor_ != nullptr) {
        error_code_sensor_->publish_state(error_msg);
      }
      break;
    }

    default:
      ESP_LOGW(TAG, "Unknown sensor: %d with value %d", sensor, value);
      break;
  }
  
  this->rx_message_.clear();  // message processed, clear buffer
  this->publish_state();      // publish current values to MQTT
}

void ToshibaClimateUart::update_timer_status() {
  std::string status = "None";
  if (this->on_timer_.enabled) {
    status = "On Timer: " + std::to_string(this->on_timer_.hours) + "h";
  } else if (this->off_timer_.enabled) {
    status = "Off Timer: " + std::to_string(this->off_timer_.hours) + "h";
  } else if (this->sleep_timer_.enabled) {
    status = "Sleep Timer: " + std::to_string(this->sleep_timer_.hours) + "h";
  }
  
  if (timer_status_sensor_ != nullptr) {
    timer_status_sensor_->publish_state(status);
  }
}

void ToshibaClimateUart::dump_config() {
  ESP_LOGCONFIG(TAG, "ToshibaClimate:");
  LOG_CLIMATE("", "Thermostat", this);
  
  // Show active features
  ESP_LOGCONFIG(TAG, "  Features:");
  ESP_LOGCONFIG(TAG, "    Horizontal Swing: %s", this->horizontal_swing_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "    WiFi LED Control: %s", this->wifi_led_disabled_ ? "DISABLED" : "ENABLED");
  ESP_LOGCONFIG(TAG, "    Min Temperature: %d°C", this->min_temp_);
  ESP_LOGCONFIG(TAG, "    Special Modes: %s", this->special_mode_select_ ? "ENABLED" : "DISABLED");
  ESP_LOGCONFIG(TAG, "    Air Quality Control: %s", this->air_quality_select_ ? "ENABLED" : "DISABLED");
  ESP_LOGCONFIG(TAG, "    Timer Functions: %s", this->on_timer_number_ ? "ENABLED" : "DISABLED");
  ESP_LOGCONFIG(TAG, "    I Feel Function: %s", this->i_feel_switch_ ? "ENABLED" : "DISABLED");
  ESP_LOGCONFIG(TAG, "    Turbo Mode: %s", this->turbo_switch_ ? "ENABLED" : "DISABLED");
  
  if (outdoor_temp_sensor_ != nullptr) {
    LOG_SENSOR("", "Outdoor Temp", this->outdoor_temp_sensor_);
  }
  if (pwr_select_ != nullptr) {
    LOG_SELECT("", "Power selector", this->pwr_select_);
  }
  if (special_mode_select_ != nullptr) {
    LOG_SELECT("", "Special mode selector", this->special_mode_select_);
  }
  if (air_quality_select_ != nullptr) {
    LOG_SELECT("", "Air quality selector", this->air_quality_select_);
  }
  if (filter_time_sensor_ != nullptr) {
    LOG_SENSOR("", "Filter Time", this->filter_time_sensor_);
  }
  if (operating_hours_sensor_ != nullptr) {
    LOG_SENSOR("", "Operating Hours", this->operating_hours_sensor_);
  }
}

/**
 * Periodically request room and outdoor temperature plus extended data.
 */
void ToshibaClimateUart::update() {
  this->requestData(ToshibaCommandType::ROOM_TEMP);
  if (outdoor_temp_sensor_ != nullptr) {
    this->requestData(ToshibaCommandType::OUTDOOR_TEMP);
    
    // Calculate and log temperature differential
    if (this->current_temperature.has_value() && outdoor_temp_sensor_->state) {
      float diff = this->current_temperature.value() - outdoor_temp_sensor_->state;
      ESP_LOGD(TAG, "Indoor/Outdoor Δ: %.1f°C", diff);
    }
  }
  
  // Request filter and operating hours periodically
  if (filter_time_sensor_ != nullptr) {
    this->requestData(ToshibaCommandType::FILTER_TIME);
  }
  if (operating_hours_sensor_ != nullptr) {
    this->requestData(ToshibaCommandType::OPERATING_HOURS);
  }
  
  // Check for errors
  this->requestData(ToshibaCommandType::ERROR_CODE);
}

void ToshibaClimateUart::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    ClimateMode mode = *call.get_mode();
    ESP_LOGD(TAG, "Setting mode to %s", climate_mode_to_string(mode));
    
    // Add seasonal recommendations
    if (mode == CLIMATE_MODE_HEAT && outdoor_temp_sensor_->state > 20) {
      ESP_LOGW(TAG, "Heating mode selected with outdoor temp %.1f°C", outdoor_temp_sensor_->state);
    } else if (mode == CLIMATE_MODE_COOL && outdoor_temp_sensor_->state < 15) {
      ESP_LOGW(TAG, "Cooling mode selected with outdoor temp %.1f°C", outdoor_temp_sensor_->state);
    }
    
    if (this->mode == CLIMATE_MODE_OFF && mode != CLIMATE_MODE_OFF) {
      ESP_LOGD(TAG, "Setting AC unit power state to ON.");
      this->sendCmd(ToshibaCommandType::POWER_STATE, static_cast<uint8_t>(STATE::ON));
    }
    if (mode == CLIMATE_MODE_OFF) {
      ESP_LOGD(TAG, "Setting AC unit power state to OFF.");
      this->sendCmd(ToshibaCommandType::POWER_STATE, static_cast<uint8_t>(STATE::OFF));
    } else {
      auto requestedMode = ClimateModeToInt(mode);
      this->sendCmd(ToshibaCommandType::MODE, static_cast<uint8_t>(requestedMode));
    }
    this->mode = mode;
  }

  if (call.get_target_temperature().has_value()) {
    auto target_temp = *call.get_target_temperature();
    uint8_t newTargetTemp = (uint8_t) target_temp;
    bool special_mode_changed = false;
    if (newTargetTemp >= MIN_TEMP_STANDARD && this->special_mode_ == SPECIAL_MODE::EIGHT_DEG) {
      // if target temp is above MIN_TEMP_STANDARD and special mode is EIGHT_DEG, change to Standard mode
      this->special_mode_ = SPECIAL_MODE::STANDARD;
      special_mode_changed = true;
      ESP_LOGD(TAG, "Changing to Standard Mode");
    } else if (newTargetTemp < MIN_TEMP_STANDARD && this->special_mode_ != SPECIAL_MODE::EIGHT_DEG) {
      // if target temp is below MIN_TEMP_STANDARD and special mode is not EIGHT_DEG, change to FrostGuard mode
      this->special_mode_ = SPECIAL_MODE::EIGHT_DEG;
      special_mode_changed = true;
      ESP_LOGD(TAG, "Changing to FrostGuard Mode");
    }
    if (special_mode_changed) {
      // send command to change special mode and update HA frontend
      this->sendCmd(ToshibaCommandType::SPECIAL_MODE, static_cast<uint8_t>(this->special_mode_.value()));
      special_mode_select_->publish_state(IntToSpecialMode(this->special_mode_.value()));
    }

    ESP_LOGD(TAG, "Setting target temp to %d", newTargetTemp);
    if (this->special_mode_ == SPECIAL_MODE::EIGHT_DEG) {
      newTargetTemp += SPECIAL_TEMP_OFFSET;
      ESP_LOGD(TAG, "Note: Special Mode \"%s\" active, shifting setpoint temp to %d", SPECIAL_MODE_EIGHT_DEG.c_str(),
               newTargetTemp);
    }
    // set the target temperature from HA to Climate component
    this->target_temperature = target_temp;
    // send command to set the target temperature to the unit
    // (which will be shifted by SPECIAL_TEMP_OFFSET if special mode is active)
    this->sendCmd(ToshibaCommandType::TARGET_TEMP, newTargetTemp);
  }

  if (call.get_fan_mode().has_value()) {
    auto fan_mode = *call.get_fan_mode();
    if (fan_mode == CLIMATE_FAN_AUTO) {
      ESP_LOGD(TAG, "Setting fan mode to %s", climate_fan_mode_to_string(fan_mode));
      this->set_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_AUTO));
    } else if (fan_mode == CLIMATE_FAN_QUIET) {
      ESP_LOGD(TAG, "Setting fan mode to %s", climate_fan_mode_to_string(fan_mode));
      this->set_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_QUIET));
    } else if (fan_mode == CLIMATE_FAN_LOW) {
      ESP_LOGD(TAG, "Setting fan mode to %s", climate_fan_mode_to_string(fan_mode));
      this->set_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_LOW));
    } else if (fan_mode == CLIMATE_FAN_MEDIUM) {
      ESP_LOGD(TAG, "Setting fan mode to %s", climate_fan_mode_to_string(fan_mode));
      this->set_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_MEDIUM));
    } else if (fan_mode == CLIMATE_FAN_HIGH) {
      ESP_LOGD(TAG, "Setting fan mode to %s", climate_fan_mode_to_string(fan_mode));
      this->set_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_HIGH));
    }
  }

  if (call.get_custom_fan_mode().has_value()) {
    auto fan_mode = *call.get_custom_fan_mode();
    auto payload = StringToFanLevel(fan_mode);
    if (payload.has_value()) {
      ESP_LOGD(TAG, "Setting fan mode to %s", fan_mode);
      this->set_custom_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(payload.value()));
    }
  }

  if (call.get_swing_mode().has_value()) {
    auto swing_mode = *call.get_swing_mode();
    auto function_value = ClimateSwingModeToInt(swing_mode);
    ESP_LOGD(TAG, "Setting swing mode to %s", climate_swing_mode_to_string(swing_mode));
    this->swing_mode = swing_mode;
    this->sendCmd(ToshibaCommandType::SWING, static_cast<uint8_t>(function_value));
  }

  this->publish_state();
}

ClimateTraits ToshibaClimateUart::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT_COOL, climate::CLIMATE_MODE_COOL,
                              climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_DRY, climate::CLIMATE_MODE_FAN_ONLY});
  if (this->horizontal_swing_) {
    traits.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_VERTICAL,
                                      climate::CLIMATE_SWING_HORIZONTAL, climate::CLIMATE_SWING_BOTH});
  } else {
    traits.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_VERTICAL});
  }
  traits.set_supports_current_temperature(true);

  // Toshiba AC has more FAN levels that standard climate component, we have to use custom.
  traits.add_supported_fan_mode(CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(CLIMATE_FAN_QUIET);
  traits.add_supported_fan_mode(CLIMATE_FAN_LOW);
  traits.add_supported_custom_fan_mode(CUSTOM_FAN_LEVEL_2);
  traits.add_supported_fan_mode(CLIMATE_FAN_MEDIUM);
  traits.add_supported_custom_fan_mode(CUSTOM_FAN_LEVEL_4);
  traits.add_supported_fan_mode(CLIMATE_FAN_HIGH);

  traits.set_visual_temperature_step(1);
  traits.set_visual_min_temperature(this->min_temp_);
  traits.set_visual_max_temperature(MAX_TEMP);

  return traits;
}

// Existing callback methods
void ToshibaClimateUart::on_set_pwr_level(const std::string &value) {
  ESP_LOGD(TAG, "Setting power level to %s", value.c_str());
  auto pwr_level = StringToPwrLevel(value);
  this->sendCmd(ToshibaCommandType::POWER_SEL, static_cast<uint8_t>(pwr_level.value()));
  pwr_select_->publish_state(value);
}

void ToshibaClimateUart::on_set_special_mode(const std::string &value) {
  auto new_special_mode = SpecialModeToInt(value);
  ESP_LOGD(TAG, "Setting special mode to %s", value.c_str());
  this->sendCmd(ToshibaCommandType::SPECIAL_MODE, static_cast<uint8_t>(new_special_mode.value()));
  special_mode_select_->publish_state(value);
  if (new_special_mode != this->special_mode_) {
    if (this->special_mode_ == SPECIAL_MODE::EIGHT_DEG && this->target_temperature < this->min_temp_) {
      // when switching from FrostGuard to Standard mode, set target temperature to default for Standard mode
      this->target_temperature = NORMAL_MODE_DEF_TEMP;
    }
    this->special_mode_ = new_special_mode;
    if (new_special_mode == SPECIAL_MODE::EIGHT_DEG && this->target_temperature >= this->min_temp_) {
      // when switching from Standard to FrostGuard mode, set target temperature to default for FrostGuard mode
      this->target_temperature = SPECIAL_MODE_EIGHT_DEG_DEF_TEMP;
    }
    // update Climate component in HA with new target temperature
    this->publish_state();
  }
}

// New callback methods for air quality
void ToshibaClimateUart::on_set_air_quality_mode(const std::string &value) {
  ESP_LOGD(TAG, "Setting air quality mode to %s", value.c_str());
  auto air_quality_mode = StringToAirQualityMode(value);
  if (air_quality_mode.has_value()) {
    this->sendCmd(ToshibaCommandType::AIR_QUALITY, static_cast<uint8_t>(air_quality_mode.value()));
    air_quality_select_->publish_state(value);
  }
}

// New callback methods for display
void ToshibaClimateUart::on_set_display_brightness(const std::string &value) {
  ESP_LOGD(TAG, "Setting display brightness to %s", value.c_str());
  auto brightness = StringToDisplayBrightness(value);
  if (brightness.has_value()) {
    this->sendCmd(ToshibaCommandType::DISPLAY_BRIGHTNESS, static_cast<uint8_t>(brightness.value()));
    display_brightness_select_->publish_state(value);
  }
}

// New callback methods for dry level
void ToshibaClimateUart::on_set_dry_level(const std::string &value) {
  ESP_LOGD(TAG, "Setting dry level to %s", value.c_str());
  auto dry_level = StringToDryLevel(value);
  if (dry_level.has_value()) {
    this->sendCmd(ToshibaCommandType::DRY_LEVEL, static_cast<uint8_t>(dry_level.value()));
    dry_level_select_->publish_state(value);
  }
}

// New callback methods for switches
void ToshibaClimateUart::on_set_i_feel(bool state) {
  ESP_LOGD(TAG, "Setting I Feel to %s", state ? "ON" : "OFF");
  this->sendCmd(ToshibaCommandType::I_FEEL, state ? 65 : 66);
  i_feel_switch_->publish_state(state);
}

void ToshibaClimateUart::on_set_turbo_mode(bool state) {
  ESP_LOGD(TAG, "Setting Turbo mode to %s", state ? "ON" : "OFF");
  this->sendCmd(ToshibaCommandType::TURBO_MODE, state ? 65 : 66);
  turbo_switch_->publish_state(state);
}

void ToshibaClimateUart::on_set_beep(bool state) {
  ESP_LOGD(TAG, "Setting Beep to %s", state ? "ON" : "OFF");
  this->sendCmd(ToshibaCommandType::BEEP_CONTROL, state ? 65 : 66);
  beep_switch_->publish_state(state);
}

void ToshibaClimateUart::on_set_auto_restart(bool state) {
  ESP_LOGD(TAG, "Setting Auto Restart to %s", state ? "ON" : "OFF");
  this->sendCmd(ToshibaCommandType::AUTO_RESTART, state ? 65 : 66);
  auto_restart_switch_->publish_state(state);
}

// New callback methods for buttons
void ToshibaClimateUart::on_self_clean_press() {
  ESP_LOGI(TAG, "Starting self-cleaning cycle");
  this->sendCmd(ToshibaCommandType::SELF_CLEAN, 65);
}

void ToshibaClimateUart::on_filter_reset_press() {
  ESP_LOGI(TAG, "Resetting filter timer");
  this->sendCmd(ToshibaCommandType::FILTER_RESET, 65);
}

// New callback methods for timers
void ToshibaClimateUart::on_set_on_timer(float hours) {
  ESP_LOGD(TAG, "Setting On Timer to %.1f hours", hours);
  uint8_t timer_value = (uint8_t) hours;
  this->sendCmd(ToshibaCommandType::ON_TIMER, timer_value);
  on_timer_number_->publish_state(hours);
}

void ToshibaClimateUart::on_set_off_timer(float hours) {
  ESP_LOGD(TAG, "Setting Off Timer to %.1f hours", hours);
  uint8_t timer_value = (uint8_t) hours;
  this->sendCmd(ToshibaCommandType::OFF_TIMER, timer_value);
  off_timer_number_->publish_state(hours);
}

void ToshibaClimateUart::on_set_sleep_timer(float hours) {
  ESP_LOGD(TAG, "Setting Sleep Timer to %.1f hours", hours);
  uint8_t timer_value = (uint8_t) hours;
  this->sendCmd(ToshibaCommandType::SLEEP_TIMER, timer_value);
  sleep_timer_number_->publish_state(hours);
}

// Component control methods
void ToshibaPwrModeSelect::control(const std::string &value) { parent_->on_set_pwr_level(value); }
void ToshibaSpecialModeSelect::control(const std::string &value) { parent_->on_set_special_mode(value); }
void ToshibaAirQualitySelect::control(const std::string &value) { parent_->on_set_air_quality_mode(value); }
void ToshibaDisplayBrightnessSelect::control(const std::string &value) { parent_->on_set_display_brightness(value); }
void ToshibaDryLevelSelect::control(const std::string &value) { parent_->on_set_dry_level(value); }

void ToshibaIFeelSwitch::write_state(bool state) { parent_->on_set_i_feel(state); }
void ToshibaTurboSwitch::write_state(bool state) { parent_->on_set_turbo_mode(state); }
void ToshibaBeepSwitch::write_state(bool state) { parent_->on_set_beep(state); }
void ToshibaAutoRestartSwitch::write_state(bool state) { parent_->on_set_auto_restart(state); }

void ToshibaSelfCleanButton::press_action() { parent_->on_self_clean_press(); }
void ToshibaFilterResetButton::press_action() { parent_->on_filter_reset_press(); }

void ToshibaOnTimerNumber::control(float value) { parent_->on_set_on_timer(value); }
void ToshibaOffTimerNumber::control(float value) { parent_->on_set_off_timer(value); }
void ToshibaSleepTimerNumber::control(float value) { parent_->on_set_sleep_timer(value); }

/**
 * Scan all statuses from 128 to 255 in order to find unknown features.
 */
void ToshibaClimateUart::scan() {
  ESP_LOGI(TAG, "Scan started.");
  for (uint8_t i = 128; i < 255; i++) {
    this->requestData(static_cast<ToshibaCommandType>(i));
  }
}

}  // namespace toshiba_suzumi
}  // namespace esphome