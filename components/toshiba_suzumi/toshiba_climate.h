#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "toshiba_climate_mode.h"

namespace esphome {
namespace toshiba_suzumi {

static const char *const TAG = "ToshibaClimateUart";
// default max temp for units
static const uint8_t MAX_TEMP = 30;
// default min temp for units without 8° heating mode
static const uint8_t MIN_TEMP_STANDARD = 17;
static const uint8_t SPECIAL_TEMP_OFFSET = 16;
static const uint8_t SPECIAL_MODE_EIGHT_DEG_MIN_TEMP = 5;
static const uint8_t SPECIAL_MODE_EIGHT_DEG_MAX_TEMP = 13;
static const uint8_t SPECIAL_MODE_EIGHT_DEG_DEF_TEMP = 8;
static const uint8_t NORMAL_MODE_DEF_TEMP = 20;

static const std::vector<uint8_t> HANDSHAKE[6] = {
    {2, 255, 255, 0, 0, 0, 0, 2},       {2, 255, 255, 1, 0, 0, 1, 2, 254}, {2, 0, 0, 0, 0, 0, 2, 2, 2, 250},
    {2, 0, 1, 129, 1, 0, 2, 0, 0, 123}, {2, 0, 1, 2, 0, 0, 2, 0, 0, 254},  {2, 0, 2, 0, 0, 0, 0, 254},
};

static const std::vector<uint8_t> AFTER_HANDSHAKE[2] = {
    {2, 0, 2, 1, 0, 0, 2, 0, 0, 251},
    {2, 0, 2, 2, 0, 0, 2, 0, 0, 250},
};

struct ToshibaCommand {
  ToshibaCommandType cmd;
  std::vector<uint8_t> payload;
  int delay;
};

// Forward declarations for new select classes
class ToshibaPwrModeSelect;
class ToshibaSpecialModeSelect;
class ToshibaAirQualitySelect;
class ToshibaDisplayBrightnessSelect;
class ToshibaDryLevelSelect;

// Forward declarations for new switch classes
class ToshibaIFeelSwitch;
class ToshibaTurboSwitch;
class ToshibaBeepSwitch;
class ToshibaAutoRestartSwitch;

// Forward declarations for new button classes
class ToshibaSelfCleanButton;
class ToshibaFilterResetButton;

// Forward declarations for new number classes
class ToshibaOnTimerNumber;
class ToshibaOffTimerNumber;
class ToshibaSleepTimerNumber;

class ToshibaClimateUart : public PollingComponent, public climate::Climate, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void update() override;
  void scan();
  float get_setup_priority() const override { return setup_priority::LATE; }

  // Existing setters
  void set_outdoor_temp_sensor(sensor::Sensor *outdoor_temp_sensor) { outdoor_temp_sensor_ = outdoor_temp_sensor; }
  void set_pwr_select(select::Select *pws_select) { pwr_select_ = pws_select; }
  void set_horizontal_swing(bool enabled) { horizontal_swing_ = enabled; }
  void disable_wifi_led(bool disabled) { wifi_led_disabled_ = disabled; }
  void set_special_mode_select(select::Select *special_mode_select) { special_mode_select_ = special_mode_select; }
  void set_min_temp(uint8_t min_temp) { min_temp_ = min_temp; }

  // New sensor setters
  void set_filter_time_sensor(sensor::Sensor *filter_time_sensor) { filter_time_sensor_ = filter_time_sensor; }
  void set_operating_hours_sensor(sensor::Sensor *operating_hours_sensor) { operating_hours_sensor_ = operating_hours_sensor; }
  void set_error_code_sensor(text_sensor::TextSensor *error_code_sensor) { error_code_sensor_ = error_code_sensor; }
  void set_timer_status_sensor(text_sensor::TextSensor *timer_status_sensor) { timer_status_sensor_ = timer_status_sensor; }

  // New select setters
  void set_air_quality_select(select::Select *air_quality_select) { air_quality_select_ = air_quality_select; }
  void set_display_brightness_select(select::Select *display_brightness_select) { display_brightness_select_ = display_brightness_select; }
  void set_dry_level_select(select::Select *dry_level_select) { dry_level_select_ = dry_level_select; }

  // New switch setters
  void set_i_feel_switch(switch_::Switch *i_feel_switch) { i_feel_switch_ = i_feel_switch; }
  void set_turbo_switch(switch_::Switch *turbo_switch) { turbo_switch_ = turbo_switch; }
  void set_beep_switch(switch_::Switch *beep_switch) { beep_switch_ = beep_switch; }
  void set_auto_restart_switch(switch_::Switch *auto_restart_switch) { auto_restart_switch_ = auto_restart_switch; }

  // New button setters
  void set_self_clean_button(button::Button *self_clean_button) { self_clean_button_ = self_clean_button; }
  void set_filter_reset_button(button::Button *filter_reset_button) { filter_reset_button_ = filter_reset_button; }

  // New number setters
  void set_on_timer_number(number::Number *on_timer_number) { on_timer_number_ = on_timer_number; }
  void set_off_timer_number(number::Number *off_timer_number) { off_timer_number_ = off_timer_number; }
  void set_sleep_timer_number(number::Number *sleep_timer_number) { sleep_timer_number_ = sleep_timer_number; }

 protected:
  /// Override control to change settings of the climate device.
  void control(const climate::ClimateCall &call) override;

  /// Return the traits of this controller.
  climate::ClimateTraits traits() override;

 private:
  std::vector<uint8_t> rx_message_;
  std::vector<ToshibaCommand> command_queue_;
  uint32_t last_command_timestamp_ = 0;
  uint32_t last_rx_char_timestamp_ = 0;
  STATE power_state_ = STATE::OFF;
  optional<SPECIAL_MODE> special_mode_ = SPECIAL_MODE::STANDARD;
  
  // Existing components
  select::Select *pwr_select_ = nullptr;
  sensor::Sensor *outdoor_temp_sensor_ = nullptr;
  bool horizontal_swing_ = false;
  uint8_t min_temp_ = 17; // default min temp for units without 8° heating mode
  bool wifi_led_disabled_ = false;
  select::Select *special_mode_select_ = nullptr;

  // New sensor components
  sensor::Sensor *filter_time_sensor_ = nullptr;
  sensor::Sensor *operating_hours_sensor_ = nullptr;
  text_sensor::TextSensor *error_code_sensor_ = nullptr;
  text_sensor::TextSensor *timer_status_sensor_ = nullptr;

  // New select components
  select::Select *air_quality_select_ = nullptr;
  select::Select *display_brightness_select_ = nullptr;
  select::Select *dry_level_select_ = nullptr;

  // New switch components
  switch_::Switch *i_feel_switch_ = nullptr;
  switch_::Switch *turbo_switch_ = nullptr;
  switch_::Switch *beep_switch_ = nullptr;
  switch_::Switch *auto_restart_switch_ = nullptr;

  // New button components
  button::Button *self_clean_button_ = nullptr;
  button::Button *filter_reset_button_ = nullptr;

  // New number components
  number::Number *on_timer_number_ = nullptr;
  number::Number *off_timer_number_ = nullptr;
  number::Number *sleep_timer_number_ = nullptr;

  // New state variables
  optional<AIR_QUALITY_MODE> air_quality_mode_ = AIR_QUALITY_MODE::OFF;
  optional<DISPLAY_BRIGHTNESS> display_brightness_ = DISPLAY_BRIGHTNESS::BRIGHT;
  optional<DRY_LEVEL> dry_level_ = DRY_LEVEL::MEDIUM;
  bool i_feel_enabled_ = false;
  bool turbo_mode_enabled_ = false;
  bool beep_enabled_ = true;
  bool auto_restart_enabled_ = false;
  TimerSettings on_timer_ = {0, 0, false};
  TimerSettings off_timer_ = {0, 0, false};
  TimerSettings sleep_timer_ = {0, 0, false};

  // Existing methods
  void enqueue_command_(const ToshibaCommand &command);
  void send_to_uart(const ToshibaCommand command);
  void start_handshake();
  void parseResponse(std::vector<uint8_t> rawData);
  void requestData(ToshibaCommandType cmd);
  void process_command_queue_();
  void sendCmd(ToshibaCommandType cmd, uint8_t value);
  void getInitData();
  void handle_rx_byte_(uint8_t c);
  bool validate_message_();
  void on_set_pwr_level(const std::string &value);
  void on_set_special_mode(const std::string &value);

  // New methods for timer functions
  void on_set_on_timer(float hours);
  void on_set_off_timer(float hours);
  void on_set_sleep_timer(float hours);
  void update_timer_status();

  // New methods for air quality
  void on_set_air_quality_mode(const std::string &value);

  // New methods for display
  void on_set_display_brightness(const std::string &value);

  // New methods for dry level
  void on_set_dry_level(const std::string &value);

  // New methods for switches
  void on_set_i_feel(bool state);
  void on_set_turbo_mode(bool state);
  void on_set_beep(bool state);
  void on_set_auto_restart(bool state);

  // New methods for buttons
  void on_self_clean_press();
  void on_filter_reset_press();

  // Enhanced initialization
  void getExtendedInitData();

  friend class ToshibaPwrModeSelect;
  friend class ToshibaSpecialModeSelect;
  friend class ToshibaAirQualitySelect;
  friend class ToshibaDisplayBrightnessSelect;
  friend class ToshibaDryLevelSelect;
  friend class ToshibaIFeelSwitch;
  friend class ToshibaTurboSwitch;
  friend class ToshibaBeepSwitch;
  friend class ToshibaAutoRestartSwitch;
  friend class ToshibaSelfCleanButton;
  friend class ToshibaFilterResetButton;
  friend class ToshibaOnTimerNumber;
  friend class ToshibaOffTimerNumber;
  friend class ToshibaSleepTimerNumber;
};

// Existing select classes
class ToshibaPwrModeSelect : public select::Select, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void control(const std::string &value) override;
};

class ToshibaSpecialModeSelect : public select::Select, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void control(const std::string &value) override;
};

// New select classes
class ToshibaAirQualitySelect : public select::Select, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void control(const std::string &value) override;
};

class ToshibaDisplayBrightnessSelect : public select::Select, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void control(const std::string &value) override;
};

class ToshibaDryLevelSelect : public select::Select, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void control(const std::string &value) override;
};

// New switch classes
class ToshibaIFeelSwitch : public switch_::Switch, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void write_state(bool state) override;
};

class ToshibaTurboSwitch : public switch_::Switch, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void write_state(bool state) override;
};

class ToshibaBeepSwitch : public switch_::Switch, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void write_state(bool state) override;
};

class ToshibaAutoRestartSwitch : public switch_::Switch, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void write_state(bool state) override;
};

// New button classes
class ToshibaSelfCleanButton : public button::Button, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void press_action() override;
};

class ToshibaFilterResetButton : public button::Button, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void press_action() override;
};

// New number classes
class ToshibaOnTimerNumber : public number::Number, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void control(float value) override;
};

class ToshibaOffTimerNumber : public number::Number, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void control(float value) override;
};

class ToshibaSleepTimerNumber : public number::Number, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void control(float value) override;
};

}  // namespace toshiba_suzumi
}  // namespace esphome