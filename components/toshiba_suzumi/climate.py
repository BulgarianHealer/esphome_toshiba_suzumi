import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, climate, uart, select, switch, button, number, text_sensor
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_HOUR,
    DEVICE_CLASS_DURATION,
    ICON_TIMER,
    ICON_FAN,
    ICON_LIGHTBULB,
    ICON_RESTART,
    ICON_VOLUME_HIGH,
    ICON_WASHING_MACHINE,
    ICON_FILTER,
    ICON_BRIGHTNESS_5,
    ICON_WATER_PERCENT,
    ICON_SPEEDOMETER,
    ICON_ALERT_CIRCLE,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "select", "switch", "button", "number", "text_sensor"]

# Configuration keys
CONF_ROOM_TEMP = "room_temp"
CONF_OUTDOOR_TEMP = "outdoor_temp"
CONF_PWR_SELECT = "power_select"
CONF_SPECIAL_MODE = "special_mode"
CONF_SPECIAL_MODE_MODES = "modes"

# New configuration keys
CONF_FILTER_TIME = "filter_time"
CONF_OPERATING_HOURS = "operating_hours"
CONF_ERROR_CODE = "error_code"
CONF_TIMER_STATUS = "timer_status"

CONF_AIR_QUALITY_SELECT = "air_quality_select"
CONF_DISPLAY_BRIGHTNESS_SELECT = "display_brightness_select"
CONF_DRY_LEVEL_SELECT = "dry_level_select"

CONF_I_FEEL_SWITCH = "i_feel_switch"
CONF_TURBO_SWITCH = "turbo_switch"
CONF_BEEP_SWITCH = "beep_switch"
CONF_AUTO_RESTART_SWITCH = "auto_restart_switch"

CONF_SELF_CLEAN_BUTTON = "self_clean_button"
CONF_FILTER_RESET_BUTTON = "filter_reset_button"

CONF_ON_TIMER_NUMBER = "on_timer_number"
CONF_OFF_TIMER_NUMBER = "off_timer_number"
CONF_SLEEP_TIMER_NUMBER = "sleep_timer_number"

# Existing configuration keys
FEATURE_HORIZONTAL_SWING = "horizontal_swing"
MIN_TEMP = "min_temp"
DISABLE_WIFI_LED = "disable_wifi_led"

toshiba_ns = cg.esphome_ns.namespace("toshiba_suzumi")
ToshibaClimateUart = toshiba_ns.class_("ToshibaClimateUart", cg.PollingComponent, climate.Climate, uart.UARTDevice)

# Existing classes
ToshibaPwrModeSelect = toshiba_ns.class_('ToshibaPwrModeSelect', select.Select)
ToshibaSpecialModeSelect = toshiba_ns.class_('ToshibaSpecialModeSelect', select.Select)

# New select classes
ToshibaAirQualitySelect = toshiba_ns.class_('ToshibaAirQualitySelect', select.Select)
ToshibaDisplayBrightnessSelect = toshiba_ns.class_('ToshibaDisplayBrightnessSelect', select.Select)
ToshibaDryLevelSelect = toshiba_ns.class_('ToshibaDryLevelSelect', select.Select)

# New switch classes
ToshibaIFeelSwitch = toshiba_ns.class_('ToshibaIFeelSwitch', switch.Switch)
ToshibaTurboSwitch = toshiba_ns.class_('ToshibaTurboSwitch', switch.Switch)
ToshibaBeepSwitch = toshiba_ns.class_('ToshibaBeepSwitch', switch.Switch)
ToshibaAutoRestartSwitch = toshiba_ns.class_('ToshibaAutoRestartSwitch', switch.Switch)

# New button classes
ToshibaSelfCleanButton = toshiba_ns.class_('ToshibaSelfCleanButton', button.Button)
ToshibaFilterResetButton = toshiba_ns.class_('ToshibaFilterResetButton', button.Button)

# New number classes
ToshibaOnTimerNumber = toshiba_ns.class_('ToshibaOnTimerNumber', number.Number)
ToshibaOffTimerNumber = toshiba_ns.class_('ToshibaOffTimerNumber', number.Number)
ToshibaSleepTimerNumber = toshiba_ns.class_('ToshibaSleepTimerNumber', number.Number)

# Version-compatible schema selection
def get_climate_schema():
    """Get climate schema compatible with both old and new ESPHome versions"""
    if hasattr(climate, 'climate_schema'):
        # New way (ESPHome 2025.05+) - requires class parameter
        return climate.climate_schema(climate.Climate)
    else:
        # Old way (ESPHome < 2025.11)
        return climate.CLIMATE_SCHEMA

def get_select_schema():
    """Get select schema compatible with both old and new ESPHome versions"""
    if hasattr(select, 'select_schema'):
        # New way (ESPHome 2025.05+) - requires class parameter
        return select.select_schema(select.Select)
    else:
        # Old way (ESPHome < 2025.11)
        return select.SELECT_SCHEMA

def get_switch_schema():
    """Get switch schema compatible with both old and new ESPHome versions"""
    if hasattr(switch, 'switch_schema'):
        return switch.switch_schema(switch.Switch)
    else:
        return switch.SWITCH_SCHEMA

def get_button_schema():
    """Get button schema compatible with both old and new ESPHome versions"""
    if hasattr(button, 'button_schema'):
        return button.button_schema(button.Button)
    else:
        return button.BUTTON_SCHEMA

def get_number_schema():
    """Get number schema compatible with both old and new ESPHome versions"""
    if hasattr(number, 'number_schema'):
        return number.number_schema(number.Number)
    else:
        return number.NUMBER_SCHEMA

CONFIG_SCHEMA = get_climate_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(ToshibaClimateUart),
        
        # Existing sensor configurations
        cv.Optional(CONF_OUTDOOR_TEMP): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        
        # New sensor configurations
        cv.Optional(CONF_FILTER_TIME): sensor.sensor_schema(
            unit_of_measurement=UNIT_HOUR,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_DURATION,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_FILTER,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_OPERATING_HOURS): sensor.sensor_schema(
            unit_of_measurement=UNIT_HOUR,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_DURATION,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_SPEEDOMETER,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        
        # Text sensors
        cv.Optional(CONF_ERROR_CODE): text_sensor.text_sensor_schema(
            icon=ICON_ALERT_CIRCLE,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_TIMER_STATUS): text_sensor.text_sensor_schema(
            icon=ICON_TIMER,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        
        # Existing select configurations
        cv.Optional(CONF_PWR_SELECT): get_select_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaPwrModeSelect),
        }),
        cv.Optional(CONF_SPECIAL_MODE): get_select_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaSpecialModeSelect),
            cv.Required(CONF_SPECIAL_MODE_MODES): cv.ensure_list(cv.one_of("Standard","Hi POWER","ECO","Fireplace 1","Fireplace 2","8 degrees","Silent#1","Silent#2","Sleep","Floor","Comfort"))
        }),
        
        # New select configurations
        cv.Optional(CONF_AIR_QUALITY_SELECT): get_select_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaAirQualitySelect),
        }),
        cv.Optional(CONF_DISPLAY_BRIGHTNESS_SELECT): get_select_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaDisplayBrightnessSelect),
        }),
        cv.Optional(CONF_DRY_LEVEL_SELECT): get_select_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaDryLevelSelect),
        }),
        
        # New switch configurations
        cv.Optional(CONF_I_FEEL_SWITCH): get_switch_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaIFeelSwitch),
        }),
        cv.Optional(CONF_TURBO_SWITCH): get_switch_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaTurboSwitch),
        }),
        cv.Optional(CONF_BEEP_SWITCH): get_switch_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaBeepSwitch),
        }),
        cv.Optional(CONF_AUTO_RESTART_SWITCH): get_switch_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaAutoRestartSwitch),
        }),
        
        # New button configurations
        cv.Optional(CONF_SELF_CLEAN_BUTTON): get_button_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaSelfCleanButton),
        }),
        cv.Optional(CONF_FILTER_RESET_BUTTON): get_button_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaFilterResetButton),
        }),
        
        # New number configurations
        cv.Optional(CONF_ON_TIMER_NUMBER): get_number_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaOnTimerNumber),
        }),
        cv.Optional(CONF_OFF_TIMER_NUMBER): get_number_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaOffTimerNumber),
        }),
        cv.Optional(CONF_SLEEP_TIMER_NUMBER): get_number_schema().extend({
            cv.GenerateID(): cv.declare_id(ToshibaSleepTimerNumber),
        }),
        
        # Existing feature flags
        cv.Optional(FEATURE_HORIZONTAL_SWING): cv.boolean,
        cv.Optional(DISABLE_WIFI_LED): cv.boolean,
        cv.Optional(MIN_TEMP): cv.int_,
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.polling_component_schema("120s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    await uart.register_uart_device(var, config)

    # Existing sensor configurations
    if CONF_OUTDOOR_TEMP in config:
        conf = config[CONF_OUTDOOR_TEMP]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_temp_sensor(sens))

    # New sensor configurations
    if CONF_FILTER_TIME in config:
        conf = config[CONF_FILTER_TIME]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_filter_time_sensor(sens))

    if CONF_OPERATING_HOURS in config:
        conf = config[CONF_OPERATING_HOURS]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_operating_hours_sensor(sens))

    # Text sensor configurations
    if CONF_ERROR_CODE in config:
        conf = config[CONF_ERROR_CODE]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_error_code_sensor(sens))

    if CONF_TIMER_STATUS in config:
        conf = config[CONF_TIMER_STATUS]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_timer_status_sensor(sens))

    # Existing select configurations
    if CONF_PWR_SELECT in config:
        sel = await select.new_select(config[CONF_PWR_SELECT], options=['50 %', '75 %', '100 %'])
        await cg.register_parented(sel, config[CONF_ID])
        cg.add(var.set_pwr_select(sel))

    if CONF_SPECIAL_MODE in config:
        sel = await select.new_select(config[CONF_SPECIAL_MODE], options=config[CONF_SPECIAL_MODE][CONF_SPECIAL_MODE_MODES])
        if "8 degrees" in config[CONF_SPECIAL_MODE][CONF_SPECIAL_MODE_MODES]:
            # if "8 degrees" feature is in the list, set the min visual temperature to 5
            cg.add(var.set_min_temp(5))
        await cg.register_parented(sel, config[CONF_ID])
        cg.add(var.set_special_mode_select(sel))

    # New select configurations
    if CONF_AIR_QUALITY_SELECT in config:
        sel = await select.new_select(config[CONF_AIR_QUALITY_SELECT], options=['Off', 'Ion', 'Pure', 'Auto'])
        await cg.register_parented(sel, config[CONF_ID])
        cg.add(var.set_air_quality_select(sel))

    if CONF_DISPLAY_BRIGHTNESS_SELECT in config:
        sel = await select.new_select(config[CONF_DISPLAY_BRIGHTNESS_SELECT], options=['Off', 'Dim', 'Bright'])
        await cg.register_parented(sel, config[CONF_ID])
        cg.add(var.set_display_brightness_select(sel))

    if CONF_DRY_LEVEL_SELECT in config:
        sel = await select.new_select(config[CONF_DRY_LEVEL_SELECT], options=['Low', 'Medium', 'High'])
        await cg.register_parented(sel, config[CONF_ID])
        cg.add(var.set_dry_level_select(sel))

    # New switch configurations
    if CONF_I_FEEL_SWITCH in config:
        sw = await switch.new_switch(config[CONF_I_FEEL_SWITCH])
        await cg.register_parented(sw, config[CONF_ID])
        cg.add(var.set_i_feel_switch(sw))

    if CONF_TURBO_SWITCH in config:
        sw = await switch.new_switch(config[CONF_TURBO_SWITCH])
        await cg.register_parented(sw, config[CONF_ID])
        cg.add(var.set_turbo_switch(sw))

    if CONF_BEEP_SWITCH in config:
        sw = await switch.new_switch(config[CONF_BEEP_SWITCH])
        await cg.register_parented(sw, config[CONF_ID])
        cg.add(var.set_beep_switch(sw))

    if CONF_AUTO_RESTART_SWITCH in config:
        sw = await switch.new_switch(config[CONF_AUTO_RESTART_SWITCH])
        await cg.register_parented(sw, config[CONF_ID])
        cg.add(var.set_auto_restart_switch(sw))

    # New button configurations
    if CONF_SELF_CLEAN_BUTTON in config:
        btn = await button.new_button(config[CONF_SELF_CLEAN_BUTTON])
        await cg.register_parented(btn, config[CONF_ID])
        cg.add(var.set_self_clean_button(btn))

    if CONF_FILTER_RESET_BUTTON in config:
        btn = await button.new_button(config[CONF_FILTER_RESET_BUTTON])
        await cg.register_parented(btn, config[CONF_ID])
        cg.add(var.set_filter_reset_button(btn))

    # New number configurations
    if CONF_ON_TIMER_NUMBER in config:
        conf = config[CONF_ON_TIMER_NUMBER]
        num = await number.new_number(conf, min_value=0, max_value=24, step=1)
        await cg.register_parented(num, config[CONF_ID])
        cg.add(var.set_on_timer_number(num))

    if CONF_OFF_TIMER_NUMBER in config:
        conf = config[CONF_OFF_TIMER_NUMBER]
        num = await number.new_number(conf, min_value=0, max_value=24, step=1)
        await cg.register_parented(num, config[CONF_ID])
        cg.add(var.set_off_timer_number(num))

    if CONF_SLEEP_TIMER_NUMBER in config:
        conf = config[CONF_SLEEP_TIMER_NUMBER]
        num = await number.new_number(conf, min_value=0, max_value=7, step=1)
        await cg.register_parented(num, config[CONF_ID])
        cg.add(var.set_sleep_timer_number(num))

    # Existing feature flags
    if FEATURE_HORIZONTAL_SWING in config:
        cg.add(var.set_horizontal_swing(True))

    if MIN_TEMP in config:
        cg.add(var.set_min_temp(config[MIN_TEMP]))

    if DISABLE_WIFI_LED in config:
        cg.add(var.disable_wifi_led(True))