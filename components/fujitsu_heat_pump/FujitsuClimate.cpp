#include "FujitsuClimate.h"

#include "FujiHeatPump.h"

namespace esphome {
namespace fujitsu {

void FujitsuClimate::setup() {
    ESP_LOGD("fuji", "Fuji initialized");
    memcpy(&(this->sharedState), this->heatPump.getCurrentState(),
           sizeof(FujiFrame));
    // c.f. https://github.com/esphome/esphome/blob/acd55b960120265a0a4ce0bd06d08758dce5bbbd/esphome/components/uart/uart_component_esp32_arduino.cpp#L95
    int8_t tx = this->tx_pin_ != nullptr ? this->tx_pin_->get_pin() : -1;
    int8_t rx = this->rx_pin_ != nullptr ? this->rx_pin_->get_pin() : -1;
    this->heatPump.connect(&Serial2, !this->is_master_, rx, tx);
    ESP_LOGD("fuji", "starting task");
    this->heatPump.connect(UART_NUM_2, true);
}

optional<climate::ClimateMode> FujitsuClimate::fujiToEspMode(
    FujiMode fujiMode) {
    if (fujiMode == FujiMode::FAN) {
        return climate::ClimateMode::CLIMATE_MODE_FAN_ONLY;
    }
    if (fujiMode == FujiMode::DRY) {
        return climate::ClimateMode::CLIMATE_MODE_DRY;
    }
    if (fujiMode == FujiMode::COOL) {
        return climate::ClimateMode::CLIMATE_MODE_COOL;
    }
    if (fujiMode == FujiMode::HEAT) {
        return climate::ClimateMode::CLIMATE_MODE_HEAT;
    }
    if (fujiMode == FujiMode::AUTO) {
        return climate::ClimateMode::CLIMATE_MODE_AUTO;
    }
    return {};
}

optional<FujiMode> FujitsuClimate::espToFujiMode(climate::ClimateMode espMode) {
    if (espMode == climate::ClimateMode::CLIMATE_MODE_FAN_ONLY) {
        return FujiMode::FAN;
    }
    if (espMode == climate::ClimateMode::CLIMATE_MODE_DRY) {
        return FujiMode::DRY;
    }
    if (espMode == climate::ClimateMode::CLIMATE_MODE_COOL) {
        return FujiMode::COOL;
    }
    if (espMode == climate::ClimateMode::CLIMATE_MODE_HEAT) {
        return FujiMode::HEAT;
    }
    if (espMode == climate::ClimateMode::CLIMATE_MODE_AUTO) {
        return FujiMode::AUTO;
    }
    return {};
}

optional<climate::ClimateFanMode> FujitsuClimate::fujiToEspFanMode(
    FujiFanMode fujiFanMode) {
    if (fujiFanMode == FujiFanMode::FAN_AUTO) {
        return climate::ClimateFanMode::CLIMATE_FAN_AUTO;
    }

    if (fujiFanMode == FujiFanMode::FAN_HIGH) {
        return climate::ClimateFanMode::CLIMATE_FAN_HIGH;
    }

    if (fujiFanMode == FujiFanMode::FAN_MEDIUM) {
        return climate::ClimateFanMode::CLIMATE_FAN_MEDIUM;
    }

    if (fujiFanMode == FujiFanMode::FAN_LOW) {
        return climate::ClimateFanMode::CLIMATE_FAN_LOW;
    }

    return {};
}

optional<FujiFanMode> FujitsuClimate::espToFujiFanMode(
    climate::ClimateFanMode espFanMode) {
    if (espFanMode == climate::ClimateFanMode::CLIMATE_FAN_AUTO) {
        return FujiFanMode::FAN_AUTO;
    }

    if (espFanMode == climate::ClimateFanMode::CLIMATE_FAN_HIGH) {
        return FujiFanMode::FAN_HIGH;
    }

    if (espFanMode == climate::ClimateFanMode::CLIMATE_FAN_MEDIUM) {
        return FujiFanMode::FAN_MEDIUM;
    }

    if (espFanMode == climate::ClimateFanMode::CLIMATE_FAN_LOW) {
        return FujiFanMode::FAN_LOW;
    }

    return {};
}

void FujitsuClimate::updateState() {
    bool updated = false;
    // Room temp, first checking if we should use a remote temp sensor
    if (this->remote_temperature_ != nullptr && this->remote_temperature_->has_state()) {
        // TODO this probably isn't implemented correctly
        this->current_temperature = this->remote_temperature_->state;
        updated = true;
    } else if (this->current_temperature != this->sharedState.controllerTemp) {
        this->current_temperature = this->sharedState.controllerTemp;
        updated = true;
    }

    // Target temp
    if (this->sharedState.temperature != this->target_temperature) {
        ESP_LOGD("fuji", "ctrl temp %d vs my temp %f",
                 this->sharedState.temperature, this->target_temperature);
        this->target_temperature = this->sharedState.temperature;
        updated = true;
    }

    // Mode
    auto newMode = fujiToEspMode((FujiMode)this->sharedState.acMode);
    if (newMode.has_value() && this->sharedState.onOff &&
        newMode.value() != this->mode) {
        ESP_LOGD("fuji", "ctrl mode %d vs my mode %d", newMode.value(),
                 this->mode);
        this->mode = newMode.value();
        updated = true;
    }

    // Fan speed
    auto newFanMode =
        fujiToEspFanMode((FujiFanMode)this->sharedState.fanMode);
    if (newFanMode.has_value() && newFanMode.value() != this->fan_mode) {
        ESP_LOGD("fujitsu", "ctrl fan mode %d vs my fan mode %d",
                 newFanMode.value(), this->fan_mode.value_or(-1));
        this->fan_mode = newFanMode.value();
        updated = true;
    }

    if (this->sharedState.economyMode &&
        this->preset != climate::ClimatePreset::CLIMATE_PRESET_ECO) {
        ESP_LOGD("fujitsu",
                 "ECO mode turned on by controller, adding preset change "
                 "to call %d ",
                 this->sharedState.economyMode);

        this->preset = climate::ClimatePreset::CLIMATE_PRESET_ECO;
        updated = true;
    } else if (!this->sharedState.economyMode &&
               this->preset == climate::ClimatePreset::CLIMATE_PRESET_ECO) {
        ESP_LOGD("fujitsu",
                 "ECO mode turned off by controller, adding preset change "
                 "to call, %d",
                 this->sharedState.economyMode);

        this->preset = climate::ClimatePreset::CLIMATE_PRESET_NONE;
        updated = true;
    }

    if (!this->sharedState.onOff &&
        this->mode != climate::ClimateMode::CLIMATE_MODE_OFF) {
        ESP_LOGD("fuji",
                 "Controller turned off AC, adding mode change to call");
        this->mode = climate::ClimateMode::CLIMATE_MODE_OFF;
        updated = true;
    }

    if (updated) {
        ESP_LOGD("fuji", "publishing state");
        this->publish_state();
    }
}

void FujitsuClimate::loop() {
    // Atomically recieve the state when it changes
    xQueueReceive(this->heatPump.state_dropbox, &this->sharedState, pdMS_TO_TICKS(100));
    this->updateState();
}

void FujitsuClimate::control(const climate::ClimateCall &call) {
    bool updated = false;
    if (call.get_mode().has_value()) {
        climate::ClimateMode callMode = call.get_mode().value();
        ESP_LOGD("fuji", "Fuji setting mode %d", callMode);

        auto fujiMode = this->espToFujiMode(callMode);

        if (fujiMode.has_value()) {
            this->sharedState.acMode = static_cast<byte>(fujiMode.value());
            if (callMode != climate::ClimateMode::CLIMATE_MODE_OFF) {
                this->sharedState.onOff = 1;
            }
            updated = true;
        }

        if (callMode == climate::ClimateMode::CLIMATE_MODE_OFF) {
            this->sharedState.onOff = 0;
            updated = true;
        }
    }
    if (call.get_target_temperature().has_value()) {
        auto callTargetTemp = call.get_target_temperature().value();
        this->sharedState.temperature = callTargetTemp;
        updated = true;
        ESP_LOGD("fuji", "Fuji setting temperature %f", callTargetTemp);
    }

    if (call.get_preset().has_value()) {
        auto callPreset = call.get_preset().value();
        this->sharedState.economyMode = static_cast<byte>(
            callPreset == climate::ClimatePreset::CLIMATE_PRESET_ECO ? 1
                                                                     : 0);
        updated = true;
        ESP_LOGD("fuji", "Fuji setting preset %d", callPreset);
    }

    if (call.get_fan_mode().has_value()) {
        auto callFanMode = call.get_fan_mode().value();
        auto fujiFanMode = this->espToFujiFanMode(callFanMode);
        if (fujiFanMode.has_value()) {
            this->sharedState.fanMode = static_cast<byte>(fujiFanMode.value());
        }
        updated = true;
        ESP_LOGD("fuji", "Fuji setting fan mode %d", this->fan_mode.value_or(-1));
    }
    if (updated) {
        this->heatPump.setState(&(this->sharedState));
    }
}

climate::ClimateTraits FujitsuClimate::traits() {
    auto traits = climate::ClimateTraits();

    traits.set_supports_current_temperature(true);
    traits.set_supported_modes({
        climate::CLIMATE_MODE_AUTO,
        climate::CLIMATE_MODE_HEAT,
        climate::CLIMATE_MODE_FAN_ONLY,
        climate::CLIMATE_MODE_DRY,
        climate::CLIMATE_MODE_COOL,
        climate::CLIMATE_MODE_OFF,
    });

    traits.set_visual_temperature_step(1);
    traits.set_visual_min_temperature(16);
    traits.set_visual_max_temperature(30);

    traits.set_supported_fan_modes(
        {climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW,
         climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_HIGH});
    traits.set_supported_presets({
        climate::CLIMATE_PRESET_ECO,
        climate::CLIMATE_PRESET_NONE,
    });

    return traits;
}

void FujitsuClimate::dump_config() {
    ESP_LOGCONFIG(TAG, "Fujitsu Climate Heat Pump:");
    ESP_LOGCONFIG(TAG, "  Using uart #2");
    if (this->is_master_) {
        ESP_LOGCONFIG(TAG, "  Running as master");
    } else {
        ESP_LOGCONFIG(TAG, "  Running as secondary");
    }
    LOG_PIN("  TX Pin:", this->tx_pin_);
    LOG_PIN("  RX Pin:", this->rx_pin_);
    //if (this->remote_temperature_ != nullptr) {
    //    LOG_SENSOR("  ", "Remote Temp Sensor", this->remote_temperature_);
    //}
}

}  // namespace fujitsu
}  // namespace esphome
