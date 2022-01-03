#include "FujitsuClimate.h"

#include "FujiHeatPump.h"

namespace esphome {
namespace fujitsu {

void FujitsuClimate::setup() {
    ESP_LOGD("fuji", "Fuji initialized");
    this->_heatPump.connect(&Serial2, true);
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
    climate::ClimateCall call(this);

    if (this->_heatPump.getTemp() != target_temperature) {
        call.set_target_temperature(this->_heatPump.getTemp());
    }

    auto newMode = fujiToEspMode((FujiMode)this->_heatPump.getMode());
    if (newMode != this->mode) {
        call.set_mode(fujiToEspMode((FujiMode)this->_heatPump.getMode()));
    }

    auto newFanMode =
        fujiToEspFanMode((FujiFanMode)this->_heatPump.getFanMode());
    if (newFanMode != this->fan_mode) {
        call.set_fan_mode(newFanMode);
    }

    if (this->_heatPump.getEconomyMode() &&
        this->preset != climate::ClimatePreset::CLIMATE_PRESET_ECO) {
        call.set_preset(climate::ClimatePreset::CLIMATE_PRESET_ECO);
    }

    if (!this->_heatPump.getEconomyMode() &&
        this->preset == climate::ClimatePreset::CLIMATE_PRESET_ECO) {
        call.set_preset(climate::ClimatePreset::CLIMATE_PRESET_NONE);
    }

    call.perform();
}

void FujitsuClimate::loop() {
    if (this->_heatPump.waitForFrame()) {
        delay(60);
        this->_heatPump.sendPendingFrame();
    }

    if (this->current_temperature != this->_heatPump.getControllerTemp()) {
        this->current_temperature = this->_heatPump.getControllerTemp();
        this->publish_state();
    }

    this->updateState();
}

void FujitsuClimate::control(const climate::ClimateCall &call) {
    bool updated = false;

    if (call.get_mode().has_value()) {
        climate::ClimateMode mode = *call.get_mode();

        this->mode = mode;
        ESP_LOGD("fuji", "Fuji setting mode %d", mode);
        auto fujiMode = this->espToFujiMode(this->mode);
        if (fujiMode.has_value()) {
            this->_heatPump.setMode(static_cast<byte>(fujiMode.value()));
        }
        updated = true;
    }
    if (call.get_target_temperature().has_value()) {
        this->target_temperature = call.get_target_temperature().value();
        this->_heatPump.setTemp(this->target_temperature);
        updated = true;
        ESP_LOGD("fuji", "Fuji setting temperature %f",
                 this->target_temperature);
    }

    if (call.get_preset().has_value()) {
        this->preset = call.get_preset().value();
        this->_heatPump.setEconomyMode(
            this->preset.value() == climate::ClimatePreset::CLIMATE_PRESET_ECO);
        updated = true;
        ESP_LOGD("fuji", "Fuji setting preset %d", this->preset);
    }

    if (call.get_fan_mode().has_value()) {
        this->fan_mode = call.get_fan_mode().value();
        auto fujiFanMode = this->espToFujiFanMode(this->fan_mode.value());
        if (fujiFanMode.has_value()) {
            this->_heatPump.setFanMode(static_cast<byte>(fujiFanMode.value()));
        }
        updated = true;
        ESP_LOGD("fuji", "Fuji setting fan mode %d", this->fan_mode);
    }

    if (updated) {
        this->publish_state();
    }
}

climate::ClimateTraits FujitsuClimate::traits() {
    ESP_LOGD("fuji", "Fuji traits called");
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
}  // namespace fujitsu
}  // namespace esphome