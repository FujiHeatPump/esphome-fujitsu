#pragma once
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"
#include "esphome/core/preferences.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "FujiHeatPump.h"

namespace esphome {
namespace fujitsu {

class FujitsuClimate : public climate::Climate, public Component {
   public:
    void setup() override;
    void loop() override;
    void late_start() {
        // c.f. https://github.com/esphome/esphome/blob/acd55b960120265a0a4ce0bd06d08758dce5bbbd/esphome/components/uart/uart_component_esp32_arduino.cpp#L95
        int8_t tx = this->tx_pin_ != nullptr ? this->tx_pin_->get_pin() : UART_PIN_NO_CHANGE;
        int8_t rx = this->rx_pin_ != nullptr ? this->rx_pin_->get_pin() : UART_PIN_NO_CHANGE;
        ESP_LOGD("fuji", "starting task");
        this->heatPump.connect(UART_NUM_2, !this->is_master_, rx, tx);
    }
    void control(const climate::ClimateCall &call) override;
    void dump_config() override;
    climate::ClimateTraits traits() override;
    FujiHeatPump heatPump;
    FujiFrame sharedState;

    void set_master(bool is_master) { is_master_ = is_master; }
    void set_tx_pin(InternalGPIOPin *tx_pin) { this->tx_pin_ = tx_pin; }
    void set_rx_pin(InternalGPIOPin *rx_pin) { this->rx_pin_ = rx_pin; }
    void set_remote_temperature(sensor::Sensor *sensor) { this->remote_temperature_ = sensor; }


   protected:
    bool is_master_;
    InternalGPIOPin *tx_pin_;
    InternalGPIOPin *rx_pin_;
    sensor::Sensor *remote_temperature_{nullptr};


    void updateState();
    optional<climate::ClimateMode> fujiToEspMode(FujiMode fujiMode);
    optional<FujiMode> espToFujiMode(climate::ClimateMode espMode);
    
    optional<climate::ClimateFanMode> fujiToEspFanMode(FujiFanMode fujiFanMode);
    optional<FujiFanMode> espToFujiFanMode(climate::ClimateFanMode espFanMode);
};

}  // namespace fujitsu
}  // namespace esphome
