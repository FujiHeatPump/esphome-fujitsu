#pragma once
#include "FujiHeatPump.h"
#include "esphome.h"

namespace esphome {
namespace fujitsu {

class FujitsuClimate : public climate::Climate, public Component {
   public:
    void setup() override;
    void loop() override;
    void control(const climate::ClimateCall &call) override;
    climate::ClimateTraits traits() override;
#ifdef USE_ARDUINO
    TaskHandle_t taskHandle;
#endif
    FujiHeatPump heatPump;
    FujiFrame sharedState;
#ifdef USE_ARDUINO
    SemaphoreHandle_t lock;
    bool pendingUpdate;
#endif

   protected:

    void updateState();
    optional<climate::ClimateMode> fujiToEspMode(FujiMode fujiMode);
    optional<FujiMode> espToFujiMode(climate::ClimateMode espMode);
    
    optional<climate::ClimateFanMode> fujiToEspFanMode(FujiFanMode fujiFanMode);
    optional<FujiFanMode> espToFujiFanMode(climate::ClimateFanMode espFanMode);
};

}  // namespace fujitsu
}  // namespace esphome
