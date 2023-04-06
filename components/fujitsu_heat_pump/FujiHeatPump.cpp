/* This file is based on unreality's FujiHeatPump project */

// #define DEBUG_FUJI
#include "FujiHeatPump.h"
#include "esp_log.h"
#include "string.h"

namespace esphome {
namespace fujitsu {

static const char* TAG = "FujiHeatPump";

FujiFrame FujiHeatPump::decodeFrame() {
    FujiFrame ff;

    ff.messageSource = readBuf[0];
    ff.messageDest = readBuf[1] & 0b01111111;
    ff.messageType = (readBuf[2] & 0b00110000) >> 4;

    ff.acError = (readBuf[kErrorIndex] & kErrorMask) >> kErrorOffset;
    ff.temperature =
        (readBuf[kTemperatureIndex] & kTemperatureMask) >> kTemperatureOffset;
    ff.acMode = (readBuf[kModeIndex] & kModeMask) >> kModeOffset;
    ff.fanMode = (readBuf[kFanIndex] & kFanMask) >> kFanOffset;
    ff.economyMode = (readBuf[kEconomyIndex] & kEconomyMask) >> kEconomyOffset;
    ff.swingMode = (readBuf[kSwingIndex] & kSwingMask) >> kSwingOffset;
    ff.swingStep =
        (readBuf[kSwingStepIndex] & kSwingStepMask) >> kSwingStepOffset;
    ff.controllerPresent =
        (readBuf[kControllerPresentIndex] & kControllerPresentMask) >>
        kControllerPresentOffset;
    ff.updateMagic =
        (readBuf[kUpdateMagicIndex] & kUpdateMagicMask) >> kUpdateMagicOffset;
    ff.onOff = (readBuf[kEnabledIndex] & kEnabledMask) >> kEnabledOffset;
    ff.controllerTemp = (readBuf[kControllerTempIndex] & kControllerTempMask) >>
                        kControllerTempOffset;  // there are 2 leading bits here
                                                // that are unknown

    ff.writeBit = (readBuf[2] & 0b00001000) != 0;
    ff.loginBit = (readBuf[1] & 0b00100000) != 0;
    ff.unknownBit = (readBuf[1] & 0b10000000) > 0;

    return ff;
}

void FujiHeatPump::encodeFrame(FujiFrame ff) {
    memset(writeBuf, 0, 8);

    writeBuf[0] = ff.messageSource;

    writeBuf[1] &= 0b10000000;
    writeBuf[1] |= ff.messageDest & 0b01111111;

    writeBuf[2] &= 0b11001111;
    writeBuf[2] |= ff.messageType << 4;

    if (ff.writeBit) {
        writeBuf[2] |= 0b00001000;
    } else {
        writeBuf[2] &= 0b11110111;
    }

    writeBuf[1] &= 0b01111111;
    if (ff.unknownBit) {
        writeBuf[1] |= 0b10000000;
    }

    if (ff.loginBit) {
        writeBuf[1] |= 0b00100000;
    } else {
        writeBuf[1] &= 0b11011111;
    }

    writeBuf[kModeIndex] =
        (writeBuf[kModeIndex] & ~kModeMask) | (ff.acMode << kModeOffset);
    writeBuf[kModeIndex] = (writeBuf[kEnabledIndex] & ~kEnabledMask) |
                           (ff.onOff << kEnabledOffset);
    writeBuf[kFanIndex] =
        (writeBuf[kFanIndex] & ~kFanMask) | (ff.fanMode << kFanOffset);
    writeBuf[kErrorIndex] =
        (writeBuf[kErrorIndex] & ~kErrorMask) | (ff.acError << kErrorOffset);
    writeBuf[kEconomyIndex] = (writeBuf[kEconomyIndex] & ~kEconomyMask) |
                              (ff.economyMode << kEconomyOffset);
    writeBuf[kTemperatureIndex] =
        (writeBuf[kTemperatureIndex] & ~kTemperatureMask) |
        (ff.temperature << kTemperatureOffset);
    writeBuf[kSwingIndex] =
        (writeBuf[kSwingIndex] & ~kSwingMask) | (ff.swingMode << kSwingOffset);
    writeBuf[kSwingStepIndex] = (writeBuf[kSwingStepIndex] & ~kSwingStepMask) |
                                (ff.swingStep << kSwingStepOffset);
    writeBuf[kControllerPresentIndex] =
        (writeBuf[kControllerPresentIndex] & ~kControllerPresentMask) |
        (ff.controllerPresent << kControllerPresentOffset);
    writeBuf[kUpdateMagicIndex] =
        (writeBuf[kUpdateMagicIndex] & ~kUpdateMagicMask) |
        (ff.updateMagic << kUpdateMagicOffset);
    writeBuf[kControllerTempIndex] =
        (writeBuf[kControllerTempIndex] & ~kControllerTempMask) |
        (ff.controllerTemp << kControllerTempOffset);
}

void heat_pump_uart_event_task(void *pvParameters) {
    FujiHeatPump *heatpump = (FujiHeatPump *)pvParameters;
    uart_event_t event;
    TickType_t wakeTime;
    bool pendingFrame;
    while (true) {
        if(xQueueReceive(heatpump->uart_queue, (void * )&event, pdMS_TO_TICKS(1000))) {
            switch(event.type) {

                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                  other types of events. If we take too much time on data event, the queue might
                  be full.*/
                case UART_DATA:
                    wakeTime = xTaskGetTickCount();
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    if (8 != uart_read_bytes(heatpump->uart_port, heatpump->readBuf, 8, portMAX_DELAY)) {
                        ESP_LOGW(TAG, "Failed to read state update as expected");
                    } else {
                        ESP_LOGD(TAG, "Got heat pump update frame");
                        heatpump->processReceivedFrame(pendingFrame);
                        ESP_LOGD(TAG, "Processed heat pump update frame");
                        xQueueOverwrite(heatpump->state_dropbox, &heatpump->currentState);
                        if (pendingFrame) {
                            ESP_LOGD(TAG, "Now, handling a pending frame txmit");
                            // This causes us to wait until 60 ms have passed since we read wakeTime, so that we account for the processReceivedFrame(). It also allows other tasks to use the core in the meantime because it suspends instead of busy-waiting.
                            vTaskDelayUntil(&wakeTime, pdMS_TO_TICKS(60));
                            if (uart_write_bytes(heatpump->uart_port, (const char*) heatpump->writeBuf, 8) != 8) {
                                ESP_LOGW(TAG, "Failed to write state update as expected");
                            }
                            if (!xSemaphoreTake(heatpump->updateStateMutex, portMAX_DELAY)) {
                                ESP_LOGW(TAG, "Failed to take update state mutex");
                            }
                            heatpump->updateFields = 0;
                            if (!xSemaphoreGive(heatpump->updateStateMutex)) {
                                ESP_LOGW(TAG, "Failed to give update state mutex");
                            }
                        }
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(heatpump->uart_port);
                    xQueueReset(heatpump->uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(heatpump->uart_port);
                    xQueueReset(heatpump->uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
        ESP_LOGI(TAG, "uart task heartbeat");
    }
}

void FujiHeatPump::connect(uart_port_t uart_port, bool secondary, int rxPin, int txPin) {
    ESP_LOGD("FujitsuClimate", "Connect has been entered!");
    int rc;
    uart_config_t uart_config = {
        .baud_rate = 500,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    if (uart_is_driver_installed(uart_port)) {
        ESP_LOGW(TAG, "uninstalling uart driver...");
        rc = uart_driver_delete(uart_port);
        if (rc != 0) {
            ESP_LOGW(TAG, "Failed to uninstall existing uart driver");
            return;
        }
    }
    rc = uart_driver_install(uart_port, 2048, 2048, 20, &this->uart_queue, 0);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to install uart driver");
        return;
    }
    rc = uart_param_config(uart_port, &uart_config);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to configure uart params");
        return;
    }
    rc = uart_set_pin(uart_port, txPin /* TXD */,  rxPin /* RXD */, UART_PIN_NO_CHANGE /* RTS */, UART_PIN_NO_CHANGE /* CTS */);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to set uart pins");
        return;
    }

    rc = uart_set_mode(uart_port, UART_MODE_RS485_HALF_DUPLEX);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to set uart to half duplex");
        return;
    }
    ESP_LOGD(TAG, "Serial port configured");

    if (secondary) {
        controllerIsPrimary = false;
        controllerAddress = static_cast<byte>(FujiAddress::SECONDARY);
        ESP_LOGI(TAG, "Controller in secondary mode");
    } else {
        controllerIsPrimary = true;
        controllerAddress = static_cast<byte>(FujiAddress::PRIMARY);
        ESP_LOGI(TAG, "Controller in primary mode");
    }

    this->uart_port = uart_port;
    //rc = xTaskCreatePinnedToCore(heat_pump_uart_event_task, "FujiTask", 4096, (void *)this,
    //        // TODO is the priority reasonable? find & investigate the freertosconfig.h
    //                        configMAX_PRIORITIES - 1, NULL /* ignore the task handle */, 1);
    rc = xTaskCreate(heat_pump_uart_event_task, "FujiTask", 4096, (void *)this,
                            12, NULL /* ignore the task handle */);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to create heat pump event task");
        return;
    }
}

void FujiHeatPump::printFrame(byte buf[8], FujiFrame ff) {
    ESP_LOGD(TAG, "%X %X %X %X %X %X %X %X  ", buf[0], buf[1], buf[2],
             buf[3], buf[4], buf[5], buf[6], buf[7]);
    ESP_LOGD(
        TAG,
        " mSrc: %d mDst: %d mType: %d write: %d login: %d unknown: %d onOff: "
        "%d temp: %d, mode: %d cP:%d uM:%d cTemp:%d acError:%d \n",
        ff.messageSource, ff.messageDest, ff.messageType, ff.writeBit,
        ff.loginBit, ff.unknownBit, ff.onOff, ff.temperature, ff.acMode,
        ff.controllerPresent, ff.updateMagic, ff.controllerTemp, ff.acError);
}

bool FujiHeatPump::processReceivedFrame(bool& pendingFrame) {
    FujiFrame ff;

    for (int i = 0; i < 8; i++) {
        readBuf[i] ^= 0xFF;
    }

    ff = decodeFrame();

#ifdef DEBUG_FUJI
    ESP_LOGD(TAG, "<-- ");
    printFrame(readBuf, ff);
#endif

    if (ff.messageDest == controllerAddress) {
        lastFrameReceived = xTaskGetTickCount();

        if (ff.messageType == static_cast<byte>(FujiMessageType::STATUS)) {
            if (ff.controllerPresent == 1) {
                // we have logged into the indoor unit
                // this is what most frames are
                ff.messageSource = controllerAddress;

                if (seenSecondaryController) {
                    ff.messageDest =
                        static_cast<byte>(FujiAddress::SECONDARY);
                    ff.loginBit = true;
                    ff.controllerPresent = 0;
                } else {
                    ff.messageDest = static_cast<byte>(FujiAddress::UNIT);
                    ff.loginBit = false;
                    ff.controllerPresent = 1;
                }

                ff.updateMagic = 0;
                ff.unknownBit = true;
                ff.writeBit = 0;
                ff.messageType = static_cast<byte>(FujiMessageType::STATUS);
            } else {
                if (controllerIsPrimary) {
                    // if this is the first message we have received,
                    // announce ourselves to the indoor unit
                    ff.messageSource = controllerAddress;
                    ff.messageDest = static_cast<byte>(FujiAddress::UNIT);
                    ff.loginBit = false;
                    ff.controllerPresent = 0;
                    ff.updateMagic = 0;
                    ff.unknownBit = true;
                    ff.writeBit = 0;
                    ff.messageType =
                        static_cast<byte>(FujiMessageType::LOGIN);

                    ff.onOff = 0;
                    ff.temperature = 0;
                    ff.acMode = 0;
                    ff.fanMode = 0;
                    ff.swingMode = 0;
                    ff.swingStep = 0;
                    ff.acError = 0;
                } else {
                    // secondary controller never seems to get any other
                    // message types, only status with controllerPresent ==
                    // 0 the secondary controller seems to send the same
                    // flags no matter which message type

                    ff.messageSource = controllerAddress;
                    ff.messageDest = static_cast<byte>(FujiAddress::UNIT);
                    ff.loginBit = false;
                    ff.controllerPresent = 1;
                    ff.updateMagic = 2;
                    ff.unknownBit = true;
                    ff.writeBit = 0;
                }
            }

            // if we have any updates, set the flags
            if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
                ESP_LOGW(TAG, "Failed to take update state mutex");
            }

            if (updateFields) {
                ff.writeBit = 1;
            }

            if (updateFields & kOnOffUpdateMask) {
                // updateStateMutex is held
                ff.onOff = updateState.onOff;
            }

            if (updateFields & kTempUpdateMask) {
                // updateStateMutex is held
                ff.temperature = updateState.temperature;
            }

            if (updateFields & kModeUpdateMask) {
                // updateStateMutex is held
                ff.acMode = updateState.acMode;
            }

            if (updateFields & kFanModeUpdateMask) {
                // updateStateMutex is held
                ff.fanMode = updateState.fanMode;
            }

            if (updateFields & kSwingModeUpdateMask) {
                // updateStateMutex is held
                ff.swingMode = updateState.swingMode;
            }

            if (updateFields & kSwingStepUpdateMask) {
                // updateStateMutex is held
                ff.swingStep = updateState.swingStep;
            }

            if (!xSemaphoreGive(updateStateMutex)) {
                ESP_LOGW(TAG, "Failed to give update state mutex");
            }

            memcpy(&currentState, &ff, sizeof(FujiFrame));
        } else if (ff.messageType ==
                   static_cast<byte>(FujiMessageType::LOGIN)) {
            // received a login frame OK frame
            // the primary will send packet to a secondary controller to see
            // if it exists
            ff.messageSource = controllerAddress;
            ff.messageDest = static_cast<byte>(FujiAddress::SECONDARY);
            ff.loginBit = true;
            ff.controllerPresent = 1;
            ff.updateMagic = 0;
            ff.unknownBit = true;
            ff.writeBit = 0;

            ff.onOff = currentState.onOff;
            ff.temperature = currentState.temperature;
            ff.acMode = currentState.acMode;
            ff.fanMode = currentState.fanMode;
            ff.swingMode = currentState.swingMode;
            ff.swingStep = currentState.swingStep;
            ff.acError = currentState.acError;
        } else if (ff.messageType ==
                   static_cast<byte>(FujiMessageType::ERROR)) {
            ESP_LOGD(TAG, "AC ERROR RECV: ");
            printFrame(readBuf, ff);
            // handle errors here
            return false;
        }

        encodeFrame(ff);

#ifdef DEBUG_FUJI
        ESP_LOGD(TAG, "--> ");
        printFrame(writeBuf, ff);
#endif

        for (int i = 0; i < 8; i++) {
            writeBuf[i] ^= 0xFF;
        }

        pendingFrame = true;
    } else if (ff.messageDest ==
               static_cast<byte>(FujiAddress::SECONDARY)) {
        seenSecondaryController = true;
        currentState.controllerTemp =
            ff.controllerTemp;  // we dont have a temp sensor, use the temp
                                // reading from the secondary controller
    }

    return true;
}

bool FujiHeatPump::isBound() {
    if (xTaskGetTickCount() - lastFrameReceived < pdMS_TO_TICKS(1000))
    {
        return true;
    }
    return false;
}

bool FujiHeatPump::updatePending() {
    if (updateFields) {
        return true;
    }
    return false;
}

void FujiHeatPump::setOnOff(bool o) {
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
    updateFields |= kOnOffUpdateMask;
    updateState.onOff = o ? 1 : 0;
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
}
void FujiHeatPump::setTemp(byte t) {
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
    updateFields |= kTempUpdateMask;
    updateState.temperature = t;
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
}
void FujiHeatPump::setMode(byte m) {
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
    updateFields |= kModeUpdateMask;
    updateState.acMode = m;
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
}
void FujiHeatPump::setFanMode(byte fm) {
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
    updateFields |= kFanModeUpdateMask;
    updateState.fanMode = fm;
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
}
void FujiHeatPump::setEconomyMode(byte em) {
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
    updateFields |= kEconomyModeUpdateMask;
    updateState.economyMode = em;
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
}
void FujiHeatPump::setSwingMode(byte sm) {
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
    updateFields |= kSwingModeUpdateMask;
    updateState.swingMode = sm;
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
}
void FujiHeatPump::setSwingStep(byte ss) {
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
    updateFields |= kSwingStepUpdateMask;
    updateState.swingStep = ss;
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
}

bool FujiHeatPump::getOnOff() { return currentState.onOff == 1 ? true : false; }
byte FujiHeatPump::getTemp() { return currentState.temperature; }
byte FujiHeatPump::getMode() { return currentState.acMode; }
byte FujiHeatPump::getFanMode() { return currentState.fanMode; }
byte FujiHeatPump::getEconomyMode() { return currentState.economyMode; }
byte FujiHeatPump::getSwingMode() { return currentState.swingMode; }
byte FujiHeatPump::getSwingStep() { return currentState.swingStep; }
byte FujiHeatPump::getControllerTemp() { return currentState.controllerTemp; }

FujiFrame *FujiHeatPump::getCurrentState() { return &currentState; }

void FujiHeatPump::setState(FujiFrame *state) {
    FujiFrame *current = this->getCurrentState();
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
    if (state->onOff != current->onOff) {
        this->setOnOff(state->onOff);
    }

    if (state->temperature != current->temperature) {
        this->setTemp(state->temperature);
    }

    if (state->acMode != current->acMode) {
        this->setMode(state->acMode);
    }

    if (state->fanMode != current->fanMode) {
        this->setFanMode(state->fanMode);
    }

    if (state->economyMode != current->economyMode) {
        this->setEconomyMode(state->economyMode);
    }

    if (state->swingMode != current->swingMode) {
        this->setSwingMode(state->swingMode);
    }

    if (state->swingStep != current->swingStep) {
        this->setSwingStep(state->swingStep);
    }
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
    ESP_LOGD(TAG, "Successfully set state");
}

byte FujiHeatPump::getUpdateFields() { return updateFields; }

}
}
