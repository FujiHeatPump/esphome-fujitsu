/* This file is based on unreality's FujiHeatPump project */

// #define DEBUG_FUJI
#include "FujiHeatPump.h"

#define TAG "FujiHeatPump"

// The esphome ESP_LOGx macros expand to reference esp_log_printf_, but do so
// without using its namespace. https://github.com/esphome/issues/issues/3196
// The workaround is to pull that particular function into this namespace.
using esphome::esp_log_printf_;

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

#ifdef USE_ARDUINO
void FujiHeatPump::connect(HardwareSerial *serial, bool secondary) {
    return this->connect(serial, secondary, -1, -1);
}

void FujiHeatPump::connect(HardwareSerial *serial, bool secondary,
                           int rxPin = -1, int txPin = -1) {
    _serial = serial;
    if (rxPin != -1 && txPin != -1) {
#ifdef ESP32
        _serial->begin(500, SERIAL_8E1, rxPin, txPin);
#else
        Serial.print("Setting RX/TX pin unsupported, using defaults.\n");
        _serial->begin(500, SERIAL_8E1);
#endif
    } else {
        _serial->begin(500, SERIAL_8E1);
    }
    _serial->setTimeout(200);

    if (secondary) {
        controllerIsPrimary = false;
        controllerAddress = static_cast<byte>(FujiAddress::SECONDARY);
    } else {
        controllerIsPrimary = true;
        controllerAddress = static_cast<byte>(FujiAddress::PRIMARY);
    }

    lastFrameReceived = 0;
}
#else
void heat_pump_uart_event_task(void *pvParameters) {
    FujiHeatPump *heatpump = (FujiHeatPump *)pvParameters;
    uart_event_t event;
    size_t buffered_size;
    TickType_t wakeTime;
    bool pendingFrame;
    while (true) {
        if(xQueueReceive(heatpump->uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
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
                        heatpump->processReceivedFrame(pendingFrame);
                        xQueueOverwrite(heatpump->state_dropbox, &heatpump->currentState);
                        if (pendingFrame) {
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
                            // TODO I think we should be using RS485_HALF_DUPLEX mode instead: https://github.com/espressif/esp-idf/blob/81e1e6599553a646a689ad51e32a5d48b34cfec5/examples/peripherals/uart/uart_echo_rs485/main/rs485_example.c
                            // read back our own frame so we dont process it again
                            uart_read_bytes(heatpump->uart_port, heatpump->writeBuf, 8, portMAX_DELAY);
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
    }
}

void FujiHeatPump::connect(uart_port_t uart_port, bool secondary, int rxPin, int txPin) {
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

    if (secondary) {
        controllerIsPrimary = false;
        controllerAddress = static_cast<byte>(FujiAddress::SECONDARY);
    } else {
        controllerIsPrimary = true;
        controllerAddress = static_cast<byte>(FujiAddress::PRIMARY);
    }

    this->updateStateMutex = xSemaphoreCreateRecursiveMutex();
    state_dropbox = xQueueCreate(1, sizeof(FujiFrame));
    this->uart_port = uart_port;
    rc = xTaskCreatePinnedToCore(heat_pump_uart_event_task, "FujiTask", 4096, (void *)this,
            // TODO is the priority reasonable? find & investigate the freertosconfig.h
                            configMAX_PRIORITIES - 1, NULL /* ignore the task handle */, 1);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to create heat pump event task");
        return;
    }
}
#endif

void FujiHeatPump::printFrame(byte buf[8], FujiFrame ff) {
    ESP_LOGD("fuji", "%X %X %X %X %X %X %X %X  ", buf[0], buf[1], buf[2],
             buf[3], buf[4], buf[5], buf[6], buf[7]);
    ESP_LOGD(
        "fuji",
        " mSrc: %d mDst: %d mType: %d write: %d login: %d unknown: %d onOff: "
        "%d temp: %d, mode: %d cP:%d uM:%d cTemp:%d acError:%d \n",
        ff.messageSource, ff.messageDest, ff.messageType, ff.writeBit,
        ff.loginBit, ff.unknownBit, ff.onOff, ff.temperature, ff.acMode,
        ff.controllerPresent, ff.updateMagic, ff.controllerTemp, ff.acError);
}

#ifdef USE_ARDUINO
void FujiHeatPump::sendPendingFrame() {
    if (pendingFrame && (millis() - lastFrameReceived) > 50) {
        _serial->write(writeBuf, 8);
        _serial->flush();
        pendingFrame = false;
        updateFields = 0;

        _serial->readBytes(
            writeBuf,
            8);  // read back our own frame so we dont process it again
    }
}

bool FujiHeatPump::waitForFrame() {
    if (_serial->available()) {
        memset(readBuf, 0, 8);
        int bytesRead = _serial->readBytes(readBuf, 8);

        if (bytesRead < 8) {
            // skip incomplete frame
            return false;
        }

        return processReceivedFrame(pendingFrame);
    }
    return false;
}
#endif

bool FujiHeatPump::processReceivedFrame(bool& pendingFrame) {
    FujiFrame ff;

    for (int i = 0; i < 8; i++) {
        readBuf[i] ^= 0xFF;
    }

    ff = decodeFrame();

#ifdef DEBUG_FUJI
    ESP_LOGD("fuji", "<-- ");
    printFrame(readBuf, ff);
#endif

    if (ff.messageDest == controllerAddress) {
#ifdef USE_ARDUINO
        lastFrameReceived = millis();
#else
        lastFrameReceived = xTaskGetTickCount();
#endif

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
#ifdef USE_ESP_IDF
            if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
                ESP_LOGW(TAG, "Failed to take update state mutex");
            }
#endif

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

#ifdef USE_ESP_IDF
            if (!xSemaphoreGive(updateStateMutex)) {
                ESP_LOGW(TAG, "Failed to give update state mutex");
            }
#endif

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
            ESP_LOGD("fuji", "AC ERROR RECV: ");
            printFrame(readBuf, ff);
            // handle errors here
            return false;
        }

        encodeFrame(ff);

#ifdef DEBUG_FUJI
        ESP_LOGD("fuji", "--> ");
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
#ifdef USE_ARDUINO
    if (millis() - lastFrameReceived < 1000)
#else
    if (xTaskGetTickCount() - lastFrameReceived < pdMS_TO_TICKS(1000))
#endif
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
#ifdef USE_ESP_IDF
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
#endif
    updateFields |= kOnOffUpdateMask;
    updateState.onOff = o ? 1 : 0;
#ifdef USE_ESP_IDF
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
#endif
}
void FujiHeatPump::setTemp(byte t) {
#ifdef USE_ESP_IDF
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
#endif
    updateFields |= kTempUpdateMask;
    updateState.temperature = t;
#ifdef USE_ESP_IDF
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
#endif
}
void FujiHeatPump::setMode(byte m) {
#ifdef USE_ESP_IDF
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
#endif
    updateFields |= kModeUpdateMask;
    updateState.acMode = m;
#ifdef USE_ESP_IDF
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
#endif
}
void FujiHeatPump::setFanMode(byte fm) {
#ifdef USE_ESP_IDF
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
#endif
    updateFields |= kFanModeUpdateMask;
    updateState.fanMode = fm;
#ifdef USE_ESP_IDF
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
#endif
}
void FujiHeatPump::setEconomyMode(byte em) {
#ifdef USE_ESP_IDF
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
#endif
    updateFields |= kEconomyModeUpdateMask;
    updateState.economyMode = em;
#ifdef USE_ESP_IDF
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
#endif
}
void FujiHeatPump::setSwingMode(byte sm) {
#ifdef USE_ESP_IDF
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
#endif
    updateFields |= kSwingModeUpdateMask;
    updateState.swingMode = sm;
#ifdef USE_ESP_IDF
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
#endif
}
void FujiHeatPump::setSwingStep(byte ss) {
#ifdef USE_ESP_IDF
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
#endif
    updateFields |= kSwingStepUpdateMask;
    updateState.swingStep = ss;
#ifdef USE_ESP_IDF
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
#endif
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
#ifdef USE_ESP_IDF
    if (!xSemaphoreTake(updateStateMutex, portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to take update state mutex");
    }
#endif
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
#ifdef USE_ESP_IDF
    if (!xSemaphoreGive(updateStateMutex)) {
        ESP_LOGW(TAG, "Failed to give update state mutex");
    }
#endif
}

byte FujiHeatPump::getUpdateFields() { return updateFields; }
