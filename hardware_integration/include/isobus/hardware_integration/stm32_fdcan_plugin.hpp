#pragma once

#include <queue>
#include <string>

#include "stm32h7xx_hal.h"
#include "isobus/hardware_integration/can_hardware_plugin.hpp"

class STM32FDCANPlugin : public isobus::CANHardwarePlugin
{
public:

    STM32FDCANPlugin(FDCAN_HandleTypeDef *fdcan);

    std::string get_name() const override;
    bool get_is_valid() const override;

    void open() override;
    void close() override;

    bool read_frame(isobus::CANMessageFrame &frame) override;
    bool write_frame(const isobus::CANMessageFrame &frame) override;

    void poll_rx();

private:

    FDCAN_HandleTypeDef *hfdcan;

    std::queue<isobus::CANMessageFrame> rxQueue;
};