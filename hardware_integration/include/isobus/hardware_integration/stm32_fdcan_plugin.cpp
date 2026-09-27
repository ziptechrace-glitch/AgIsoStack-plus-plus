#include "stm32_fdcan_plugin.hpp"

#include <cstring>

volatile uint32_t stm32_can_tx_count = 0;
volatile uint32_t stm32_can_rx_count = 0;
volatile uint32_t stm32_can_tx_last_id = 0;
volatile uint32_t stm32_can_rx_last_id = 0;
volatile uint32_t stm32_can_tx_error = 0;
volatile uint32_t stm32_can_stack_rx_count = 0;

STM32FDCANPlugin::STM32FDCANPlugin(FDCAN_HandleTypeDef *fdcan)
{
    hfdcan = fdcan;
}

std::string STM32FDCANPlugin::get_name() const
{
    return "STM32 FDCAN";
}

bool STM32FDCANPlugin::get_is_valid() const
{
    return (nullptr != hfdcan);
}

void STM32FDCANPlugin::open()
{
    HAL_StatusTypeDef ret = HAL_FDCAN_Start(hfdcan);

    if (ret != HAL_OK)
    {
        volatile uint32_t erro = hfdcan->ErrorCode;
        (void)erro;

        while (1)
        {
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
            HAL_Delay(200);
        }
    }
}

void STM32FDCANPlugin::close()
{
    HAL_FDCAN_Stop(hfdcan);
}

bool STM32FDCANPlugin::write_frame(const isobus::CANMessageFrame &frame)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};

    txHeader.Identifier = frame.identifier;

    txHeader.IdType =
        frame.isExtendedFrame ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;

    txHeader.TxFrameType = FDCAN_DATA_FRAME;

    switch (frame.dataLength)
    {
    case 0:
        txHeader.DataLength = FDCAN_DLC_BYTES_0;
        break;
    case 1:
        txHeader.DataLength = FDCAN_DLC_BYTES_1;
        break;
    case 2:
        txHeader.DataLength = FDCAN_DLC_BYTES_2;
        break;
    case 3:
        txHeader.DataLength = FDCAN_DLC_BYTES_3;
        break;
    case 4:
        txHeader.DataLength = FDCAN_DLC_BYTES_4;
        break;
    case 5:
        txHeader.DataLength = FDCAN_DLC_BYTES_5;
        break;
    case 6:
        txHeader.DataLength = FDCAN_DLC_BYTES_6;
        break;
    case 7:
        txHeader.DataLength = FDCAN_DLC_BYTES_7;
        break;
    default:
        txHeader.DataLength = FDCAN_DLC_BYTES_8;
        break;
    }

    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(
        hfdcan,
        &txHeader,
        const_cast<uint8_t *>(frame.data));
    if (status != HAL_OK)
    {
        printf("FDCAN TX ERRO ID=0x%08lX ERR=0x%08lX\r\n",
               frame.identifier,
               hfdcan->ErrorCode);
    }

    stm32_can_tx_count++;
    stm32_can_tx_last_id = frame.identifier;

    if (status != HAL_OK)
    {
        stm32_can_tx_error++;
    }

    return status == HAL_OK;
}

bool STM32FDCANPlugin::read_frame(isobus::CANMessageFrame &frame)
{
    if (rxQueue.empty())
    {
        return false;
    }

    frame = rxQueue.front();
    rxQueue.pop();
    stm32_can_stack_rx_count++;
    return true;
}

void STM32FDCANPlugin::poll_rx()
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(
                hfdcan,
                FDCAN_RX_FIFO0,
                &rxHeader,
                data) == HAL_OK)
        {
            isobus::CANMessageFrame frame;

            frame.identifier = rxHeader.Identifier;

            stm32_can_rx_count++;
            stm32_can_rx_last_id = frame.identifier;

            frame.isExtendedFrame =
                (rxHeader.IdType == FDCAN_EXTENDED_ID);

            frame.channel = 0;

            frame.timestamp_us =
                HAL_GetTick() * 1000ULL;

            switch (rxHeader.DataLength)
            {
            case FDCAN_DLC_BYTES_0:
                frame.dataLength = 0;
                break;
            case FDCAN_DLC_BYTES_1:
                frame.dataLength = 1;
                break;
            case FDCAN_DLC_BYTES_2:
                frame.dataLength = 2;
                break;
            case FDCAN_DLC_BYTES_3:
                frame.dataLength = 3;
                break;
            case FDCAN_DLC_BYTES_4:
                frame.dataLength = 4;
                break;
            case FDCAN_DLC_BYTES_5:
                frame.dataLength = 5;
                break;
            case FDCAN_DLC_BYTES_6:
                frame.dataLength = 6;
                break;
            case FDCAN_DLC_BYTES_7:
                frame.dataLength = 7;
                break;
            default:
                frame.dataLength = 8;
                break;
            }

            memcpy(frame.data, data, 8);

            rxQueue.push(frame);
        }
    }
}