#include <string.h>
#include "common_inc.h"
#include "configurations.h"
#include "usart.h"
#include "gpio.h"

extern Motor motor;
extern EncoderCalibrator encoderCalibrator;
extern Led statusLed;
static uint8_t g_tx_buffer[8];


static void UartCAN_Send(uint8_t id, uint8_t *data)
{
    HAL_GPIO_WritePin(RS486_RE_GPIO_Port, RS486_RE_Pin, GPIO_PIN_SET);
    memcpy(g_tx_buffer, data, 7);
    g_tx_buffer[7] = 0x6b;
    HAL_UART_Transmit_DMA(&huart1, g_tx_buffer, 8);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
    HAL_GPIO_WritePin(RS486_RE_GPIO_Port, RS486_RE_Pin, GPIO_PIN_RESET);
}

void OnUartCmd(uint8_t* _data, uint16_t _len)
{
    float tmpF;
    int32_t tmpI;
    uint8_t id = _data[0];
    uint8_t _cmd = _data[1];
    _data = &_data[2];
    uint8_t StdId=0;

    if(boardConfig.canNodeId != id){
        return;
    }
    statusLed.Status(1, true);

    switch (_cmd)
    {
        // 0x00~0x0F No Memory CMDs
        case 0x01:  // Enable Motor
            motor.controller->requestMode = (*(uint32_t*) (_data) == 1) ?
                                            Motor::MODE_COMMAND_VELOCITY : Motor::MODE_STOP;
            break;
        case 0x02:  // Do Calibration
            encoderCalibrator.isTriggered = true;
            break;
        case 0x03:  // Set Current SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_CURRENT)
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
            motor.controller->SetCurrentSetPoint((int32_t) (*(float*) _data * 1000));
            break;
        case 0x04:  // Set Velocity SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_VELOCITY)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_VELOCITY);
            }
            motor.controller->SetVelocitySetPoint(
                (int32_t) (*(float*) _data *
                           (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            break;
        case 0x05:  // Set Position SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            }
            motor.controller->SetPositionSetPoint(
                (int32_t) (*(float*) _data * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            printf("SET MOTOR[0x05] POSITION[]\r\n");
            if (_data[4]) // Need Position & Finished ACK
            {
                tmpF = motor.controller->GetPosition();
                auto* b = (unsigned char*) &tmpF;
                for (int i = 0; i < 4; i++)
                    _data[i] = *(b + i);
                _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
                StdId = (boardConfig.canNodeId );
                UartCAN_Send(StdId, _data);
            }
            break;
        case 0x06:  // Set Position with Time
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            motor.controller->SetPositionSetPointWithTime(
                (int32_t) (*(float*) _data * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
                *(float*) (_data + 4));
            if (_data[4]) // Need Position & Finished ACK
            {
                tmpF = motor.controller->GetPosition();
                auto* b = (unsigned char*) &tmpF;
                for (int i = 0; i < 4; i++)
                    _data[i] = *(b + i);
                _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
                StdId = (boardConfig.canNodeId );
                UartCAN_Send(StdId, _data);
            }
            break;
        case 0x07:  // Set Position with Velocity-Limit
        {
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            }
            motor.config.motionParams.ratedVelocity =
                (int32_t) (*(float*) (_data + 4) * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            motor.controller->SetPositionSetPoint(
                (int32_t) (*(float*) _data * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            // Always Need Position & Finished ACK
            tmpF = motor.controller->GetPosition();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
            StdId = (boardConfig.canNodeId );
            UartCAN_Send(StdId, _data);
        }
            break;

            // 0x10~0x1F CMDs with Memory
        case 0x11:  // Set Node-ID and Store to EEPROM
            boardConfig.canNodeId = *(uint32_t*) (_data);
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x12:  // Set Current-Limit and Store to EEPROM
            motor.config.motionParams.ratedCurrent = (int32_t) (*(float*) _data * 1000);
            boardConfig.currentLimit = motor.config.motionParams.ratedCurrent;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x13:  // Set Velocity-Limit and Store to EEPROM
            motor.config.motionParams.ratedVelocity =
                (int32_t) (*(float*) _data *
                           (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            boardConfig.velocityLimit = motor.config.motionParams.ratedVelocity;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x14:  // Set Acceleration （and Store to EEPROM）
            tmpF = *(float*) _data * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

            motor.config.motionParams.ratedVelocityAcc = (int32_t) tmpF;
            motor.motionPlanner.velocityTracker.SetVelocityAcc((int32_t) tmpF);
            motor.motionPlanner.positionTracker.SetVelocityAcc((int32_t) tmpF);
            boardConfig.velocityAcc = motor.config.motionParams.ratedVelocityAcc;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x15:  // Apply Home-Position and Store to EEPROM
            motor.controller->ApplyPosAsHomeOffset();
            boardConfig.encoderHomeOffset = motor.config.motionParams.encoderHomeOffset %
                                            motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
            boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x16:  // Set Auto-Enable and Store to EEPROM
            boardConfig.enableMotorOnBoot = (*(uint32_t*) (_data) == 1);
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x17:  // Set DCE Kp
            motor.config.ctrlParams.dce.kp = *(int32_t*) (_data);
            boardConfig.dce_kp = motor.config.ctrlParams.dce.kp;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x18:  // Set DCE Kv
            motor.config.ctrlParams.dce.kv = *(int32_t*) (_data);
            boardConfig.dce_kv = motor.config.ctrlParams.dce.kv;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x19:  // Set DCE Ki
            motor.config.ctrlParams.dce.ki = *(int32_t*) (_data);
            boardConfig.dce_ki = motor.config.ctrlParams.dce.ki;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x1A:  // Set DCE Kd
            motor.config.ctrlParams.dce.kd = *(int32_t*) (_data);
            boardConfig.dce_kd = motor.config.ctrlParams.dce.kd;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x1B:  // Set Enable Stall-Protect
            motor.config.ctrlParams.stallProtectSwitch = (*(uint32_t*) (_data) == 1);
            boardConfig.enableStallProtect = motor.config.ctrlParams.stallProtectSwitch;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;


            // 0x20~0x2F Inquiry CMDs
        case 0x21: // Get Current
        {
            tmpF = motor.controller->GetFocCurrent();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = (motor.controller->state == Motor::STATE_FINISH ? 1 : 0);

            StdId = (boardConfig.canNodeId );
            UartCAN_Send(StdId, _data);
        }
            break;
        case 0x22: // Get Velocity
        {
            tmpF = motor.controller->GetVelocity();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = (motor.controller->state == Motor::STATE_FINISH ? 1 : 0);

            StdId = (boardConfig.canNodeId );
            UartCAN_Send(StdId, _data);
        }
            break;
        case 0x23: // Get Position
        {
            tmpF = motor.controller->GetPosition();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            // Finished ACK
            _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
            StdId = (boardConfig.canNodeId );
            UartCAN_Send(StdId, _data);
//            printf("CAN SEND BACK to NODE[%d]\n", boardConfig.canNodeId );
        }
            break;
        case 0x24: // Get Offset
        {
            tmpI = motor.config.motionParams.encoderHomeOffset;
            auto* b = (unsigned char*) &tmpI;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            StdId = (boardConfig.canNodeId );
            UartCAN_Send(StdId, _data);
        }
            break;

        case 0x25: // Get temperature
        {
            auto* b = (unsigned char*) &boardConfig.motor_temperature;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = 0;
            _data[5] = 0;
            _data[6] = 0;
            _data[7] = 0;
            StdId = (boardConfig.canNodeId );
            UartCAN_Send(StdId, _data);
        }
            break;

        case 0x7d:  // enable motor temperature watch
            boardConfig.enableTempWatch = true;
            break;
        case 0x7e:  // Erase Configs
            boardConfig.configStatus = CONFIG_RESTORE;
            break;
        case 0x7f:  // Reboot
            HAL_NVIC_SystemReset();
            break;
        default:
            break;
    }
    statusLed.Status(1, false);
}

