#include <string.h>
#include "common_inc.h"
#include "configurations.h"
#include "Platform/utils.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"

/* ----------------------- Defines ------------------------------------------*/
#define REG_HOLDING_START           0x1000
#define REG_HOLDING_NREGS           128


extern Motor motor;
extern EncoderCalibrator encoderCalibrator;
extern Led statusLed;

/* ----------------------- Static variables ---------------------------------*/
static uint16_t   usRegHoldingBuf[REG_HOLDING_NREGS];


static void float_2u8(float tmpF, uint8_t *_data)
{
    auto *b = (unsigned char *) &tmpF;
    for (int i = 0; i < 4; i++) {
        _data[i] = *(b + i);
    }
}

static void int32_2u8(int32_t tmpI, uint8_t *_data)
{
    auto *b = (unsigned char *) &tmpI;
    for (int i = 0; i < 4; i++) {
        _data[i] = *(b + i);
    }
}

void modbus_init(uint8_t id)
{
    /* Select either ASCII or RTU Mode. */
    ( void )eMBInit( MB_RTU, id, 0, 115200, MB_PAR_NONE );

    /* Initialize the holding register values before starting the
     * Modbus stack
     */
    for (int i = 0; i < REG_HOLDING_NREGS; i++ ) {
        usRegHoldingBuf[i] = ( unsigned short )i;
    }

    /* Enable the Modbus Protocol Stack. */
    ( void )eMBEnable(  );
}

void modbus_poll(void)
{
    /* Call the main polling loop of the Modbus protocol stack. */
    ( void )eMBPoll(  );
}

static void read_reg(uint16_t iRegIndex)
{
#define _DATA (&usRegHoldingBuf[iRegIndex])
    switch (iRegIndex) {
    case 0x01:
        // usRegHoldingBuf[iRegIndex] = motor.controller->requestMode;
        break;
    case 0x02:
        usRegHoldingBuf[iRegIndex] = motor.encoder->angleData.rectifyValid;
        break;
    case 0x03: // Current SetPoint
        float_2u8(motor.controller->GetFocCurrent(), (uint8_t *)_DATA);
        break;
    case 0x05: // Velocity SetPoint
        float_2u8(motor.controller->GetVelocity(), (uint8_t *)_DATA);
        break;
    case 0x07: case 0x09: case 13:// Position SetPoint
        float_2u8(motor.controller->GetPosition(), (uint8_t *)_DATA);
        break;
    case 17: // Node-ID
        usRegHoldingBuf[iRegIndex] = boardConfig.canNodeId;
        break;
    case 18: // Current-Limit
        float_2u8((float)motor.config.motionParams.ratedCurrent / 1000.0f, (uint8_t *)_DATA);
        break;
    case 20: // Velocity-Limit
        float_2u8((float)motor.config.motionParams.ratedVelocity / (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, (uint8_t *)_DATA);
        break;
    case 22: // Acceleration-Limit
        float_2u8((float)motor.config.motionParams.ratedVelocityAcc / (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, (uint8_t *)_DATA);
        break;
    case 24: // Apply Home-Position
        int32_2u8(motor.config.motionParams.encoderHomeOffset, (uint8_t *)_DATA);
        break;
    case 25: // Auto-Enable
        usRegHoldingBuf[iRegIndex] = boardConfig.enableMotorOnBoot;
        break;
    case 26: // DCE Kp
        int32_2u8(motor.config.ctrlParams.dce.kp, (uint8_t *)_DATA);
        break;
    case 28: // DCE Kv
        int32_2u8(motor.config.ctrlParams.dce.kv, (uint8_t *)_DATA);
        break;
    case 30: // DCE Ki
        int32_2u8(motor.config.ctrlParams.dce.ki, (uint8_t *)_DATA);
        break;
    case 32: // DCE Kd
        int32_2u8(motor.config.ctrlParams.dce.kd, (uint8_t *)_DATA);
        break;
    case 34: // Enable Stall-Protect
        usRegHoldingBuf[iRegIndex] = motor.config.ctrlParams.stallProtectSwitch;
        break;
    case 35: // temperature
        float_2u8(boardConfig.motor_temperature, (uint8_t *)_DATA);
        break;
    case 37: // voltage
        float_2u8(boardConfig.motor_voltage, (uint8_t *)_DATA);
        break;

    default:
        break;
    }
#undef _DATA
}

static void write_reg(uint16_t iRegIndex)
{
    float tmpF;
#define _DATA (&usRegHoldingBuf[iRegIndex])
    switch (iRegIndex) {
    case 0x01:
        motor.controller->requestMode = (usRegHoldingBuf[iRegIndex] == 1) ?
                                        Motor::MODE_COMMAND_VELOCITY : Motor::MODE_STOP;
        break;
    case 0x02:
        encoderCalibrator.isTriggered = true;
        break;
    case 0x03: // Current SetPoint
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_CURRENT) {
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
        }
        motor.controller->SetCurrentSetPoint((int32_t) (*(float *) _DATA * 1000));
        break;
    case 0x05: // Velocity SetPoint
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_VELOCITY) {
            motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_VELOCITY);
        }
        motor.controller->SetVelocitySetPoint(
            (int32_t) (*(float *) _DATA *
                       (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        break;
    case 0x07:// Position SetPoint
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION) {
            motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
        }
        motor.controller->SetPositionSetPoint(
            (int32_t) (*(float *) _DATA * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        break;
    case 0x09:// Position SetPoint with time
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION) {
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
        }
        motor.controller->SetPositionSetPointWithTime(
            (int32_t) (*(float *) _DATA * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
            *(float *) (_DATA + 2));
        break;
    case 13:// Position SetPoint with Velocity-Limit
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION) {
            motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
        }
        motor.config.motionParams.ratedVelocity =
            (int32_t) (*(float *) (_DATA + 2) * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
        motor.controller->SetPositionSetPoint(
            (int32_t) (*(float *) _DATA * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        break;


    case 17: // Node-ID
        boardConfig.canNodeId = usRegHoldingBuf[iRegIndex];
        break;
    case 18: // Current-Limit
        motor.config.motionParams.ratedCurrent = (int32_t) (*(float *) _DATA * 1000);
        boardConfig.currentLimit = motor.config.motionParams.ratedCurrent;
        break;
    case 20: // Velocity-Limit
        motor.config.motionParams.ratedVelocity =
            (int32_t) (*(float *) _DATA *
                       (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
        boardConfig.velocityLimit = motor.config.motionParams.ratedVelocity;
        break;
    case 22: // Acceleration-Limit
        tmpF = *(float *) _DATA * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

        motor.config.motionParams.ratedVelocityAcc = (int32_t) tmpF;
        motor.motionPlanner.velocityTracker.SetVelocityAcc((int32_t) tmpF);
        motor.motionPlanner.positionTracker.SetVelocityAcc((int32_t) tmpF);
        boardConfig.velocityAcc = motor.config.motionParams.ratedVelocityAcc;
        break;
    case 24: // Apply Home-Position
        motor.controller->ApplyPosAsHomeOffset();
        boardConfig.encoderHomeOffset = motor.config.motionParams.encoderHomeOffset %
                                        motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 25: // Auto-Enable
        boardConfig.enableMotorOnBoot = (usRegHoldingBuf[iRegIndex] == 1);
        break;
    case 26: // DCE Kp
        motor.config.ctrlParams.dce.kp = *(int32_t *) (_DATA);
        boardConfig.dce_kp = motor.config.ctrlParams.dce.kp;
        break;
    case 28: // DCE Kv
        motor.config.ctrlParams.dce.kv = *(int32_t *) (_DATA);
        boardConfig.dce_kv = motor.config.ctrlParams.dce.kv;
        break;
    case 30: // DCE Ki
        motor.config.ctrlParams.dce.ki = *(int32_t *) (_DATA);
        boardConfig.dce_ki = motor.config.ctrlParams.dce.ki;
        break;
    case 32: // DCE Kd
        motor.config.ctrlParams.dce.kd = *(int32_t *) (_DATA);
        boardConfig.dce_kd = motor.config.ctrlParams.dce.kd;
        break;
    case 34: // Enable Stall-Protect
        motor.config.ctrlParams.stallProtectSwitch = (usRegHoldingBuf[iRegIndex] == 1);
        boardConfig.enableStallProtect = motor.config.ctrlParams.stallProtectSwitch;
        break;
    case 35: // temperature
        boardConfig.motor_temperature_threhold = *(float *) (_DATA);
        break;
    case 37: // voltage
        boardConfig.motor_voltage_threhold = *(float *) (_DATA);
        break;

    case REG_HOLDING_NREGS-3: // config commit
        boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case REG_HOLDING_NREGS-2: // Erase Configs
        boardConfig.configStatus = CONFIG_RESTORE;
        break;
    case REG_HOLDING_NREGS-1: // Reboot
        HAL_NVIC_SystemReset();
        break;

    default:
        break;
    }
#undef _DATA
}

eMBErrorCode
eMBRegHoldingCB( UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    statusLed.Status(1, true);
    if ( ( usAddress >= REG_HOLDING_START ) &&
            ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) ) {
        iRegIndex = ( int )( usAddress - REG_HOLDING_START )-1;
        switch ( eMode ) {

        /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            read_reg(iRegIndex);
            while ( usNRegs > 0 ) {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        /* Update current register values with new values from the
         * protocol stack. */
        case MB_REG_WRITE:
            while ( usNRegs > 0 ) {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            iRegIndex = ( int )( usAddress - REG_HOLDING_START )-1;
            write_reg(iRegIndex);
            break;
        }

    } else {
        eStatus = MB_ENOREG;
    }

    statusLed.Status(1, false);
    return eStatus;
}

