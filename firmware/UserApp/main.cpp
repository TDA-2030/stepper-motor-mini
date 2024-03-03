#include "common_inc.h"
#include "Platform\Memory\eeprom_interface.h"
#include "configurations.h"
#include "Platform/utils.h"
#include "tim.h"
#include "usart.h"

#ifdef GD32F130_150
#include "systick.h"
#endif

/* Component Definitions -----------------------------------------------------*/
BoardConfig_t boardConfig;
Motor motor;
TB67H450 tb67H450;
MT6816 mt6816;
EncoderCalibrator encoderCalibrator(&motor);
Button button1(1, 2000), button2(2, 3000);
void OnButton1Event(Button::Event _event);
void OnButton2Event(Button::Event _event);
Led statusLed;


extern void modbus_init(uint8_t id);
extern void modbus_poll(void);


static void led_show_id(uint16_t id)
{
    for (size_t i = 0; i < 2; i++) {
        statusLed.Status(0, true);
        statusLed.Status(1, true);
        HAL_Delay(80);
        statusLed.Status(0, false);
        statusLed.Status(1, false);
        HAL_Delay(80);
    }
    HAL_Delay(200);

    id = id < 8 ? id : 8;
    for (size_t i = 0; i < id; i++) {
        statusLed.Status(1, true);
        HAL_Delay(150);
        statusLed.Status(1, false);
        HAL_Delay(150);
    }
    statusLed.Status(0, true);
    statusLed.Status(1, true);
    HAL_Delay(400);
    statusLed.Status(0, false);
    statusLed.Status(1, false);
}

/* Main Entry ----------------------------------------------------------------*/
extern "C" void Main()
{
    uint64_t serialNum = GetSerialNumber();
    uint16_t defaultNodeID = 0;

    /*---------- Apply EEPROM Settings ----------*/
    // Setting priority is EEPROM > Motor.h
    EEPROM eeprom;
    eeprom.get(0, boardConfig);
    if (boardConfig.configStatus != CONFIG_OK) { // use default settings
        boardConfig = BoardConfig_t{
            .configStatus = CONFIG_OK,
            .canNodeId = 1,
            .encoderHomeOffset = 0,
            .defaultMode = Motor::MODE_COMMAND_POSITION,
            .currentLimit = 1 * 1000,    // A
            .velocityLimit = 30 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s
            .velocityAcc = 100 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,   // r/s^2
            .calibrationCurrent = 2000,
#ifdef GD32F130_150
            .dce_kp = 250,
            .dce_kv = 80,
            .dce_ki = 100,
            .dce_kd = 500,
#elif defined(STM32F103xB)
            .dce_kp = 200,
            .dce_kv = 80,
            .dce_ki = 300,
            .dce_kd = 250,
#endif
            .motor_temperature = 0.0,
            .enableMotorOnBoot = false,
            .enableStallProtect = false,
            .enableTempWatch = false,
            .reverse_direction = false,
        };
        eeprom.put(0, boardConfig);
    }
    led_show_id(boardConfig.canNodeId);
    // boardConfig.enableTempWatch=false;
    //depends on 3 bits switch now
    // boardConfig.canNodeId = defaultNodeID;
    motor.config.motionParams.encoderHomeOffset = boardConfig.encoderHomeOffset;
    motor.config.motionParams.ratedCurrent = boardConfig.currentLimit;
    motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
    motor.config.motionParams.ratedVelocityAcc = boardConfig.velocityAcc;
    motor.motionPlanner.velocityTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.config.motionParams.caliCurrent = boardConfig.calibrationCurrent;
    motor.config.ctrlParams.dce.kp = boardConfig.dce_kp;
    motor.config.ctrlParams.dce.kv = boardConfig.dce_kv;
    motor.config.ctrlParams.dce.ki = boardConfig.dce_ki;
    motor.config.ctrlParams.dce.kd = boardConfig.dce_kd;
    motor.config.ctrlParams.stallProtectSwitch = boardConfig.enableStallProtect;


//    printf("encoderHomeOffset[%d] ratedCurrent[%d] ratedVelocity[%d] ratedVelocityAcc[%d]\r\n" , boardConfig.encoderHomeOffset, boardConfig.currentLimit, boardConfig.velocityLimit, boardConfig.velocityAcc);
//    printf("velocityTracker[%d] positionTracker[%d] caliCurrent[%d] \r\n" , boardConfig.velocityAcc, boardConfig.velocityAcc);
//    printf("kp[%d] kv[%d] ki[%d] kd[%d]\r\n" , boardConfig.dce_kp, boardConfig.dce_kv, boardConfig.dce_ki, boardConfig.dce_kd);


    /*---------------- Init Motor ----------------*/
    motor.AttachDriver(&tb67H450);
    motor.AttachEncoder(&mt6816);
    motor.controller->Init();
    motor.driver->Init();
    motor.encoder->Init();


    /*------------- Init peripherals -------------*/
    button1.SetOnEventListener(OnButton1Event);
    button2.SetOnEventListener(OnButton2Event);


    /*------- Start Close-Loop Control Tick ------*/
    HAL_Delay(100);
#ifdef GD32F130_150
    timer1_start();  // 100Hz
    timer13_start();  // 20kHz
#elif defined(STM32F103xB)
    HAL_TIM_Base_Start_IT(&htim1);  // 100Hz
    HAL_TIM_Base_Start_IT(&htim4);  // 20kHz
#endif

    if (button1.IsPressed()) {
        encoderCalibrator.isTriggered = true;
    }
//    printf("start motor %d\r\n", defaultNodeID);

    modbus_init(boardConfig.canNodeId);

    for (;;) {
//        if ( encoderCalibrator.FlashRun() == 0)
        encoderCalibrator.TickMainLoop();

        if (boardConfig.configStatus == CONFIG_COMMIT) {
            boardConfig.configStatus = CONFIG_OK;
            eeprom.put(0, boardConfig);
        } else if (boardConfig.configStatus == CONFIG_RESTORE) {
            eeprom.put(0, boardConfig);
            NVIC_SystemReset();
        }
        modbus_poll();
    }
}


/* Event Callbacks -----------------------------------------------------------*/
//let's add a global count in here for quick test
uint32_t count;
extern "C" void Tim1Callback100Hz()
{

    button1.Tick(10);
    button2.Tick(10);
    statusLed.Tick(10, motor.controller->state);
    //1s period to collect a temperature data

    count ++;
    if ( count >= 100) {
        boardConfig.motor_temperature = AdcGetChipTemperature();
        boardConfig.motor_voltage = AdcGetVoltage();
        count = 0;
    }
    if (boardConfig.enableTempWatch) {

    }
}


extern "C" void Tim4Callback20kHz()
{


    if (encoderCalibrator.isTriggered) {
        encoderCalibrator.Tick20kHz();
    } else {
        motor.Tick20kHz();
    }
}


void OnButton1Event(Button::Event _event)
{
    switch (_event) {
    case ButtonBase::UP:
        break;
    case ButtonBase::DOWN:
        break;
    case ButtonBase::LONG_PRESS:
        boardConfig.configStatus = CONFIG_RESTORE;
        break;
    case ButtonBase::CLICK:
        // printf("KEY1\r\n");
        if (motor.controller->modeRunning != Motor::MODE_STOP) {
            boardConfig.defaultMode = motor.controller->modeRunning;
            motor.controller->requestMode = Motor::MODE_STOP;
        } else {
            motor.controller->requestMode = static_cast<Motor::Mode_t>(boardConfig.defaultMode);
        }
        break;
    }
}


void OnButton2Event(Button::Event _event)
{
    switch (_event) {
    case ButtonBase::UP:
        break;
    case ButtonBase::DOWN:
        break;
    case ButtonBase::LONG_PRESS:
        switch (motor.controller->modeRunning) {
        case Motor::MODE_COMMAND_CURRENT:
        case Motor::MODE_PWM_CURRENT:
            motor.controller->SetCurrentSetPoint(0);
            break;
        case Motor::MODE_COMMAND_VELOCITY:
        case Motor::MODE_PWM_VELOCITY:
            motor.controller->SetVelocitySetPoint(0);
            break;
        case Motor::MODE_COMMAND_POSITION:
        case Motor::MODE_PWM_POSITION:
            motor.controller->SetPositionSetPoint(0);
            break;
        case Motor::MODE_COMMAND_Trajectory:
        case Motor::MODE_STEP_DIR:
        case Motor::MODE_STOP:
            break;
        }
        break;
    case ButtonBase::CLICK:
        // printf("KEY2\r\n");
        motor.controller->ClearStallFlag();
        break;
    }
}