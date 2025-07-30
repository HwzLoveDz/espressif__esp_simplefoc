## ESP-IDF SimpleFOC Component For FOC_Pocket 
### how to use
```Cpp
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_simplefoc.h"

BLDCMotor motor = BLDCMotor(14 / 2, 6.5 / 2, 330, (1.2333e-3) / 2);
BLDCDriver6PWM driver = BLDCDriver6PWM(11, 14, 12, 15, 13, 16, 17, 0);
MA600A ma600a = MA600A(SPI2_HOST, GPIO_NUM_21, GPIO_NUM_7, GPIO_NUM_8, GPIO_NUM_9);

float target_angle = 1.57;

void motor_control_task(void *pvParameters) {
    while (1) {
        // main FOC algorithm function
        // the faster you run this function the better
        motor.loopFOC();
        
        // Motion control function
        // velocity, position or voltage (defined in motor.controller)
        // this function can be run at much lower frequency than loopFOC() function
        // You can also use motor.move() and set the motor.target in the code
        motor.move(target_angle);
        
        // get the current angle from the sensor
        float rad_angle = ma600a.getSensorAngle();
        // current_angle = (rad_angle / (2 * PI)) * 360
        printf("current angle: %.2f\n", rad_angle * 180 / PI);

        // 适当延迟以保持控制频率
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

extern "C" void app_main(void) {
    // initialise magnetic sensor hardware
    ma600a.init();
    // link the motor to the sensor
    motor.linkSensor(&ma600a);

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 8.4;
    driver.voltage_limit = 7.4;
    // pwm frequency [Hz]
    // 20kHz is the default value
    driver.pwm_frequency = 50000;
    driver.init();
    // link the motor to the driver
    motor.linkDriver(&driver);

    // choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // set motion control loop to be used
    motor.controller = MotionControlType::angle;

    // contoller configuration
    // default parameters in defaults.h

    // velocity PI controller parameters
    motor.PID_velocity.P = 0.03f;
    motor.PID_velocity.I = 1;
    motor.PID_velocity.D = 0.0001f;
    // maximal voltage to be set to the motor
    motor.voltage_limit = 2.0;

    // velocity low pass filtering time constant
    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.01f;

    // angle P controller
    motor.P_angle.P = 20;
    // maximal velocity of the position control
    motor.velocity_limit = 6.28 * 3;

    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();

    // 创建任务
    xTaskCreatePinnedToCore(
        motor_control_task,
        "motor_control",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL,
        0
    );
    printf("Motor control system started\n");
}
```

