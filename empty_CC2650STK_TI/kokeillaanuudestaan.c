/* C Standard library */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
// For MPU
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"
// For buzzer
#include "Board.h"
#include "buzzer.h"

#define SAMPLESIZE 4
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char buzzerTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

// Prototypes
void buzzerFxn();
char detectCharFromMovm(float *array);

// Global variables
enum state { WAITING=1, BUZZ, UART_SEND};
enum state programState = WAITING;

char detectedChar = 0;

// MPU power pin global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

// MPU power pin
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

char detectCharFromMovm(float *array) {

    static uint8_t reset = 0; // 0 viimeksi tunnistettu reset, 1 viimeksi tunnistettu merrki
    static float xCoordinates[SAMPLESIZE] = {0};
    static float yCoordinates[SAMPLESIZE] = {0};
    static float zCoordinates[SAMPLESIZE] = {0};
    uint8_t i = SAMPLESIZE;
    for (i = SAMPLESIZE; i > 0; i--) {
        if (i == 1) { // Viimeisellä kierroksella asetetaan uusimmat arvot talukoihin.
            xCoordinates[0] = array[0];
            yCoordinates[0] = array[1];
            zCoordinates[0] = array[2];
        } else {
            xCoordinates[i - 1] = xCoordinates[i - 2];
            yCoordinates[i - 1] = yCoordinates[i - 2];
            zCoordinates[i - 1] = zCoordinates[i - 2];
        }
    }

    float xAverage = 0;
    float yAverage = 0;
    float zAverage = 0;

    uint8_t j = 0;
    for (j = 0; j < SAMPLESIZE; j++) {
        xAverage += xCoordinates[j];
        yAverage += yCoordinates[j];
        zAverage += zCoordinates[j];
    }
    xAverage /= SAMPLESIZE;
    yAverage /= SAMPLESIZE;
    zAverage /= SAMPLESIZE;

    // logiikka merkin tunnistamiseen liikkeestä

    // Tunnistetaan asento painovoimalla (akseli antaa noin 1G arvoja)

    if (xAverage > 0.85 && reset == 0) {
        //System_printf(" ");
        /*
        memset(xCoordinates, 0, sizeof(xCoordinates));
        memset(yCoordinates, 0, sizeof(yCoordinates));
        memset(zCoordinates, 0, sizeof(zCoordinates));
        */
        reset = 1;
            return ' ';
    } else if (yAverage > 0.85 && reset == 0) {
        //System_printf(".");
        /*
        memset(xCoordinates, 0, sizeof(xCoordinates));
        memset(yCoordinates, 0, sizeof(yCoordinates));
        memset(zCoordinates, 0, sizeof(zCoordinates));
        */
        reset = 1;
            return '.';
    } else if (yAverage < -0.85 && reset == 0) {
        //System_printf("-");
        /*
        memset(xCoordinates, 0, sizeof(xCoordinates));
        memset(yCoordinates, 0, sizeof(yCoordinates));
        memset(zCoordinates, 0, sizeof(zCoordinates));
        */
        reset = 1;
            return '-';
    } else if (zAverage > 0.85 && reset == 1) {
        //System_printf("Reset\n");
        /*
        memset(xCoordinates, 0, sizeof(xCoordinates));
        memset(yCoordinates, 0, sizeof(yCoordinates));
        memset(zCoordinates, 0, sizeof(zCoordinates));
        */
        reset = 0;
             return 0;
    } else {
        return 0;
    }
}

Void sensorFxn(UArg arg0, UArg arg1) {

    float ax, ay, az, gx, gy, gz;

    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // MPU setup and calibration
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();

    // Loop forever
    while (1) {
        if (programState == WAITING) {
        // MPU ask data
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);

            //store MPU data to an array
            float mpuValues[6] = {ax, ay, az, gx, gy, gz};

            detectedChar = detectCharFromMovm(mpuValues);

            if(detectedChar != 0) {
                char teksti[20];
                sprintf(teksti, "%c", detectedChar);
                System_printf(teksti);
                //System_flush();
                programState = BUZZ;
            }
            // Sleep 100ms
            Task_sleep(50000 / Clock_tickPeriod);
        }
    }

    // Program never gets here..
    // MPU close i2c
    // I2C_close(i2cMPU);
    // MPU power off
    // PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);
}

// Buzzer power pin
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;

// Buzzer power pin
PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

void buzzerFxn() {
    while (1) {
        if (detectedChar == '-' && programState == BUZZ) {
            buzzerOpen(hBuzzer);
            buzzerSetFrequency(800);
            Task_sleep(500000 / Clock_tickPeriod);
            buzzerClose();
            programState = UART_SEND;
        } else if (detectedChar == '.' && programState == BUZZ) {
            buzzerOpen(hBuzzer);
            buzzerSetFrequency(1500);
            Task_sleep(250000 / Clock_tickPeriod);
            buzzerClose();
            programState = UART_SEND;
        } else if (detectedChar == ' ' && programState == BUZZ) {
            buzzerOpen(hBuzzer);
            buzzerSetFrequency(400);
            Task_sleep(250000 / Clock_tickPeriod);
            buzzerClose();
            Task_sleep(250000 / Clock_tickPeriod);
            buzzerOpen(hBuzzer);
            buzzerSetFrequency(400);
            Task_sleep(250000 / Clock_tickPeriod);
            buzzerClose();
            programState = UART_SEND;
        } else {
            Task_sleep(100000 / Clock_tickPeriod);
        }
    }
}

Void uartTaskFxn(UArg arg0, UArg arg1) {
    UART_Handle uart;
    UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.baudRate = 9600; // nopeus 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
         System_abort("Error opening the UART");
    }
    while (1) {
        if (programState == UART_SEND) {
            if (detectedChar != 0) {
                char characterOut[10];
                sprintf(characterOut, "%c\r\n\0", detectedChar);
                UART_write(uart, characterOut, strlen(characterOut));
            }
            programState = WAITING;
        }
        Task_sleep(500000 / Clock_tickPeriod);
    }
}


int main(void) {
    // Sensor task
    Task_Handle sensorTask;
    Task_Params sensorTaskParams;
    //  Buzzer task
    Task_Handle buzzerTask;
    Task_Params buzzerTaskParams;
    // UART task
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    Board_initGeneral();
    Board_initI2C();
    Board_initUART();

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }

    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTask = Task_create((Task_FuncPtr)sensorFxn, &sensorTaskParams, NULL);
    if (sensorTask == NULL) {
        System_abort("Sensor task create failed!");
    }
    // Open Buzzer power pin
    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
      System_abort("Pin open failed!");
    }

    Task_Params_init(&buzzerTaskParams);
    buzzerTaskParams.stackSize = STACKSIZE;
    buzzerTaskParams.stack = &buzzerTaskStack;
    buzzerTaskParams.priority = 2;
    buzzerTask = Task_create((Task_FuncPtr)buzzerFxn, &buzzerTaskParams, NULL);
    if (buzzerTask == NULL) {
        System_abort("Buzzer task create failed");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("UART task create failed!");
    }


    System_printf("Hello World\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
