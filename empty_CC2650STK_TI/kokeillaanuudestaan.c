/* Eemil Lohi, Lauri Partanen */

/* C standardikirjastot */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* XDCtools tiedostot */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS otsikkotiedostot */
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

/* Laitteen otsikkotiedostot */
#include "Board.h"
/* Kiihtyvyysanturin otsikkotiedosto */
#include "sensors/mpu9250.h"
/* Kaiuttimen otsikkotiedosto */
#include "buzzer.h"

#define SAMPLESIZE 4
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char buzzerCallTaskStack[STACKSIZE];
Char buzzerTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

/* Funktioiden protyypit */
Char detectCharFromMovm(float *array);
Void sensorFxn(UArg arg0, UArg arg1);
Void buzzerCallFxn();
int buzzerFxn(Char inputChar);
Void uartTaskFxn(UArg arg0, UArg arg1);
Void uartReadFxn(UART_Handle handle, void *rxBuf, size_t len);

/* Globaalit muuttujat */
enum state { WAITING=1, BUZZER_WRITE, BUZZER_READ, UART_WRITE, UART_READ};
enum state programState = WAITING;

/* Viimeisin liikeanturilla tunnistettu merkki */
Char detectedChar = 0;

/* Vastaanottopuskuri uartille */
uint8_t uartBuffer[1];

/* Tähän taulukkoon tallentuu uartilla vastaanotetut morse merkit */
char morseBuffer[50] = {0};

/* Tämä muuttuja pitää lukua, montako merkkiä morseBufferissa on */
int morseBufferSize = 0;

/* Kiihtyvyysanturin virtapinnin globaalit muuttujat */
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

/* Kiihtyvyysanturin virtapinni */
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/* Kiihtyvyysanturin I2C rajapinta */
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};


/* Palauttaa tunnistetun merkin, jos annettu data täyttää
 * merkin ehdot, muulloin palauttaa 0 */
Char detectCharFromMovm(float *mpuValuesArray) {

    /* reset vaatii että SensorTag palautetaan neutraaliin
       asentoon ennen seuraavan merkin tunnistamista,
       jos reset arvo on 0 voidaan uusi merkki tunnistaa,
       jos se on 1 täytyy laite käytää reset asennossa*/
    static uint8_t reset = 0;

    static float xCoordinates[SAMPLESIZE] = {0};
    static float yCoordinates[SAMPLESIZE] = {0};
    static float zCoordinates[SAMPLESIZE] = {0};

    /* SAMPLESIZE on glabaali muuttuja, joka määrittää
       kuinka monesta mittaustuloksesta otetaan keskiarvo */
    uint8_t i = SAMPLESIZE;
    for (i = SAMPLESIZE; i > 0; i--) {
        if (i == 1) { // Viimeisellä kierroksella asetetaan uusimmat arvot talukoihin.
            xCoordinates[0] = mpuValuesArray[0];
            yCoordinates[0] = mpuValuesArray[1];
            zCoordinates[0] = mpuValuesArray[2];
        } else { // Siirretään arvoja taulukossa yhdellä oikealle
            xCoordinates[i - 1] = xCoordinates[i - 2];
            yCoordinates[i - 1] = yCoordinates[i - 2];
            zCoordinates[i - 1] = zCoordinates[i - 2];
        }
    }
    /* Tallennetaan akseleiden keskiarvot omiin muuuttujiinsa */
    float xAverage = 0;
    float yAverage = 0;
    float zAverage = 0;

    /* Lasketaan x, y ja z arvojen summat */
    uint8_t j = 0;
    for (j = 0; j < SAMPLESIZE; j++) {
        xAverage += xCoordinates[j];
        yAverage += yCoordinates[j];
        zAverage += zCoordinates[j];
    }
    /* Jaetaan summat SAMPLESIZElla jotta saadaan keskiarvot */
    xAverage /= SAMPLESIZE;
    yAverage /= SAMPLESIZE;
    zAverage /= SAMPLESIZE;

    /* Tunnistetaan asento painovoimalla (akseli antaa noin 1G arvoja)
     * Palautetaan merkki jos vaatimukset täyttyvät */

    if (xAverage > 0.85 && reset == 0) {
        reset = 1;
            return ' ';
    } else if (yAverage > 0.85 && reset == 0) {
        reset = 1;
            return '.';
    } else if (yAverage < -0.85 && reset == 0) {
        reset = 1;
            return '-';
    }
    /* Tämä lauseke täytyy toteutua merkkien tunnistamisien väilillä */
    else if (zAverage > 0.65 && reset == 1) {
        reset = 0;
             return 0;
    } else {
        return 0;
    }
}

Void sensorFxn(UArg arg0, UArg arg1) {


    float ax, ay, az, gx, gy, gz;

    I2C_Handle i2cMPU; // MPU9250 anturille oma I2C rajapinta
    I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    /* Liikeanturin virrat päälle */
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    /* Odotetaan 100ms liikeanturin käynnistymistä */
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    /* Avataan liikeanturille I2C väylä */
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    /* Liikeanturin asetus ja kalibrointi */
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();

    /* Ikuinen silmukka */
    while (1) {
        if (programState == WAITING) {
            /* Kysytään liikeanturilata dataa */
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);

            /* Tallennetaan saatu data taulukkoon */
            float mpuValues[6] = {ax, ay, az, gx, gy, gz};

            /* Kutsutaan merkintunnistusfunktiota ja tallennetaan
             * sen palauttama arvo globaaliin muuttujaan*/
            detectedChar = detectCharFromMovm(mpuValues);

            /* detectCharFromMovm palauttaa aina 0 jos mekkiä ei tunnistettu */
            if(detectedChar != 0) {
                char teksti[20];
                sprintf(teksti, "%c", detectedChar);
                System_printf(teksti);
                //buzzerFxn(detectedChar);
                programState = UART_WRITE;
            }
        }
        /* Nukkumaan 50ms */
        Task_sleep(50000 / Clock_tickPeriod);
    }
}

/* Kaiuttimen virtapinnin globaalit muuttujat */
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;

/* Kaiuttimen virtapinni */
PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

Void buzzerCallFxn() {
    while (1) {
        /* Jos tila on BUZZER_WRITE eli SensorTag lähettää merkin
         * tunnistetaan merkki ja kutsutaan buzzerFxn tällä merkillä.
         * Kun ääni on tuotettu vaihdetaan ohjelma tilaan WAITING*/
        if (programState == BUZZER_WRITE) {
            switch (detectedChar) {
                case '-':
                    if (buzzerFxn(detectedChar) == 0) {
                    }
                    programState = WAITING;
                    break;
                case '.':
                    if (buzzerFxn(detectedChar) == 0) {
                    }
                    programState = WAITING;
                    break;
                case ' ':
                    if (buzzerFxn(detectedChar) == 0) {
                    }
                    programState = WAITING;
                    break;
                default:
                    programState = WAITING;
                    break;
            }
        }
        /* Jos tila on BUZZER_READ käydään for-silmukassa morseBuffer läpi */
        else if (programState == BUZZER_READ) {
            /* Tässä for-silmukassa käydään läpi morseBuffer taulukko,
             * johon on tallennettu uartista tulleet merkit, ja kutsutaan buzzerFxn
             * funktiota joka tuottaa merkkiä vastaavan äänen.*/
            int i = 0;
            for (i = 0; i < sizeof(morseBuffer) / sizeof(morseBuffer[0]); i++) {
                if (morseBuffer[i] == '.'
                   || morseBuffer[i] == '-'
                   || morseBuffer[i] == ' ') {
                    Task_sleep(500000 / Clock_tickPeriod);
                    buzzerFxn(morseBuffer[i]);
                    /* Vastaanotetun viestin tulostus konsoliin */
                    //char teksti[2];
                    //sprintf(teksti, "%c\n", morseBuffer[i]);
                    //System_printf(teksti);
                }
            }
            /* Asetetaan kaikki morseBufferin arvot nolliksi */
            memset(morseBuffer, 0, sizeof(morseBuffer));
            /* Nollataan myös morseBufferSize, koska merkit on postettu */
            morseBufferSize = 0;
            /* Vaihdetaan tilakone tilaan WAITING */
            programState = WAITING;
        }
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

int buzzerFxn(Char inputChar) {
    if (inputChar == '-') {
        /* Tuotetaan ääni joka vastaa merkkiä '-' */
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(800);
        Task_sleep(100000 / Clock_tickPeriod);
        buzzerClose();
        return 0;

    } else if (inputChar == '.') {
        /* Tuotetaan ääni joka vastaa merkkiä '.' */
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(3500);
        Task_sleep(30000 / Clock_tickPeriod);
        buzzerClose();
        return 0;

    } else if (inputChar == ' ') {
        /* Tuotetaan ääni joka vastaa merkkiä ' ' */
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(400);
        Task_sleep(20000 / Clock_tickPeriod);
        buzzerClose();
        Task_sleep(100000 / Clock_tickPeriod);
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(400);
        Task_sleep(20000 / Clock_tickPeriod);
        buzzerClose();
        return 0;
    } else {
        return 1;
    }
}

Void uartTaskFxn(UArg arg0, UArg arg1) {
    UART_Handle uart;
    UART_Params uartParams;

    /* Asetetaan uartin parametrit */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback  = &uartReadFxn;
    uartParams.baudRate = 9600; // nopeus 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
         System_abort("Error opening the UART");
    }
    UART_read(uart, uartBuffer, 1);


    while (1) {
        if (programState == UART_WRITE) {
            if (detectedChar != 0) {
                char characterOut[10];
                sprintf(characterOut, "%c\r\n\0", detectedChar);
                UART_write(uart, characterOut, strlen(characterOut) + 1);
            }
            programState = BUZZER_WRITE;
        }
        Task_sleep(500000 / Clock_tickPeriod);
    }
}
Void uartReadFxn(UART_Handle handle, void *rxBuf, size_t len){
    /* Jos vastaanotetaan jokin merkeistä, jotka haluamme ilmoittaa käyttäjälle
     * asetetaan se morseBufferiin. morseBufferSize muuttuja pitää lukua
     * morseBufferissa olevien merkkien määrästä */
    if (uartBuffer[0] == '-' || uartBuffer[0] == '.' || uartBuffer[0] == ' ') {
        morseBuffer[morseBufferSize] = uartBuffer[0];
        morseBuffer[morseBufferSize + 1] = '\0';
        morseBufferSize++;
    }

    UART_read(handle, rxBuf, 1);
    programState = BUZZER_READ;
}

int main(void) {
    /* Tehtävien muuttujat */

    /* Sensor task */
    Task_Handle sensorTask;
    Task_Params sensorTaskParams;
    /* Buzzer task */
    Task_Handle buzzerTask;
    Task_Params buzzerTaskParams;

    /* Buzzer Caller task */
    Task_Handle buzzerCallTask;
    Task_Params buzzerCallTaskParams;

    /* UART task */
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    /* Alustetaan laite, I2C sekä UART */
    Board_initGeneral();
    Board_initI2C();
    Board_initUART();

    /* Avaa MPU:n virtapinni */
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }
    /* Luodaan sensorTask tehtävä */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTask = Task_create((Task_FuncPtr)sensorFxn, &sensorTaskParams, NULL);
    if (sensorTask == NULL) {
        System_abort("Sensor task create failed!");
    }
    /* Avaa kaiuttimen virtapinni */
    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
      System_abort("Pin open failed!");
    }
    /* Luodaan buzzerTask tehtävä */
    Task_Params_init(&buzzerTaskParams);
    buzzerTaskParams.stackSize = STACKSIZE;
    buzzerTaskParams.stack = &buzzerTaskStack;
    buzzerTaskParams.priority = 2;
    buzzerTask = Task_create((Task_FuncPtr)buzzerFxn, &buzzerTaskParams, NULL);
    if (buzzerTask == NULL) {
        System_abort("Buzzer task create failed");
    }
    /* Luodaan buzzerCallTask tehtävä */
    Task_Params_init(&buzzerCallTaskParams);
    buzzerCallTaskParams.stackSize = STACKSIZE;
    buzzerCallTaskParams.stack = &buzzerCallTaskStack;
    uartTaskParams.priority=2;
    buzzerCallTask = Task_create((Task_FuncPtr)buzzerCallFxn, &buzzerCallTaskParams, NULL);
    if (buzzerCallTask == NULL) {
        System_abort("Buzzer call task create failed");
    }
    /* Luodaan uartTask tehtävä */
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
