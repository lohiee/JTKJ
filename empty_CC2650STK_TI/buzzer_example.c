/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

/* Virtapainikkeen otsikkotiedostot */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* Board Header files */
#include "Board.h"
#include "buzzer.h" // remember to add buzzer.h and buzzer.c into project directory!

/* Task */
#define STACKSIZE 2048
Char taskStack[STACKSIZE];

static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;

// Pinnien alustukset, molemmille pinneille oma konfiguraatio
// Vakio BOARD_BUTTON_0 vastaa toista painonappia
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

// Vakio Board_LED0 vastaa toista lediä
PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    // JTKJ: Exercise 1. Blink either led of the device

       uint_t pinValue = PIN_getOutputValue( Board_LED0 );
       pinValue = !pinValue;
       PIN_setOutputValue( ledHandle, Board_LED0, pinValue );
}

static PIN_Handle powerButtonHandle;
static PIN_State powerButtonState;

PIN_Config powerButtonConfig[] = {
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};
PIN_Config powerButtonWakeConfig[] = {
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
   PIN_TERMINATE
};

Void powerFxn(PIN_Handle handle, PIN_Id pinId) {
    System_printf("Sammutetaan");
    System_flush();
   // Näyttö pois päältä
   // Display_clear(displayHandle);
   // Display_close(displayHandle);

   // Odotetana hetki ihan varalta..
   Task_sleep(100 * 1000 / Clock_tickPeriod);

   // Taikamenot
   PIN_close(powerButtonHandle);

   PINCC26XX_setWakeup(powerButtonWakeConfig);
   System_printf("Virtapainike asetettu");
   System_flush();
   Power_shutdown(NULL,0);
}

static PIN_Handle hBuzzer;
static PIN_State sBuzzer;
PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

Void taskFxn(UArg arg0, UArg arg1) {

  while (1) {
    buzzerOpen(hBuzzer);
    buzzerSetFrequency(1000);
    Task_sleep(500000 / Clock_tickPeriod);
    buzzerClose();

    Task_sleep(950000 / Clock_tickPeriod);
  }

}

Int main(void) {

  Task_Handle task;
  Task_Params taskParams;

  // Initialize board
  Board_initGeneral();

  // Buzzer
  hBuzzer = PIN_open(&sBuzzer, cBuzzer);
  if (hBuzzer == NULL) {
    System_abort("Pin open failed!");
  }

  Task_Params_init(&taskParams);
  taskParams.stackSize = STACKSIZE;
  taskParams.stack = &taskStack;
  task = Task_create((Task_FuncPtr)taskFxn, &taskParams, NULL);
  if (task == NULL) {
    System_abort("Task create failed!");
  }

  /* Sanity check */
  System_printf("Beeps!\n");
  System_flush();

  // Power
  powerButtonHandle = PIN_open(&powerButtonState, powerButtonConfig);
  if(!powerButtonHandle) {
     System_abort("Error initializing power button\n");
  }
  if (PIN_registerIntCb(powerButtonHandle, &powerFxn) != 0) {
     System_abort("Error registering power button callback");
  }

  // Ledi käyttöön ohjelmassa
  ledHandle = PIN_open( &ledState, ledConfig );
  if(!ledHandle) {
     System_abort("Error initializing LED pin\n");
  }

  // Painonappi käyttöön ohjelmassa
  buttonHandle = PIN_open(&buttonState, buttonConfig);
  if(!buttonHandle) {
     System_abort("Error initializing button pin\n");
  }

  // Painonapille keskeytyksen käsittellijä
  if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
     System_abort("Error registering button callback function");
  }

  /* Start BIOS */
  BIOS_start();

  return (0);
}
