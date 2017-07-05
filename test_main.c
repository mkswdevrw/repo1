/*
 * test_main.c
 *
 *  Created on: 23 Feb 2015
 *      Author: rob
 */


#include "test_main.h"

// emlib header files
#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "gpiointerrupt.h" // gpiointerrupt module: C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2\emdrv\gpiointerrupt

// MK Thism header files
#include "gecko.h"
#include "ledhw.h"
#include "uart.h"
#include "leuart.h"
#include "dprint.h"
#include "str.h"
#include "spi_config.h"
#include "spi.h"
#include "Si446x.h"
#include "radio_config_Si4463.h"
#include "radio.h"
#include "_25LC512.h"
#include "timer.h"
#include "task_util.h"

// freeRTOS header files
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "croutine.h"

#include "map.h"
#include "sap_config.h"
#include "sap.h"
#include "rad.h"
#include "mod.h"
#include "irq.h"

#include "pendant.h"

#include "app_config.h"

#include <stdbool.h>
#include <string.h>



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// **** project wide global variables

// inter-task message queues
xQueueHandle RAD_msgQ;
xQueueHandle MAP_msgQ;
xQueueHandle SAP_msgQ;
xQueueHandle IRQ_msgQ;
xQueueHandle MOD_msgQ;

xTaskHandle RAD_task;
xTaskHandle MAP_task;
xTaskHandle SAP_task;
xTaskHandle IRQ_task;
xTaskHandle MOD_task;
xTaskHandle IDL_task; // initiated at start of MAP_run()

const uint8_t MAP_name[] = "MAP";
const uint8_t SAP_name[] = "SAP";
const uint8_t RAD_name[] = "RAD";
const uint8_t IRQ_name[] = "IRQ";
const uint8_t MOD_name[] = "MOD";
const uint8_t IDL_name[] = "IDL";

//--------------------------------------

// debug message queues
xQueueHandle debugTxQ, debugRxQ;

PendantPkt_t PendantPkt[POOL_CONFIG_PENDANT_PKT_NUM];

// pkt_t MKR_pkt[POOL_CONFIG_MKR_PKT_NUM];

// global pool data & header blocks
//MKR_Headers_t MKR_headers[POOL_CONFIG_MKR_HEADER_NUM_BLOCKS];
//
//MKR_Short_Data_t MKR_shortData[POOL_CONFIG_MKR_SHORT_NUM_BLOCKS];
//MKR_Audio_Data_t MKR_AudioData[POOL_CONFIG_MKR_AUDIO_NUM_BLOCKS];

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// test 1:

void test_1_main()
{
  uint32_t ret;
  volatile uint32_t n;

  // Chip errata
  CHIP_Init();

  // hardware initialisation
  gecko_init();               // initialise cpu clock and sysTick timer
  gecko_enablePeriphClock();  // route main clock through to peripherals
  gecko_enableGPIOclock();    // give GPIO controller clock
  GPIOINT_Init();             // Initialize GPIO interrupt dispatcher

  ledhw_init(); // initialise led pins
  uart1_init(); // initialise UART1 module
  spi0_init(); // initialise SPI0/USART0 module
  timer_init(TIMER_NUM_0); // initialise timer 0
  leuart0_init(); // (debug uart)

  // ----------------------------------

  // task initialisation
  pool_init();

  // ----------------------------------
  debugTxQ = xQueueCreate(SAP_CONFIG_DEBUGQ_TX_LEN, sizeof(char));
  if(debugTxQ == 0)
    for(;;);

  debugRxQ = xQueueCreate(SAP_CONFIG_DEBUGQ_RX_LEN, sizeof(char));
  if(debugRxQ == 0)
    for(;;);

  // ----------------------------------

  // **** create inter-task message queues

  SAP_msgQ = xQueueCreate(APP_CONFIG_MSGQ_LEN_SAP, sizeof(msg_t));
  if(SAP_msgQ == 0)
    for(;;);

  MAP_msgQ = xQueueCreate(APP_CONFIG_MSGQ_LEN_MAP, sizeof(msg_t));
  if(MAP_msgQ == 0)
    for(;;);

  RAD_msgQ = xQueueCreate(APP_CONFIG_MSGQ_LEN_RAD, sizeof(msg_t));
  if(RAD_msgQ == 0)
    for(;;);

  IRQ_msgQ = xQueueCreate(APP_CONFIG_MSGQ_LEN_IRQ, sizeof(msg_t));
  if(IRQ_msgQ == 0)
    for(;;);

  MOD_msgQ = xQueueCreate(APP_CONFIG_MSGQ_LEN_MOD, sizeof(msg_t));
  if(MOD_msgQ == 0)
    for(;;);

  // ----------------------------------

  // task creation
  ret = xTaskCreate(sap_run, (const signed char *) SAP_name, TASK_STD_STACK_SIZE, NULL, TASK_PRIORITY_1, &SAP_task);
  if(ret != pdPASS)
    for(;;);

  ret = xTaskCreate(MAP_run, (const signed char *) MAP_name, TASK_STD_STACK_SIZE, NULL, TASK_PRIORITY_1, &MAP_task);
  if(ret != pdPASS)
    for(;;);

  ret = xTaskCreate(RAD_run, (const signed char *) RAD_name, TASK_STD_STACK_SIZE, NULL, TASK_PRIORITY_1, &RAD_task);
  if(ret != pdPASS)
    for(;;);

  ret = xTaskCreate(MOD_run, (const signed char *) MOD_name, TASK_STD_STACK_SIZE, NULL, TASK_PRIORITY_1, &MOD_task);
  if(ret != pdPASS)
    for(;;);

  ret = xTaskCreate(IRQ_run, (const signed char *) IRQ_name, TASK_STD_STACK_SIZE, NULL, TASK_PRIORITY_2, &IRQ_task);
  if(ret != pdPASS)
    for(;;);

// ----------------------------------

  timer_enable(TIMER_NUM_0);
  printStr("\n\r--------\n\r", 12);
  printStr("start\n\r", 7);

  // Start FreeRTOS Scheduler
  vTaskStartScheduler();

}


  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
