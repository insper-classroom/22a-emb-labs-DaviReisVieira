#include "conf_board.h"
#include <asf.h>

/************************************************************************/
/* BOARD CONFIG                                                         */
/************************************************************************/

#define BUT0_PIO PIOA
#define BUT0_PIO_ID ID_PIOA
#define BUT0_PIO_PIN 11
#define BUT0_PIO_PIN_MASK (1 << BUT0_PIO_PIN)

#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_PIN 28
#define BUT1_PIO_PIN_MASK (1 << BUT1_PIO_PIN)

#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_PIO_IDX 8
#define LED_IDX_MASK (1 << LED_PIO_IDX)

#define USART_COM_ID ID_USART1
#define USART_COM USART1

/************************************************************************/
/* RTOS                                                                */
/************************************************************************/

#define TASK_LED_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_BUT_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_BUT_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/** Queue for msg log send data */
QueueHandle_t xQueueLedFreq;

QueueHandle_t xQueue_Decrement;
QueueHandle_t xQueue_Increment;

/************************************************************************/
/* prototypes local                                                     */
/************************************************************************/

static void USART1_init(void);
void but0_callback(void);
void but1_callback(void);
void pin_toggle(Pio *pio, uint32_t mask);
void configure_pio_input(Pio *pio, const pio_type_t ul_type, const uint32_t ul_mask, const uint32_t ul_attribute, uint32_t ul_id);
void configure_interruption(Pio *pio, uint32_t ul_id, const uint32_t ul_mask, uint32_t ul_attr, void (*p_handler)(uint32_t, uint32_t), uint32_t priority);
void LED_init(int estado);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName)
{
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
   * identify which task has overflowed its stack.
   */
  for (;;)
  {
  }
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void) { pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); }

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void)
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but0_callback(void)
{
  uint32_t delayDecrement = 100;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueue_Decrement, (void *)&delayDecrement, 10);
}

void but1_callback(void)
{
  uint32_t delayIncrement = 100;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueue_Increment, (void *)&delayIncrement, 10);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_led(void *pvParameters)
{

  LED_init(1);

  uint32_t msg = 0;
  uint32_t delayMs = 2000;

  /* tarefas de um RTOS não devem retornar */
  for (;;)
  {
    /* verifica se chegou algum dado na queue, e espera por 0 ticks */
    if (xQueueReceive(xQueueLedFreq, &msg, (TickType_t)0))
    {
      /* chegou novo valor, atualiza delay ! */
      /* aqui eu poderia verificar se msg faz sentido (se esta no range certo)
       */
      /* converte ms -> ticks */
      delayMs = msg / portTICK_PERIOD_MS;
      printf("task_led: %d \n", delayMs);
    }

    /* pisca LED */
    pin_toggle(LED_PIO, LED_IDX_MASK);

    /* suspende por delayMs */
    vTaskDelay(delayMs);
  }
}

static void task_but(void *pvParameters)
{

  configure_pio_input(BUT0_PIO, PIO_INPUT, BUT0_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE, BUT0_PIO_ID);
  configure_interruption(BUT0_PIO, BUT0_PIO_ID, BUT0_PIO_PIN_MASK, PIO_IT_FALL_EDGE, but0_callback, 4);

  configure_pio_input(BUT1_PIO, PIO_INPUT, BUT1_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE, BUT1_PIO_ID);
  configure_interruption(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_PIN_MASK, PIO_IT_FALL_EDGE, but1_callback, 4);

  uint32_t delayTicks = 2000;
  uint32_t delay_Decrement = 0;
  uint32_t delay_Increment = 0;

  for (;;)
  {

    if (xQueueReceive(xQueue_Decrement, (void *)&delay_Decrement, 10))
    {
      delayTicks -= delay_Decrement;
      printf("Task do botão: %d \n", delayTicks);
      if (delayTicks == 100)
      {
        delayTicks = 2000;
      }

      xQueueSend(xQueueLedFreq, (void *)&delayTicks, 10);
    }

    if (xQueueReceive(xQueue_Increment, (void *)&delay_Increment, 10))
    {
      delayTicks += delay_Increment;
      printf("task_but: %d \n", delayTicks);
      xQueueSend(xQueueLedFreq, (void *)&delayTicks, 10);
    }
  }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
  const usart_serial_options_t uart_serial_options = {
      .baudrate = CONF_UART_BAUDRATE,
      .charlength = CONF_UART_CHAR_LENGTH,
      .paritytype = CONF_UART_PARITY,
      .stopbits = CONF_UART_STOP_BITS,
  };

  /* Configure console UART. */
  stdio_serial_init(CONF_UART, &uart_serial_options);

  /* Specify that stdout should not be buffered. */
  setbuf(stdout, NULL);
}

void pin_toggle(Pio *pio, uint32_t mask)
{
  if (pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
  else
    pio_set(pio, mask);
}

void configure_pio_input(Pio *pio, const pio_type_t ul_type, const uint32_t ul_mask, const uint32_t ul_attribute, uint32_t ul_id)
{
  pmc_enable_periph_clk(ul_id);
  pio_configure(pio, ul_type, ul_mask, ul_attribute);
  pio_set_debounce_filter(pio, ul_mask, 60);
}

void configure_interruption(Pio *pio, uint32_t ul_id, const uint32_t ul_mask, uint32_t ul_attr, void (*p_handler)(uint32_t, uint32_t), uint32_t priority)
{
  pio_handler_set(pio, ul_id, ul_mask, ul_attr, p_handler);
  pio_enable_interrupt(pio, ul_mask);
  pio_get_interrupt_status(pio);
  NVIC_EnableIRQ(ul_id);
  NVIC_SetPriority(ul_id, priority);
}

void LED_init(int estado)
{
  pmc_enable_periph_clk(LED_PIO_ID);
  pio_set_output(LED_PIO, LED_IDX_MASK, estado, 0, 0);
};
/************************************************************************/
/* main                                                                 */
/************************************************************************/

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
  /* Initialize the SAM system */
  sysclk_init();
  board_init();
  configure_console();

  /* cria queue com 32 "espacos" */
  /* cada espaço possui o tamanho de um inteiro*/
  xQueueLedFreq = xQueueCreate(32, sizeof(uint32_t));
  if (xQueueLedFreq == NULL)
    printf("falha em criar a queue \n");

  xQueue_Decrement = xQueueCreate(32, sizeof(uint32_t));
  if (xQueue_Decrement == NULL)
    printf("falha em criar queue decrement \n");

  xQueue_Increment = xQueueCreate(32, sizeof(uint32_t));
  if (xQueue_Increment == NULL)
    printf("falha em criar a queue increment \n");

  /* Create task to make led blink */
  if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, NULL) != pdPASS)
  {
    printf("Failed to create test led task\r\n");
  }

  /* Create task to monitor processor activity */
  if (xTaskCreate(task_but, "BUT", TASK_BUT_STACK_SIZE, NULL, TASK_BUT_STACK_PRIORITY, NULL) != pdPASS)
  {
    printf("Failed to create UartTx task\r\n");
  }

  /* Start the scheduler. */
  vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
  while (1)
  {
  }

  /* Will only get here if there was insufficient memory to create the idle
   * task. */
  return 0;
}