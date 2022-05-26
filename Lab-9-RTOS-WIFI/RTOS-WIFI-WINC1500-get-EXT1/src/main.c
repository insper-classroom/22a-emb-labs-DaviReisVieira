#include "asf.h"
#include "main.h"
#include <string.h>
#include "bsp/include/nm_bsp.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "util.h"

/************************************************************************/
/* WIFI                                                                 */
/************************************************************************/

/** IP address of host. */
uint32_t gu32HostIp = 0;

/** TCP client socket handlers. */
static SOCKET tcp_client_socket = -1;

/** Receive buffer definition. */
static uint8_t g_receivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};
static uint8_t g_sendBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};

/** Wi-Fi status variable. */
static bool gbConnectedWifi = false;

/** Get host IP status variable. */
/** Wi-Fi connection state */
static uint8_t wifi_connected;

/** Instance of HTTP client module. */
static bool gbHostIpByName = false;

/** TCP Connection status variable. */
static bool gbTcpConnection = false;
static bool gbTcpConnected = false;

/** Server host name. */
static char server_host_name[] = MAIN_SERVER_NAME;

// LED 0
#define LED0_PIO PIOC                         // periferico que controla o LED
#define LED0_PIO_ID ID_PIOC                   // ID do periférico PIOC (controla LED)
#define LED0_PIO_IDX 8                        // ID do LED no PIO
#define LED0_PIO_IDX_MASK (1 << LED0_PIO_IDX) // Mascara para CONTROLARMOS o LED

#define BUT_0_PIO PIOA
#define BUT_0_PIO_ID ID_PIOA
#define BUT_0_IDX 11
#define BUT_0_IDX_MASK (1u << BUT_0_IDX)

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_WIFI_STACK_SIZE (6 * 4096 / sizeof(portSTACK_TYPE))
#define TASK_WIFI_PRIORITY (1)
#define TASK_PROCESS_STACK_SIZE (4 * 4096 / sizeof(portSTACK_TYPE))
#define TASK_PROCESS_PRIORITY (0)

SemaphoreHandle_t xSemaphore;
QueueHandle_t xQueueMsg;

SemaphoreHandle_t xSemaphoreBut;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

TaskHandle_t xHandleWifi = NULL;

/************************************************************************/
/* HOOKs                                                                */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName)
{
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  for (;;)
  {
  }
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void)
{
  configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void send_get_request(char *endpoint)
{
  sprintf((char *)g_sendBuffer, MAIN_PREFIX_BUFFER, endpoint);
  send(tcp_client_socket, g_sendBuffer, strlen((char *)g_sendBuffer), 0);
}

void send_post_request(char *endpoint, char status)
{
  char *buffer[10];
  sprintf(buffer, "LED=%c\n", status);
  sprintf((char *)g_sendBuffer, MAIN_POST_PREFIX, endpoint, strlen(buffer), buffer);
  printf(g_sendBuffer);
  send(tcp_client_socket, g_sendBuffer, strlen((char *)g_sendBuffer), 0);
}
/************************************************************************/
/* callbacks                                                            */
/************************************************************************/

/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * \return None.
 */
static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
  gu32HostIp = hostIp;
  gbHostIpByName = true;
  printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", hostName,
         (int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
         (int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
}

/**
 * \brief Callback function of TCP client socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg Type of Socket notification
 * \param[in] pvMsg A structure contains notification informations.
 *
 * \return None.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
  /* Check for socket event on TCP socket. */
  if (sock == tcp_client_socket)
  {

    switch (u8Msg)
    {
    case SOCKET_MSG_CONNECT:
    {
      printf("socket_msg_connect\n");
      if (gbTcpConnection)
      {
        tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
        if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR)
        {
          printf("socket_cb: connect ok \n");
          gbTcpConnected = true;
        }
        else
        {
          printf("socket_cb: connect error!\r\n");
          gbTcpConnection = false;
          gbTcpConnected = false;
          close(tcp_client_socket);
          tcp_client_socket = -1;
        }
      }
    }
    break;

    case SOCKET_MSG_RECV:
    {
      char *pcIndxPtr;
      char *pcEndPtr;

      tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
      if (pstrRecv && pstrRecv->s16BufferSize > 0)
      {
        xQueueSend(xQueueMsg, &pstrRecv, 10);
        xSemaphoreGive(xSemaphore);
      }
      else
      {
        // printf("socket_cb: recv error!\r\n");
        close(tcp_client_socket);
        tcp_client_socket = -1;
      }
    }
    break;

    default:
      break;
    }
  }
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType Type of Wi-Fi notification.
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters.
 *
 * \return None.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
  switch (u8MsgType)
  {
  case M2M_WIFI_RESP_CON_STATE_CHANGED:
  {
    tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
    if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED)
    {
      printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
      m2m_wifi_request_dhcp_client();
    }
    else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED)
    {
      printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
      gbConnectedWifi = false;
      wifi_connected = 0;
    }

    break;
  }

  case M2M_WIFI_REQ_DHCP_CONF:
  {
    uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
    printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
           pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
    wifi_connected = M2M_WIFI_CONNECTED;
    break;
  }

  case M2M_WIFI_RESP_GET_SYS_TIME:
    /*Initial first callback will be provided by the WINC itself on the first communication with NTP */
    {
      tstrSystemTime *strSysTime_now = (tstrSystemTime *)pvMsg;

      /* Print the hour, minute and second.
       * GMT is the time at Greenwich Meridian.
       */
      printf("socket_cb: Year: %d, Month: %d, The GMT time is %u:%02u:%02u\r\n",
             strSysTime_now->u16Year,
             strSysTime_now->u8Month,
             strSysTime_now->u8Hour,    /* hour (86400 equals secs per day) */
             strSysTime_now->u8Minute,  /* minute (3600 equals secs per minute) */
             strSysTime_now->u8Second); /* second */
      break;
    }

  default:
  {
    break;
  }
  }
}

void but0_callback(void *pvParameters)
{
  printf("\nBUTTON 0 clicked\n");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphoreBut, &xHigherPriorityTaskWoken);
}

void led_on(void)
{
  pio_clear(LED0_PIO, LED0_PIO_IDX_MASK);
}

void led_off(void)
{
  pio_set(LED0_PIO, LED0_PIO_IDX_MASK);
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_process(void *pvParameters)
{

  printf("task process created \n");
  vTaskDelay(1000);

  char status = 0;
  char request_status = 0;

  uint msg_counter = 0;
  tstrSocketRecvMsg *p_recvMsg;

  enum states
  {
    WAIT = 0,
    GET,
    POST,
    ACK,
    ACK_POST,
    MSG,
    TIMEOUT,
    DONE,
  };

  enum states state = WAIT;

  while (1)
  {

    switch (state)
    {
    case WAIT:
      // aguarda task_wifi conectar no wifi e socket estar pronto
      printf("STATE: WAIT \n");
      while (gbTcpConnection == false && tcp_client_socket >= 0)
      {
        vTaskDelay(10);
      }
      if (xSemaphoreTake(xSemaphoreBut, 100))
      {
        state = POST;
      }
      else
      {
        state = GET;
      }
      break;

    case GET:
      printf("STATE: GET \n");

      send_get_request("/status");

      state = ACK;
      break;

    case POST:
      printf("STATE: POST \n");
      printf("\n\n STATUS_ATUAL: %c\n\n", status);
      request_status = status == '1' ? '0' : '1';
      printf("\n\n REQUEST_STATUS: %c\n\n", request_status);
      send_post_request("/status", request_status);
      state = ACK_POST;
      break;

    case ACK:
      printf("STATE: ACK \n");
      memset(g_receivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
      recv(tcp_client_socket, &g_receivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);

      if (xQueueReceive(xQueueMsg, &p_recvMsg, 5000) == pdTRUE)
      {
        printf(STRING_LINE);
        printf(p_recvMsg->pu8Buffer);
        printf(STRING_EOL);
        printf(STRING_LINE);
        state = MSG;
      }
      else
      {
        state = TIMEOUT;
      };
      break;

    case ACK_POST:
      printf("STATE: ACK_POST \n");

      memset(g_receivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
      recv(tcp_client_socket, &g_receivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);

      if (xQueueReceive(xQueueMsg, &p_recvMsg, 5000) == pdTRUE)
      {
        printf(STRING_LINE);
        printf(p_recvMsg->pu8Buffer);
        printf(STRING_EOL);
        printf(STRING_LINE);
        state = DONE;
      }
      else
      {
        state = TIMEOUT;
      };
      break;
    case MSG:
      printf("STATE: MSG \n");
      memset(g_receivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
      recv(tcp_client_socket, &g_receivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);

      if (xQueueReceive(xQueueMsg, &p_recvMsg, 5000) == pdTRUE)
      {
        printf(STRING_LINE);
        printf(p_recvMsg->pu8Buffer);

        if (strstr((char *)p_recvMsg->pu8Buffer, "led") != NULL)
        {
          if (strstr((char *)p_recvMsg->pu8Buffer, "1") != NULL)
          {
            printf("led on \n");
            led_on();
          }
          else
          {
            printf("led off \n");
            led_off();
          }
        }

        printf(STRING_EOL);
        printf(STRING_LINE);
        state = DONE;
      }
      else
      {
        state = TIMEOUT;
      };
      break;

    case DONE:
      printf("STATE: DONE \n");

      state = WAIT;
      break;

    case TIMEOUT:
      state = WAIT;
      break;

    default:
      state = WAIT;
    }
  }
}

static void task_wifi(void *pvParameters)
{
  tstrWifiInitParam param;
  struct sockaddr_in addr_in;

  xSemaphore = xSemaphoreCreateCounting(20, 0);
  xQueueMsg = xQueueCreate(10, sizeof(tstrSocketRecvMsg));

  /* Initialize the BSP. */
  nm_bsp_init();

  /* Initialize Wi-Fi parameters structure. */
  memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

  /* Initialize Wi-Fi driver with data and status callbacks. */
  param.pfAppWifiCb = wifi_cb;
  int8_t ret = m2m_wifi_init(&param);
  if (M2M_SUCCESS != ret)
  {
    printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
    while (1)
    {
    }
  }

  /* Initialize socket module. */
  socketInit();

  /* Register socket callback function. */
  registerSocketCallback(socket_cb, resolve_cb);

  /* Connect to router. */
  printf("main: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);
  m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

  /* formata ip */
  addr_in.sin_family = AF_INET;
  addr_in.sin_port = _htons(MAIN_SERVER_PORT);
  inet_aton(MAIN_SERVER_NAME, &addr_in.sin_addr);

  printf(STRING_LINE);

  while (1)
  {
    vTaskDelay(50);
    m2m_wifi_handle_events(NULL);

    if (wifi_connected == M2M_WIFI_CONNECTED)
    {
      /* Open client socket. */
      if (tcp_client_socket < 0)
      {
        printf(STRING_LINE);
        printf("socket init \n");
        if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
          printf("main: failed to create TCP client socket error!\r\n");
          continue;
        }

        /* Connect server */
        printf("socket connecting\n");
        if (connect(tcp_client_socket, (struct sockaddr *)&addr_in,
                    sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR)
        {
          close(tcp_client_socket);
          tcp_client_socket = -1;
          printf("main: error connect to socket\n");
        }
        else
        {
          gbTcpConnection = true;
        }
      }
    }
  }
}

int main(void)
{
  /* Initialize the board. */
  sysclk_init();
  board_init();

  pmc_enable_periph_clk(LED0_PIO_ID);
  pmc_enable_periph_clk(BUT_0_PIO);

  pio_configure(LED0_PIO, PIO_OUTPUT_1, LED0_PIO_IDX_MASK, PIO_DEFAULT);
  pio_configure(BUT_0_PIO, PIO_INPUT, BUT_0_IDX_MASK,
                PIO_PULLUP | PIO_DEBOUNCE);

  pio_handler_set(BUT_0_PIO, BUT_0_PIO_ID, BUT_0_IDX_MASK, PIO_IT_EDGE,
                  but0_callback);
  pio_enable_interrupt(BUT_0_PIO, BUT_0_IDX_MASK);
  pio_get_interrupt_status(BUT_0_PIO);
  NVIC_EnableIRQ(BUT_0_PIO_ID);
  NVIC_SetPriority(BUT_0_PIO_ID, 4);

  /* Initialize the UART console. */
  configure_console();
  printf(STRING_HEADER);

  xTaskCreate(task_wifi, "Wifi", TASK_WIFI_STACK_SIZE, NULL, TASK_WIFI_PRIORITY, &xHandleWifi);
  xTaskCreate(task_process, "process", TASK_PROCESS_STACK_SIZE, NULL, TASK_PROCESS_PRIORITY, NULL);

  xSemaphoreBut = xSemaphoreCreateBinary();
  if (xSemaphoreBut == NULL)
    printf("failed to create xSemaphoreBut\n");

  vTaskStartScheduler();

  while (1)
  {
  };
  return 0;
}