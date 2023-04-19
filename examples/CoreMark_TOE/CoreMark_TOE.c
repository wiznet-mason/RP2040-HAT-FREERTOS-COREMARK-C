/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "port_common.h"

#include "socket.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "w5x00_gpio_irq.h"

#include "timer.h"

#include "core_main.h"


/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
#define RECV_TASK_STACK_SIZE 512
#define RECV_TASK_PRIORITY 9

#define TCP_TASK_STACK_SIZE 2048
#define TCP_TASK_PRIORITY 10 

#define COREMARK_TASK_STACK_SIZE 4096
#define COREMARK_TASK_PRIORITY 8

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_NUM 0
#define TARGET_PORT 5000
static uint8_t target_ip[4] = {192, 168, 2, 100};

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 2, 120},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 2, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .dhcp = NETINFO_STATIC                         // DHCP enable/disable
};

static uint8_t eth_tx_buf[ETHERNET_BUF_MAX_SIZE];
static uint8_t eth_rx_buf[ETHERNET_BUF_MAX_SIZE];

static xSemaphoreHandle coremark_start_sem;
static xSemaphoreHandle recv_sem;
static xSemaphoreHandle send_sem;

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
void tcp_task(void *argument);
void coremark_task(void *argument);
void recv_task(void *argument);

/* Clock */
static void set_clock_khz(void);

/* Callback */
static void gpio_callback(void);

/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */
int main()
{
    /* Initialize */
    set_clock_khz();

    stdio_init_all();

    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
    network_initialize(g_net_info);
    //wizchip_1ms_timer_initialize(repeating_timer_callback);
    wizchip_gpio_interrupt_initialize(SOCKET_NUM, gpio_callback);

    sleep_ms(1000);

    xTaskCreate(recv_task, "RECV_Task", RECV_TASK_STACK_SIZE, NULL, RECV_TASK_PRIORITY, NULL);
    xTaskCreate(tcp_task, "TCP_Task", TCP_TASK_STACK_SIZE, NULL, TCP_TASK_PRIORITY, NULL);
    xTaskCreate(coremark_task, "COREMARK_Task", COREMARK_TASK_STACK_SIZE, NULL, COREMARK_TASK_PRIORITY, NULL);

    send_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    recv_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    coremark_start_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);

    vTaskStartScheduler();

    while (1)
    {
        ;
    }
}

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
void tcp_task(void *argument)
{
  int ret;
  uint32_t data_len;
  uint32_t send_delay = 0;
  uint32_t send_count = 0;

  printf("%s\r\n", __func__);

  ret = socket(SOCKET_NUM, Sn_MR_TCP, TARGET_PORT, SF_TCP_NODELAY);
  if (ret != SOCKET_NUM)
  {
      printf(" Socket failed %d\n", ret);
      while (1)
      {
        vTaskDelay(10000000);
      }
  }

  while(1)
  {
      ret = connect(SOCKET_NUM, target_ip, TARGET_PORT);
      if (ret == SOCK_OK)
          break;
      vTaskDelay(5000);
  }
  xSemaphoreGive(coremark_start_sem);
  
  while(1)
  {
    send_count++;    
    sprintf(eth_tx_buf, "send count = %d\r\n\0", send_count);
    send(SOCKET_NUM, eth_tx_buf, strlen(eth_tx_buf));
    xSemaphoreTake(send_sem, portMAX_DELAY);
  }
}

void recv_task(void *argument)
{
    uint8_t socket_num;
    uint16_t reg_val;
    uint16_t recv_len;

    while (1)
    {
        xSemaphoreTake(recv_sem, portMAX_DELAY);
        ctlwizchip(CW_GET_INTERRUPT, (void *)&reg_val);
#if _WIZCHIP_ == W5100S
        reg_val &= 0x00FF;
#elif _WIZCHIP_ == W5500
        reg_val = (reg_val >> 8) & 0x00FF;
#endif

        for (socket_num = 0; socket_num < _WIZCHIP_SOCK_NUM_; socket_num++)
        {
            if (reg_val & (1 << socket_num))
            {
                break;
            }
        }

        if (socket_num == SOCKET_NUM)
        {
            reg_val = (SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT) & 0x00FF; // except SIK_SENT(send OK) interrupt
            ctlsocket(socket_num, CS_CLR_INTERRUPT, (void *)&reg_val);
            getsockopt(socket_num, SO_RECVBUF, (void *)&recv_len);

            if (recv_len > 0)
            {
                memset(eth_rx_buf, 0, ETHERNET_BUF_MAX_SIZE);
                recv(SOCKET_NUM, eth_rx_buf, recv_len);
                printf("%s\n", eth_rx_buf);
                xSemaphoreGive(send_sem);
            }
        }
    }
}

void coremark_task(void *argument)
{
  xSemaphoreTake(coremark_start_sem, portMAX_DELAY);
  
  coremark_test();
  printf("Test finished TOE\r\n");
  vTaskSuspendAll();

  while(1)
  {
    //vTaskDelay(1000);
  }
}

/* Clock */
static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}

static void gpio_callback(void)
{
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(recv_sem, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
