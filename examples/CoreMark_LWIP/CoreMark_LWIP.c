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
#include "w5x00_lwip.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/etharp.h"

#include "core_main.h"


/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
#define RECV_TASK_STACK_SIZE 512
#define RECV_TASK_PRIORITY 9

#define SEND_TASK_STACK_SIZE 512
#define SEND_TASK_PRIORITY 11

#define TCP_TASK_STACK_SIZE 2048
#define TCP_TASK_PRIORITY 10 

#define COREMARK_TASK_STACK_SIZE 4096
#define COREMARK_TASK_PRIORITY 8

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_MACRAW 0
#define TARGET_PORT 5000
static uint8_t target_ip[4] = {192, 168, 2, 100};

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
extern uint8_t mac[6];
static ip_addr_t g_ip;
static ip_addr_t g_mask;
static ip_addr_t g_gateway;
static ip_addr_t g_target_ip;

/* LWIP */
static struct netif g_netif;
static struct tcp_pcb *tcp_client_pcb = NULL;

static uint8_t eth_tx_buf[ETHERNET_BUF_MAX_SIZE];
//static uint8_t eth_rx_buf[ETHERNET_BUF_MAX_SIZE];
static uint8_t pack[ETHERNET_MTU];

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
void send_task(void *argument);

/* Clock */
static void set_clock_khz(void);

/* Callback */
static void gpio_callback(void);

/* LWIP */
static err_t tcp_callback_connected(void *arg, struct tcp_pcb *pcb_new, err_t err);
static err_t tcp_callback_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t tcp_callback_received(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t tcp_callback_poll(void *arg, struct tcp_pcb *tpcb);
static void tcp_callback_error(void *arg, err_t err);
static void tcp_client_close(struct tcp_pcb *tpcb);

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
    //network_initialize(g_net_info);
    //wizchip_1ms_timer_initialize(repeating_timer_callback);
    //wizchip_gpio_interrupt_initialize(SOCKET_NUM, gpio_callback);

    setSHAR(mac);
    ctlwizchip(CW_RESET_PHY, 0);

    // Initialize LWIP in NO_SYS mode
    lwip_init();

    sleep_ms(1000);

    //xTaskCreate(recv_task, "RECV_Task", RECV_TASK_STACK_SIZE, NULL, RECV_TASK_PRIORITY, NULL);
    xTaskCreate(tcp_task, "TCP_Task", TCP_TASK_STACK_SIZE, NULL, TCP_TASK_PRIORITY, NULL);
    xTaskCreate(send_task, "Send_Task", SEND_TASK_STACK_SIZE, NULL, SEND_TASK_PRIORITY, NULL);
    xTaskCreate(coremark_task, "COREMARK_Task", COREMARK_TASK_STACK_SIZE, NULL, COREMARK_TASK_PRIORITY, NULL);

    recv_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    coremark_start_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    send_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);

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
    /* Initialize */
    int retval = 0;
    struct pbuf *p = NULL;
    uint32_t pack_len = 0;

    // Initialize network configuration
    IP4_ADDR(&g_ip, 192, 168, 2, 120);
    IP4_ADDR(&g_mask, 255, 255, 255, 0);
    IP4_ADDR(&g_gateway, 192, 168, 2, 1);
    IP4_ADDR(&g_target_ip, 192, 168, 2, 100);


    netif_add(&g_netif, &g_ip, &g_mask, &g_gateway, NULL, netif_initialize, netif_input);
    g_netif.name[0] = 'e';
    g_netif.name[1] = '0';

    // Assign callbacks for link and status
    netif_set_link_callback(&g_netif, netif_link_callback);
    netif_set_status_callback(&g_netif, netif_status_callback);

    // MACRAW socket open
    retval = socket(SOCKET_MACRAW, Sn_MR_MACRAW, TARGET_PORT, 0x00);

    if (retval < 0)
    {
        printf(" MACRAW socket open failed\n");
    }

    // Set the default interface and bring it up
    netif_set_link_up(&g_netif);
    netif_set_up(&g_netif);

    tcp_client_pcb = (struct tcp_pcb *)tcp_new();
	if (tcp_client_pcb == NULL)
	{
        printf("tcp_client_pcb == NULL\r\n");
        while(1);
	}

    retval = tcp_connect(tcp_client_pcb, &g_target_ip, TARGET_PORT, tcp_callback_connected);
    if (retval != ERR_OK)
    {
        printf(" Connect failed err = %d\r\n", retval);
        /* Deallocate the pcb */
        memp_free(MEMP_TCP_PCB, tcp_client_pcb);
        while(1);
    }

    /* Infinite loop */
    while (1)
    {
        getsockopt(SOCKET_MACRAW, SO_RECVBUF, &pack_len);
        if (pack_len > 0)
        {
            pack_len = recv_lwip(SOCKET_MACRAW, (uint8_t *)pack, pack_len);
            if (pack_len)
            {
                p = pbuf_alloc(PBUF_RAW, pack_len, PBUF_POOL);
                pbuf_take(p, pack, pack_len);
            }
            else
            {
                printf(" No packet received\n");
            }

            if (pack_len && p != NULL)
            {
                LINK_STATS_INC(link.recv);
                if (g_netif.input(p, &g_netif) != ERR_OK)
                {
                    printf("g_netif.input(p, &g_netif) != ERR_OK\r\n");
                    pbuf_free(p);
                }
            }
        }
        sys_check_timeouts();
        vTaskDelay(1);
    }
}

#if 0
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

        if (socket_num == SOCKET_MACRAW)
        {
            reg_val = (SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT) & 0x00FF; // except SIK_SENT(send OK) interrupt
            ctlsocket(socket_num, CS_CLR_INTERRUPT, (void *)&reg_val);
            getsockopt(socket_num, SO_RECVBUF, (void *)&recv_len);

            if (recv_len > 0)
            {
                memset(eth_rx_buf, 0, ETHERNET_BUF_MAX_SIZE);
                recv(SOCKET_MACRAW, eth_rx_buf, recv_len);
                printf("%s\n", eth_rx_buf);
            }
        }
    }
}
#endif

void send_task(void *argument)
{
    uint32_t send_count = 0;

    while (1)
    {
        xSemaphoreTake(send_sem, portMAX_DELAY);
        send_count++;
        sprintf(eth_tx_buf, "send count = %d\r\n\0", send_count);
        tcp_write(tcp_client_pcb, eth_tx_buf, strlen(eth_tx_buf), 0);
    }
}

void coremark_task(void *argument)
{
  xSemaphoreTake(coremark_start_sem, portMAX_DELAY);
  
  coremark_test();
  printf("Test finished LWIP\r\n");
  vTaskSuspendAll();

  while(1)
  {
    vTaskDelay(1000);
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

static err_t tcp_callback_connected(void *arg, struct tcp_pcb *pcb_new, err_t err)
{
  LWIP_UNUSED_ARG(arg);
  printf("[%s] err = %d\r\n", __func__, err);
  
  if (err != ERR_OK) //error when connect to the server
      return err;
  
  tcp_setprio(pcb_new, 64); //set priority for the client pcb

  tcp_arg (pcb_new, 0); //no argument is used
  tcp_err (pcb_new, tcp_callback_error); //register error callback
  tcp_sent(pcb_new, tcp_callback_sent); //register send callback
  tcp_recv(pcb_new, tcp_callback_received);  //register receive callback
  tcp_poll(pcb_new, tcp_callback_poll, 0); //register poll callback

  xSemaphoreGive(send_sem);
  xSemaphoreGive(coremark_start_sem);
  return ERR_OK;
}

static err_t tcp_callback_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(tpcb);
  LWIP_UNUSED_ARG(len);

  return ERR_OK;
}

static err_t tcp_callback_received(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    int i;
    err_t ret_err;
    int socket_number;

    if (p == NULL) //pbuf is null when session is closed
    {
        tcp_client_close(tpcb);
        printf("Client socket closed => 0x%x\n", tpcb);
        ret_err = ERR_OK;
    }
    else if (err != ERR_OK) //ERR_ABRT is returned when called tcp_abort
    {
        tcp_recved(tpcb, p->tot_len); //advertise window size
        pbuf_free(p); //free pbuf
        ret_err = err;
    }
    else //receiving data
    {
        if (socket_number < 0)
        {
            printf("invalid socket number\n");
            pbuf_free(p);
            return err;
        }

        tcp_recved(tpcb, p->tot_len); 
        printf("%.*s\r\n", p->len, p->payload);
        pbuf_free(p); //free pbuf
        ret_err = ERR_OK;
        xSemaphoreGive(send_sem);
    }
  return ret_err;
}


static void tcp_callback_error(void *arg, err_t err)
{
  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);
}


static err_t tcp_callback_poll(void *arg, struct tcp_pcb *tpcb)
{
  return ERR_OK;
}

static void tcp_client_close(struct tcp_pcb *tpcb)
{
    /* Clear callback functions */
    tcp_arg(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_err(tpcb, NULL);
    tcp_poll(tpcb, NULL, 0);
    tcp_close(tpcb); // close connection
}


