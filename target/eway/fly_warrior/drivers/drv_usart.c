/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <firmament.h>

#include "drv_usart.h"
#include "hal/serial/serial.h"

#define LL_PERIPH_SEL       (LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_EFM | LL_PERIPH_SRAM)

#define USING_UART6         1
#define USING_UART3         1

#define SERIAL1_DEFAULT_CONFIG                 \
    {                                          \
        BAUD_RATE_921600, /* 921600 bits/s */  \
        DATA_BITS_8,      /* 8 databits */     \
        STOP_BITS_1,      /* 1 stopbit */      \
        PARITY_NONE,      /* No parity  */     \
        BIT_ORDER_LSB,    /* LSB first sent */ \
        NRZ_NORMAL,       /* Normal mode */    \
        SERIAL_RB_BUFSZ,  /* Buffer size */    \
        0                                      \
    }
#define SERIAL2_DEFAULT_CONFIG                 \
    {                                          \
        BAUD_RATE_115200, /* 115200 bits/s */  \
        DATA_BITS_8,      /* 8 databits */     \
        STOP_BITS_1,      /* 1 stopbit */      \
        PARITY_NONE,      /* No parity  */     \
        BIT_ORDER_LSB,    /* LSB first sent */ \
        NRZ_NORMAL,       /* Normal mode */    \
        SERIAL_RB_BUFSZ,  /* Buffer size */    \
        0                                      \
    }

/* uart driver */
struct wl32_uart {
    /* uart periph */
    CM_USART_TypeDef * uart_periph;
    uint32_t per_clk;
    uint32_t tx_port;
    uint16_t tx_af;
    uint16_t tx_pin;
    uint32_t rx_port;
    uint16_t rx_af;
    uint16_t rx_pin;
    /* rx full irq */
    uint32_t rx_full_irqn;
    uint32_t rx_full_int_src;
    void (*rx_full_irq_call_back)(void);
    /* rx timeout timer */
    CM_TMR0_TypeDef *rx_timeout_timer;
    uint32_t rx_timeout_timer_ch;
    /* rx timeout irq */
    uint32_t rx_timeout_irqn;
    uint32_t rx_timeout_int_src;
    void (*rx_timeout_irq_call_back)(void);
    /* tx complete irq */
    uint32_t tx_cplt_irqn;
    uint32_t tx_cplt_int_src;
    void (*tx_cplt_irq_call_back)(void);
    /* dma */
    struct wl32_uart_dma{
        /* rx dma */
        CM_DMA_TypeDef *rx_dma;
        uint8_t rx_dma_ch;
        uint32_t rx_dma_trig_sel;
        uint32_t rx_dma_trig_src;
        /* tx dma */
        CM_DMA_TypeDef *tx_dma;
        uint8_t tx_dma_ch;
        uint32_t tx_dma_trig_sel;
        uint32_t tx_dma_trig_src;
        /* dma tc interrupt */
        uint32_t dma_tc_int;
        uint32_t dma_tc_flag;
        /* rx dma tc irq */
        uint32_t rx_dma_tc_irqn;
        uint32_t rx_dma_tc_int_src;
        void (*rx_dma_tc_irq_call_back)(void);
        /* tx dma tc irq */
        uint32_t tx_dma_tc_irqn;
        uint32_t tx_dma_tc_int_src;
        void (*tx_dma_tc_irq_call_back)(void);
        /* setting receive len */
        rt_size_t setting_recv_len;
        /* last receive index */
        rt_size_t last_recv_index;
    }dma;
};

static rt_size_t usart_dma_transmit(struct serial_device* serial, rt_uint8_t* buf, rt_size_t size, int direction);
static int usart_getc(struct serial_device* serial);
static int usart_putc(struct serial_device* serial, char c);
static rt_err_t usart_control(struct serial_device* serial, int cmd, void* arg);
static rt_err_t usart_configure(struct serial_device* serial, struct serial_configure* cfg);

/**
 * @brief  Stop timeout timer.
 * @param  [in]  TMR0x                  Pointer to TMR0 instance register base.
 *                                      This parameter can be a value of the following:
 *   @arg  CM_TMR0_x or CM_TMR0
 * @param  [in]  u32Ch                  TMR0 channel.
 *                                      This parameter can be a value @ref TMR0_Channel
 */
static void USART_StopTimeoutTimer(CM_TMR0_TypeDef *TMR0x, uint32_t u32Ch)
{
    uint32_t u32ClrMask;
    uint32_t u32SetMask;
    uint32_t u32BitOffset;

    u32BitOffset = 16UL * u32Ch;

    /* Set: TMR0_BCONR.SYNCLKA<B>=1, TMR0_BCONR.SYNA<B>=0 */
    u32ClrMask = (TMR0_BCONR_SYNCLKA | TMR0_BCONR_SYNSA) << u32BitOffset;
    u32SetMask = TMR0_BCONR_SYNCLKA << u32BitOffset;
    MODIFY_REG32(TMR0x->BCONR, u32ClrMask, u32SetMask);

    /* Set: TMR0_BCONR.CSTA<B>=0, TMR0_BCONR.SYNCLKA<B>=0, TMR0_BCONR.SYNSA<B>=1 */
    u32ClrMask = (TMR0_BCONR_SYNCLKA | TMR0_BCONR_SYNSA | TMR0_BCONR_CSTA) << u32BitOffset;
    u32SetMask = TMR0_BCONR_SYNSA << u32BitOffset;
    MODIFY_REG32(TMR0x->BCONR, u32ClrMask, u32SetMask);
}

/**
 * Serial port receive timeout process. This need add to uart timeout ISR.
 *
 * @param serial serial device
 */
static void dma_uart_rx_timeout_isr(struct serial_device* serial)
{
    struct wl32_uart* uart = (struct wl32_uart*)serial->parent.user_data;
    rt_size_t recv_total_index, recv_len;
    rt_base_t level;
    uint32_t remain_bytes;
    /* disable interrupt */
    level = rt_hw_interrupt_disable();
    /* check remain bytes to receive */
    remain_bytes = DMA_GetTransCount(uart->dma.rx_dma, uart->dma.rx_dma_ch);
    /* total received bytes */
    recv_total_index = uart->dma.setting_recv_len - remain_bytes;
    /* received bytes at this time */
    recv_len = recv_total_index - uart->dma.last_recv_index;
    /* update last received total bytes */
    uart->dma.last_recv_index = recv_total_index;
    /* enable interrupt */
    rt_hw_interrupt_enable(level);
    /* check if received bytes */
    if (recv_len) {
        /* high-level ISR routine */
        hal_serial_isr(serial, SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
    }
    /* stop timeout timer */
    USART_StopTimeoutTimer(uart->rx_timeout_timer, uart->rx_timeout_timer_ch);
    /* clear timeout flag */
    USART_ClearStatus(uart->uart_periph, USART_FLAG_RX_TIMEOUT);
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void dma_rx_done_isr(struct serial_device* serial)
{
    struct wl32_uart* uart = (struct wl32_uart*)serial->parent.user_data;
    stc_dma_reconfig_init_t stcDmaReconfigInit;
    rt_size_t recv_len;
    rt_base_t level;
    /* disable interrupt */
    level = rt_hw_interrupt_disable();
    /* reconfig rx_dma */
    DMA_ReconfigStructInit(&stcDmaReconfigInit);
    stcDmaReconfigInit.u32CountMode      = DMA_RC_CNT_DEST;
    stcDmaReconfigInit.u32SrcAddrMode    = DMA_RC_SRC_ADDR_KEEP;
    stcDmaReconfigInit.u32DestAddrMode   = DMA_RC_DEST_ADDR_RPT;
    DMA_ReconfigInit(uart->dma.rx_dma, uart->dma.rx_dma_ch, &stcDmaReconfigInit);
    DMA_ReconfigCmd(uart->dma.rx_dma, ENABLE);
    AOS_SetTriggerEventSrc(AOS_DMA_RC,  EVT_SRC_AOS_STRG);
    /* Trigger for re-config USART RX DMA */
    AOS_SW_Trigger();
    /* disable dma reconfig */
    DMA_ReconfigCmd(uart->dma.rx_dma, DISABLE);
    /* received bytes at this time */
    recv_len = uart->dma.setting_recv_len - uart->dma.last_recv_index;
    /* reset last recv index */
    uart->dma.last_recv_index = 0;
    /* enable interrupt */
    rt_hw_interrupt_enable(level);
    /* check if received bytes */
    if (recv_len) {
        /* high-level ISR routine */
        hal_serial_isr(serial, SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
    }
    /* Clear rx_dma status */
    DMA_ClearTransCompleteStatus(uart->dma.rx_dma, uart->dma.dma_tc_flag);
    /* reenable rx_dma */
    DMA_ChCmd(uart->dma.rx_dma, uart->dma.rx_dma_ch, ENABLE);
}

/**
 * DMA transmit done process. This need add to uart transmit done ISR.
 *
 * @param serial serial device
 */
static void dma_tx_done_isr(struct serial_device* serial)
{
    struct wl32_uart* uart = (struct wl32_uart*)serial->parent.user_data;
    /* high-level ISR routine */
    hal_serial_isr(serial, SERIAL_EVENT_TX_DMADONE);
    /* Clear tx complete interrupt flag */
    USART_ClearStatus(uart->uart_periph, USART_FLAG_TX_CPLT);
    /* Disable tx complete interrupt */
    USART_FuncCmd(uart->uart_periph, USART_INT_TX_CPLT, DISABLE);
}

#ifdef USING_UART6
static struct serial_device serial1;

static void uart6_rx_full_isr(void)
{
    /* high-level ISR routine */
    hal_serial_isr(&serial1, SERIAL_EVENT_RX_IND);
    /* the rx full interrupt flag will be cleared by hal call usart_getc() */
}

static void uart6_rx_timeout_isr(void)
{
    dma_uart_rx_timeout_isr(&serial1);
    /* the timeout interrupt flag is cleared in the ISR */
}

static void uart6_tx_cplt_isr(void)
{
    dma_tx_done_isr(&serial1);
    /* the tx complete interrupt flag is cleared in the ISR */
}

static void uart6_rx_dma_tc_isr(void)
{
    dma_rx_done_isr(&serial1);
    /* the rx_dma status is cleared in the ISR */
}

static void uart6_tx_dma_tc_isr(void)
{
    struct wl32_uart* uart = (struct wl32_uart*)serial1.parent.user_data;
    /* Enable tx complete interrupt */
    USART_FuncCmd(uart->uart_periph, USART_INT_TX_CPLT, ENABLE);
    /* Clear tx_dma status */
    DMA_ClearTransCompleteStatus(uart->dma.tx_dma, uart->dma.dma_tc_flag);
}

struct wl32_uart uart6 = {
    .uart_periph                 = CM_USART6,
    .per_clk                     = FCG3_PERIPH_USART6,
    .rx_port                     = GPIO_PORT_A,
    .rx_pin                      = GPIO_PIN_00,
    .rx_af                       = GPIO_FUNC_37,
    .tx_port                     = GPIO_PORT_A,
    .tx_pin                      = GPIO_PIN_03,
    .tx_af                       = GPIO_FUNC_36,
    .rx_full_irqn                = INT000_IRQn,
    .rx_full_int_src             = INT_SRC_USART6_RI,
    .rx_full_irq_call_back       = uart6_rx_full_isr,
    .rx_timeout_timer            = CM_TMR0_2,
    .rx_timeout_timer_ch         = TMR0_CH_A,
    .rx_timeout_irqn             = INT001_IRQn,
    .rx_timeout_int_src          = INT_SRC_USART6_RTO,
    .rx_timeout_irq_call_back    = uart6_rx_timeout_isr,
    .tx_cplt_irqn                = INT002_IRQn,
    .tx_cplt_int_src             = INT_SRC_USART6_TCI,
    .tx_cplt_irq_call_back       = uart6_tx_cplt_isr,
    .dma = {
        .rx_dma                  = CM_DMA1,
        .rx_dma_ch               = DMA_CH0,
        .rx_dma_trig_sel         = AOS_DMA1_0,
        .rx_dma_trig_src         = EVT_SRC_USART6_RI,
        .tx_dma                  = CM_DMA2,
        .tx_dma_ch               = DMA_CH0,
        .tx_dma_trig_sel         = AOS_DMA2_0,
        .tx_dma_trig_src         = EVT_SRC_USART6_TI,
        .dma_tc_int              = DMA_INT_TC_CH0,
        .dma_tc_flag             = DMA_FLAG_TC_CH0,
        .rx_dma_tc_irqn          = INT003_IRQn,
        .rx_dma_tc_int_src       = INT_SRC_DMA1_TC0,
        .rx_dma_tc_irq_call_back = uart6_rx_dma_tc_isr,
        .tx_dma_tc_irqn          = INT004_IRQn,
        .tx_dma_tc_int_src       = INT_SRC_DMA2_TC0,
        .tx_dma_tc_irq_call_back = uart6_tx_dma_tc_isr,
    },
};
#endif

static rt_err_t usart_configure(struct serial_device* serial, struct serial_configure* cfg)
{
    stc_usart_uart_init_t stcUartInit;
    struct wl32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct wl32_uart*)serial->parent.user_data;

    LL_PERIPH_WE(LL_PERIPH_SEL);

    /* Configure USART RX/TX pin. */
    GPIO_SetFunc(uart->rx_port, uart->rx_pin, uart->rx_af);
    GPIO_SetFunc(uart->tx_port, uart->tx_pin, uart->tx_af);

    /* Enable peripheral clock */
    FCG_Fcg3PeriphClockCmd(uart->per_clk, ENABLE);

    /* Initialize UART. */
    (void)USART_UART_StructInit(&stcUartInit);

    stcUartInit.u32Baudrate = cfg->baud_rate;
    switch (cfg->data_bits) {
    case DATA_BITS_9:
        stcUartInit.u32DataWidth = USART_DATA_WIDTH_9BIT;
        break;

    default:
        stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
        break;
    }

    switch (cfg->stop_bits) {
    case STOP_BITS_2:
        stcUartInit.u32StopBit = USART_STOPBIT_2BIT;
        break;
    default:
        stcUartInit.u32StopBit = USART_STOPBIT_1BIT;
        break;
    }
    switch (cfg->parity) {
    case PARITY_ODD:
        stcUartInit.u32Parity = USART_PARITY_ODD;
        break;
    case PARITY_EVEN:
        stcUartInit.u32Parity = USART_PARITY_EVEN;
        break;
    default:
        stcUartInit.u32Parity = USART_PARITY_NONE;
        break;
    }

    stcUartInit.u32ClockDiv = USART_CLK_DIV64;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    /* Disable all USART function */
    USART_FuncCmd((CM_USART_TypeDef*)uart->uart_periph, USART_FUNC_ALL, DISABLE);
    if (LL_OK != USART_UART_Init((CM_USART_TypeDef*)uart->uart_periph, &stcUartInit, NULL))
        return RT_EIO;

    /* MCU Peripheral registers write protected */
    LL_PERIPH_WP(LL_PERIPH_SEL);

    /* Enable RX/TX function */
    USART_FuncCmd((CM_USART_TypeDef*)uart->uart_periph, (USART_RX | USART_INT_RX | USART_TX), ENABLE);
    return RT_EOK;
}
static rt_err_t usart_control(struct serial_device* serial, int cmd, void* arg)
{
    struct wl32_uart* uart;
    rt_uint32_t ctrl_arg = (rt_uint32_t)(arg);

    RT_ASSERT(serial != RT_NULL);
    uart = (struct wl32_uart*)serial->parent.user_data;
    LL_PERIPH_WE(LL_PERIPH_SEL);

    switch (cmd) {
    case RT_DEVICE_CTRL_CLR_INT:
        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX) {
            NVIC_ClearPendingIRQ(uart->rx_full_irqn);
            NVIC_DisableIRQ(uart->rx_full_irqn);
        }
        break;

    case RT_DEVICE_CTRL_SET_INT:
        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX) {
            NVIC_ClearPendingIRQ(uart->rx_full_irqn);
            NVIC_EnableIRQ(uart->rx_full_irqn);
        }
        break;

    case RT_DEVICE_CTRL_CONFIG:
        break;

    case RT_DEVICE_CTRL_SUSPEND:
        NVIC_ClearPendingIRQ(uart->rx_full_irqn);
        NVIC_DisableIRQ(uart->rx_full_irqn);
        break;

    default:
        break;
    }

    LL_PERIPH_WP(LL_PERIPH_SEL);
    return RT_EOK;
}

static int usart_putc(struct serial_device* serial, char ch)
{
    struct wl32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct wl32_uart*)serial->parent.user_data;

    /* Wait Tx data register empty */
    while (RESET == USART_GetStatus((CM_USART_TypeDef*)uart->uart_periph, USART_FLAG_TX_EMPTY)) {
    }
    USART_WriteData((CM_USART_TypeDef*)uart->uart_periph, ch);

    return RT_EOK;
}

static volatile uint8_t tmp_rx_buf[100] = { 0 };
static volatile uint8_t tmp_rx_add = 0;
static int usart_getc(struct serial_device* serial)
{
    int ch = -1;
    struct wl32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct wl32_uart*)serial->parent.user_data;

    if (SET == USART_GetStatus((CM_USART_TypeDef*)uart->uart_periph, USART_FLAG_RX_FULL)) {
        ch = USART_ReadData((CM_USART_TypeDef*)uart->uart_periph);
        tmp_rx_buf[tmp_rx_add] = ch;
        if (++tmp_rx_add >= 100)
            tmp_rx_add = 0;
    }

    return ch;
}
uint8_t tmp_dma_sendddd[] = "hello word\n";
void tmp_dma_send(uint8_t* buf, uint32_t size)
{
    DMA_SetSrcAddr(CM_DMA2, DMA_CH0, (uint32_t)buf);

    DMA_SetTransCount(CM_DMA2, DMA_CH0, size);

    (void)DMA_ChCmd(CM_DMA2, DMA_CH0, ENABLE);

    USART_FuncCmd(CM_USART4, USART_TX, ENABLE);
}
static rt_size_t usart_dma_transmit(struct serial_device* serial, rt_uint8_t* buf, rt_size_t size, int direction)
{
    tmp_dma_send(buf, size);
    return size;
}

/* usart driver operations */
static const struct usart_ops __usart_ops = {
    .configure = usart_configure,
    .control = usart_control,
    .putc = usart_putc,
    .getc = usart_getc,
    .dma_transmit = usart_dma_transmit,
};

rt_err_t drv_usart_init(void)
{
    rt_err_t rt_err = RT_EOK;
    struct serial_configure config = SERIAL_DEFAULT_CONFIG;

    return rt_err;
}
