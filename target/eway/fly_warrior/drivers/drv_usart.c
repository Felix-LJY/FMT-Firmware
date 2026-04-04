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

#define USING_UART6
#define USING_UART3

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
        /* dma trans status */
        uint32_t dma_ts_chact;
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
    rt_size_t recv_len;
    rt_base_t level;
    /* disable interrupt */
    level = rt_hw_interrupt_disable();
    /* reconfig rx_dma */
    DMA_SetDestAddr(uart->dma.rx_dma, uart->dma.rx_dma_ch, (uint32_t)((struct serial_rx_fifo*)serial->serial_rx)->buffer);
    DMA_SetTransCount(uart->dma.rx_dma, uart->dma.rx_dma_ch, uart->dma.setting_recv_len);
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
    /* clear timeout flag */
    USART_ClearStatus(uart->uart_periph, USART_FLAG_RX_TIMEOUT | USART_FLAG_OVERRUN);
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
        .dma_ts_chact            = DMA_CHSTAT_CHACT_0,
        .rx_dma_tc_irqn          = INT003_IRQn,
        .rx_dma_tc_int_src       = INT_SRC_DMA1_TC0,
        .rx_dma_tc_irq_call_back = uart6_rx_dma_tc_isr,
        .tx_dma_tc_irqn          = INT004_IRQn,
        .tx_dma_tc_int_src       = INT_SRC_DMA2_TC0,
        .tx_dma_tc_irq_call_back = uart6_tx_dma_tc_isr,
    },
};
#endif

#ifdef USING_UART3
static struct serial_device serial2;

static void uart3_rx_full_isr(void)
{
    /* high-level ISR routine */
    hal_serial_isr(&serial2, SERIAL_EVENT_RX_IND);
    /* the rx full interrupt flag will be cleared by hal call usart_getc() */
}

static void uart3_tx_cplt_isr(void)
{
    dma_tx_done_isr(&serial2);
    /* the tx complete interrupt flag is cleared in the ISR */
}

static void uart3_tx_dma_tc_isr(void)
{
    struct wl32_uart* uart = (struct wl32_uart*)serial2.parent.user_data;
    /* Enable tx complete interrupt */
    USART_FuncCmd(uart->uart_periph, USART_INT_TX_CPLT, ENABLE);
    /* Clear tx_dma status */
    DMA_ClearTransCompleteStatus(uart->dma.tx_dma, uart->dma.dma_tc_flag);
}

struct wl32_uart uart3 = {
    .uart_periph                 = CM_USART3,
    .per_clk                     = FCG3_PERIPH_USART3,
    .rx_port                     = GPIO_PORT_B,
    .rx_pin                      = GPIO_PIN_11,
    .rx_af                       = GPIO_FUNC_33,
    .tx_port                     = GPIO_PORT_B,
    .tx_pin                      = GPIO_PIN_10,
    .tx_af                       = GPIO_FUNC_32,
    .rx_full_irqn                = INT005_IRQn,
    .rx_full_int_src             = INT_SRC_USART3_RI,
    .rx_full_irq_call_back       = uart3_rx_full_isr,
    .rx_timeout_timer            = NULL,
    .rx_timeout_timer_ch         = -1,
    .rx_timeout_irqn             = -1,
    .rx_timeout_int_src          = -1,
    .rx_timeout_irq_call_back    = NULL,
    .tx_cplt_irqn                = INT006_IRQn,
    .tx_cplt_int_src             = INT_SRC_USART3_TCI,
    .tx_cplt_irq_call_back       = uart3_tx_cplt_isr,
    .dma = {
        .rx_dma                  = NULL,
        .rx_dma_ch               = -1,
        .rx_dma_trig_sel         = -1,
        .rx_dma_trig_src         = -1,
        .tx_dma                  = CM_DMA2,
        .tx_dma_ch               = DMA_CH1,
        .tx_dma_trig_sel         = AOS_DMA2_1,
        .tx_dma_trig_src         = EVT_SRC_USART3_TI,
        .dma_tc_int              = DMA_INT_TC_CH1,
        .dma_tc_flag             = DMA_FLAG_TC_CH1,
        .dma_ts_chact            = DMA_CHSTAT_CHACT_1,
        .rx_dma_tc_irqn          = -1,
        .rx_dma_tc_int_src       = -1,
        .rx_dma_tc_irq_call_back = NULL,
        .tx_dma_tc_irqn          = INT007_IRQn,
        .tx_dma_tc_int_src       = INT_SRC_DMA2_TC1,
        .tx_dma_tc_irq_call_back = uart3_tx_dma_tc_isr,
    },
};
#endif

static void _dma_transmit(struct wl32_uart* uart, rt_uint8_t* buf, rt_size_t size)
{
    /* Disable TX && TX interrupt function */
    USART_FuncCmd(uart->uart_periph, (USART_TX | USART_INT_TX_EMPTY), DISABLE);
    /* config & enable tx_dma */
    DMA_SetSrcAddr(uart->dma.tx_dma, uart->dma.tx_dma_ch, (uint32_t)buf);
    DMA_SetTransCount(uart->dma.tx_dma, uart->dma.tx_dma_ch, size);
    DMA_ChCmd(uart->dma.tx_dma, uart->dma.tx_dma_ch, ENABLE);
    /* Enable TX && TX interrupt function */
    USART_FuncCmd(uart->uart_periph, (USART_TX | USART_INT_TX_EMPTY), ENABLE);
}

static void _dma_receive(struct wl32_uart* uart)
{
    /* Disable RX && RX interrupt function */
    USART_FuncCmd(uart->uart_periph, (USART_RX | USART_INT_RX | USART_RX_TIMEOUT | USART_INT_RX_TIMEOUT), DISABLE);
    /* enable rx_dma */
    DMA_Cmd(uart->dma.rx_dma, ENABLE);
    DMA_TransCompleteIntCmd(uart->dma.rx_dma, uart->dma.dma_tc_int, ENABLE);
    DMA_ChCmd(uart->dma.rx_dma, uart->dma.rx_dma_ch, ENABLE);
    /* Enable RX && RX interrupt function */
    USART_FuncCmd(uart->uart_periph, (USART_RX | USART_INT_RX | USART_RX_TIMEOUT | USART_INT_RX_TIMEOUT), ENABLE);
}

static void _dma_rx_config(struct wl32_uart* uart, rt_uint8_t* buf, rt_size_t size)
{
    int32_t i32Ret;
    stc_dma_init_t stcDmaInit;
    /* set expected receive length */
    uart->dma.setting_recv_len = size;
    /* USART_RX_DMA */
    DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCount = size;
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr = (uint32_t)buf;
    stcDmaInit.u32SrcAddr = (uint32_t)(&uart->uart_periph->RDR);
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
    i32Ret = DMA_Init(uart->dma.rx_dma, uart->dma.rx_dma_ch, &stcDmaInit);
    if (LL_OK == i32Ret) {
        /* Set trigger event source. */
        AOS_SetTriggerEventSrc(uart->dma.rx_dma_trig_sel, uart->dma.rx_dma_trig_src);
    }
    _dma_receive(uart);
}

static void _dma_tx_config(struct wl32_uart* uart)
{
    int32_t i32Ret;
    stc_dma_init_t stcDmaInit;

    /* USART_TX_DMA */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCount = 0;
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr = (uint32_t)(&uart->uart_periph->TDR);
    stcDmaInit.u32SrcAddr = (uint32_t)NULL;
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    i32Ret = DMA_Init(uart->dma.tx_dma, uart->dma.tx_dma_ch, &stcDmaInit);
    if (LL_OK == i32Ret) {
        AOS_SetTriggerEventSrc(uart->dma.tx_dma_trig_sel, uart->dma.tx_dma_trig_src);

        DMA_Cmd(uart->dma.tx_dma, ENABLE);
        DMA_TransCompleteIntCmd(uart->dma.tx_dma, uart->dma.dma_tc_int, ENABLE);
    }
}

static void _close_usart(struct serial_device* serial)
{
    struct wl32_uart* uart = (struct wl32_uart*)serial->parent.user_data;

    if (serial->parent.open_flag & RT_DEVICE_FLAG_INT_RX) {
        /* disable int rx_full irq */
        USART_FuncCmd(uart->uart_periph, USART_INT_RX, DISABLE);
    }

    if (serial->parent.open_flag & RT_DEVICE_FLAG_DMA_RX) {
        /* disable dma rx irq and disable rx dma */
        DMA_TransCompleteIntCmd(uart->dma.rx_dma, uart->dma.dma_tc_int, DISABLE);
        USART_FuncCmd(uart->uart_periph, (USART_RX_TIMEOUT | USART_INT_RX_TIMEOUT), DISABLE);
        DMA_ChCmd(uart->dma.rx_dma, uart->dma.rx_dma_ch, DISABLE);
    }

    if (serial->parent.open_flag & RT_DEVICE_FLAG_DMA_TX) {
        /* disable dma tx irq and disable tx dma */
        DMA_TransCompleteIntCmd(uart->dma.tx_dma, uart->dma.dma_tc_int, DISABLE);
        USART_FuncCmd(uart->uart_periph, USART_INT_TX_CPLT, DISABLE);
        DMA_ChCmd(uart->dma.tx_dma, uart->dma.tx_dma_ch, DISABLE);
    }
    /* reset last recv index */
    uart->dma.last_recv_index = 0;
}

static rt_err_t usart_configure(struct serial_device* serial, struct serial_configure* cfg)
{
    stc_usart_uart_init_t stcUartInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    struct wl32_uart* uart = (struct wl32_uart*)serial->parent.user_data;
    /* Check parameters */
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    /* Write enable */
    LL_PERIPH_WE(LL_PERIPH_SEL);
    /* Configure USART RX/TX pin. */
    GPIO_SetFunc(uart->rx_port, uart->rx_pin, uart->rx_af);
    GPIO_SetFunc(uart->tx_port, uart->tx_pin, uart->tx_af);
    /* Enable peripheral clock */
    FCG_Fcg3PeriphClockCmd(uart->per_clk, ENABLE);
    /* Register rx_full irq */
    stcIrqSignConfig.enIRQn  = uart->rx_full_irqn;
    stcIrqSignConfig.enIntSrc = uart->rx_full_int_src;
    stcIrqSignConfig.pfnCallback = uart->rx_full_irq_call_back;
    INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_01);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
    /* Register rx_dma irq */
    if(uart->dma.rx_dma_tc_irqn)
    {
        if(!uart->rx_timeout_irqn)
            return RT_EINVAL;
        /* Register rx_timeout irq */
        stcIrqSignConfig.enIRQn  = uart->rx_timeout_irqn;
        stcIrqSignConfig.enIntSrc = uart->rx_timeout_int_src;
        stcIrqSignConfig.pfnCallback = uart->rx_timeout_irq_call_back;
        INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_01);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
        /* Register rx_dma_tc irq */
        stcIrqSignConfig.enIRQn  = uart->dma.rx_dma_tc_irqn;
        stcIrqSignConfig.enIntSrc = uart->dma.rx_dma_tc_int_src;
        stcIrqSignConfig.pfnCallback = uart->dma.rx_dma_tc_irq_call_back;
        INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_02);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
    }
    /* Register tx_dma irq */
    if(uart->dma.tx_dma_tc_irqn)
    {
        if(!uart->tx_cplt_irqn)
            return RT_EINVAL;
        /* Register tx_cplt irq */
        stcIrqSignConfig.enIRQn  = uart->tx_cplt_irqn;
        stcIrqSignConfig.enIntSrc = uart->tx_cplt_int_src;
        stcIrqSignConfig.pfnCallback = uart->tx_cplt_irq_call_back;
        INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_01);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
        /* Register tx_dma_tc irq */
        stcIrqSignConfig.enIRQn  = uart->dma.tx_dma_tc_irqn;
        stcIrqSignConfig.enIntSrc = uart->dma.tx_dma_tc_int_src;
        stcIrqSignConfig.pfnCallback = uart->dma.tx_dma_tc_irq_call_back;
        INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_02);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
    }
    /* Initialize UART structure */
    USART_UART_StructInit(&stcUartInit);
    /* Set parameters */
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
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_ENABLE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    /* Disable all USART function */
    USART_FuncCmd(uart->uart_periph, USART_FUNC_ALL, DISABLE);
    /* Initialize UART */
    if (LL_OK != USART_UART_Init(uart->uart_periph, &stcUartInit, NULL))
        return RT_EIO;
    /* MCU Peripheral registers write protected */
    LL_PERIPH_WP(LL_PERIPH_SEL);
    /* Enable RX/TX function */
    USART_FuncCmd(uart->uart_periph, (USART_RX | USART_TX), ENABLE);
    return RT_EOK;
}
static rt_err_t usart_control(struct serial_device* serial, int cmd, void* arg)
{
    struct wl32_uart* uart;
    rt_uint32_t ctrl_arg = (rt_uint32_t)(arg);
    RT_ASSERT(serial != RT_NULL);
    uart = (struct wl32_uart*)serial->parent.user_data;
    /* Write enabled */
    LL_PERIPH_WE(LL_PERIPH_SEL);
    switch (cmd) {
    case RT_DEVICE_CTRL_CLR_INT:
        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX) {
            USART_FuncCmd(uart->uart_periph, USART_INT_RX, DISABLE);
            NVIC_DisableIRQ(uart->rx_full_irqn);
            NVIC_ClearPendingIRQ(uart->rx_full_irqn);
        }
        break;
    case RT_DEVICE_CTRL_SET_INT:
        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX) {
            NVIC_ClearPendingIRQ(uart->rx_full_irqn);
            NVIC_EnableIRQ(uart->rx_full_irqn);
            USART_FuncCmd(uart->uart_periph, USART_INT_RX, ENABLE);
        }
        break;
    case RT_DEVICE_CTRL_CONFIG:
        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX) {
            struct serial_rx_fifo* rx_fifo = (struct serial_rx_fifo*)serial->serial_rx;
            struct wl32_uart* uart = (struct wl32_uart*)serial->parent.user_data;

            if (DMA_GetTransStatus(uart->dma.rx_dma, uart->dma.dma_ts_chact) == SET) {
                /* dma is busy */
                return RT_EBUSY;
            }
            _dma_rx_config(uart, rx_fifo->buffer, serial->config.bufsz);
        }

        if (ctrl_arg == RT_DEVICE_FLAG_DMA_TX) {
            struct wl32_uart* uart = (struct wl32_uart*)serial->parent.user_data;

            if (DMA_GetTransStatus(uart->dma.tx_dma, uart->dma.dma_ts_chact) == SET) {
                /* dma is busy */
                return RT_EBUSY;
            }
            _dma_tx_config(uart);
        }
        break;
    case RT_DEVICE_CTRL_SUSPEND:
        _close_usart(serial);
        break;
    default:
        return RT_EINVAL;
    }
    /* Write protected */
    LL_PERIPH_WP(LL_PERIPH_SEL);
    return RT_EOK;
}

static int usart_putc(struct serial_device* serial, char ch)
{
    struct wl32_uart* uart;
    RT_ASSERT(serial != RT_NULL);
    uart = (struct wl32_uart*)serial->parent.user_data;
    /* Wait Tx data register empty */
    while (RESET == USART_GetStatus(uart->uart_periph, USART_FLAG_TX_EMPTY)) 
        ;
    uart->uart_periph->TDR = ch;
    return RT_EOK;
}

static int usart_getc(struct serial_device* serial)
{
    int ch = -1;
    struct wl32_uart* uart;
    RT_ASSERT(serial != RT_NULL);
    /* Wait Rx data register full */
    uart = (struct wl32_uart*)serial->parent.user_data;
    if (SET == USART_GetStatus(uart->uart_periph, USART_FLAG_RX_FULL)) {
        ch = uart->uart_periph->RDR;
    }
    return ch;
}

static rt_size_t usart_dma_transmit(struct serial_device* serial, rt_uint8_t* buf, rt_size_t size, int direction)
{
    if (direction == SERIAL_DMA_TX && size > 0) {
        /* Wait Tx complete */
        while (RESET == USART_GetStatus(((struct wl32_uart*)serial->parent.user_data)->uart_periph, USART_FLAG_TX_CPLT)) 
            ;
        if(size > 1)
        {
            if(buf[size-1]=='\n')
                buf[size++]='\r';
            _dma_transmit((struct wl32_uart*)serial->parent.user_data, buf, size);
        }
        else
        {
            if(buf[0]=='\n')
                usart_putc(serial, '\r');
            USART_FuncCmd(((struct wl32_uart*)serial->parent.user_data)->uart_periph, USART_INT_TX_CPLT, ENABLE);
            usart_putc(serial, buf[0]);
        }
    }

    return size;
}

/**
 * @brief  Configure TMR0.
 * @param  [in] u16TimeoutBits:         Timeout bits
 * @retval None
 */
static void TMR0_Config(uint16_t u16TimeoutBits)
{
    uint16_t u16Div;
    uint16_t u16Delay;
    uint16_t u16CompareValue;
    stc_tmr0_init_t stcTmr0Init;

    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE);
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE);
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS , ENABLE);
#if defined(USING_UART1) || defined(USING_UART2)
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_1, ENABLE);
#endif
#if defined(USING_UART6) || defined(USING_UART7)
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_2, ENABLE);
#endif

    /* Initialize TMR0 base function. */
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_XTAL32;
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV8;
    stcTmr0Init.u32Func     = TMR0_FUNC_CMP;
    if (TMR0_CLK_DIV1 == stcTmr0Init.u32ClockDiv) {
        u16Delay = 7U;
    } else if (TMR0_CLK_DIV2 == stcTmr0Init.u32ClockDiv) {
        u16Delay = 5U;
    } else if ((TMR0_CLK_DIV4 == stcTmr0Init.u32ClockDiv) || \
               (TMR0_CLK_DIV8 == stcTmr0Init.u32ClockDiv) || \
               (TMR0_CLK_DIV16 == stcTmr0Init.u32ClockDiv)) {
        u16Delay = 3U;
    } else {
        u16Delay = 2U;
    }

    u16Div = (uint16_t)1U << (stcTmr0Init.u32ClockDiv >> TMR0_BCONR_CKDIVA_POS);
    u16CompareValue = ((u16TimeoutBits + u16Div - 1U) / u16Div) - u16Delay;
    stcTmr0Init.u16CompareValue = u16CompareValue;
#ifdef USING_UART1
    TMR0_Init(CM_TMR0_1, TMR0_CH_A, &stcTmr0Init);
    TMR0_HWStartCondCmd(CM_TMR0_1, TMR0_CH_A, ENABLE);
    TMR0_HWClearCondCmd(CM_TMR0_1, TMR0_CH_A, ENABLE);
#endif
#ifdef USING_UART2
    TMR0_Init(CM_TMR0_1, TMR0_CH_B, &stcTmr0Init);
    TMR0_HWStartCondCmd(CM_TMR0_1, TMR0_CH_B, ENABLE);
    TMR0_HWClearCondCmd(CM_TMR0_1, TMR0_CH_B, ENABLE);
#endif
#ifdef USING_UART6
    TMR0_Init(CM_TMR0_2, TMR0_CH_A, &stcTmr0Init);
    TMR0_HWStartCondCmd(CM_TMR0_2, TMR0_CH_A, ENABLE);
    TMR0_HWClearCondCmd(CM_TMR0_2, TMR0_CH_A, ENABLE);
#endif
#ifdef USING_UART7
    TMR0_Init(CM_TMR0_2, TMR0_CH_B, &stcTmr0Init);
    TMR0_HWStartCondCmd(CM_TMR0_2, TMR0_CH_B, ENABLE);
    TMR0_HWClearCondCmd(CM_TMR0_2, TMR0_CH_B, ENABLE);
#endif
}

/* usart driver operations */
static const struct usart_ops _usart_ops = {
    .configure = usart_configure,
    .control = usart_control,
    .putc = usart_putc,
    .getc = usart_getc,
    .dma_transmit = usart_dma_transmit,
};

rt_err_t drv_usart_init(void)
{
    rt_err_t rt_err = RT_EOK;
    struct serial_configure __unused config = SERIAL_DEFAULT_CONFIG;

    TMR0_Config(2000);

#ifdef USING_UART6
    serial1.ops = &_usart_ops;
    #ifdef SERIAL1_DEFAULT_CONFIG
    struct serial_configure serial1_config = SERIAL1_DEFAULT_CONFIG;
    serial1.config = serial1_config;
    #else
    serial1.config = config;
    #endif

    /* register serial device */
    rt_err |= hal_serial_register(
        &serial1,
        "serial1",
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
        &uart6);
#endif /* USING_UART6 */

#ifdef USING_UART3
    serial2.ops = &_usart_ops;
    #ifdef SERIAL2_DEFAULT_CONFIG
    struct serial_configure serial2_config = SERIAL2_DEFAULT_CONFIG;
    serial2.config = serial2_config;
    #else
    serial2.config = config;
    #endif

    /* register serial device */
    rt_err |= hal_serial_register(
        &serial2,
        "serial0",
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX,
        &uart3);
#endif /* USING_UART3 */


    return rt_err;
}
