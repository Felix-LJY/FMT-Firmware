/******************************************************************************
 * Copyright 2022 The Firmament Authors. All Rights Reserved.
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

#include "hal/rc/ppm.h"
#include "hal/rc/rc.h"
#include "hal/rc/sbus.h"

#ifndef min // mod by prife
    #define min(x, y) (x < y ? x : y)
#endif

/* capture accuracy is 0.000667ms */
#define PPM_DECODER_FREQUENCY 1500000

/* default config for rc device */
#define RC_CONFIG_DEFAULT                      \
    {                                          \
        RC_PROTOCOL_AUTO, /* auto */           \
            6,            /* 6 channel */      \
            0.05f,        /* sample time */    \
            1000,         /* minimal 1000us */ \
            2000,         /* maximal 2000us */ \
    }

static ppm_decoder_t ppm_decoder;
static sbus_decoder_t sbus_decoder;

static void TMRA_IrqCallback(void)
{
    static volatile uint32_t ic_val = 0;
    /* enter interrupt */
    rt_interrupt_enter();

    ic_val = TMRA_GetCompareValue(CM_TMRA_12, TMRA_CH4) + 1;
    /* Get capture value by calling function TMRA_GetCompareValue. */
    TMRA_ClearStatus(CM_TMRA_12, TMRA_FLAG_CMP_CH4);

    ppm_update(&ppm_decoder, ic_val);

    /* leave interrupt */
    rt_interrupt_leave();
}

static void USART_RxFull_IrqCallback(void)
{
    uint8_t ch;

    if (USART_GetStatus(CM_USART8, USART_FLAG_RX_FULL)) {
        do {
            ch = ~(uint8_t)USART_ReadData(CM_USART8);
            sbus_input(&sbus_decoder, &ch, 1);
        } while (USART_GetStatus(CM_USART8, USART_FLAG_RX_FULL));

        /* if it's reading sbus data, we just parse it later */
        if (!sbus_islock(&sbus_decoder)) {
            sbus_update(&sbus_decoder);
        }
        /* the RXNE flag is cleared by reading the USART_RDR register */
    }
}

static void USART_RxError_IrqCallback(void)
{
    USART_ReadData(CM_USART8);
    /* clear error flags */
    USART_ClearStatus(CM_USART8, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

static rt_err_t ppm_lowlevel_init(void)
{
    stc_gpio_init_t stcGpioInit;
    stc_tmra_init_t stcTmraInit;
    stc_irq_signin_config_t stcIrq;

    /* MCU Peripheral registers write unprotected. */
    LL_PERIPH_WE(LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU);
    /* Enable TimerA peripheral clock. */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_12, ENABLE);
    /* Configure GPIO pin. */
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PullUp = PIN_PU_ON;
    GPIO_Init(GPIO_PORT_B, GPIO_PIN_06, &stcGpioInit);
    /* Set a default initialization value for stcTmraInit. */
    TMRA_StructInit(&stcTmraInit);
    /* Modifies the initialization values depends on the application. */
    stcTmraInit.sw_count.u8ClockDiv = TMRA_CLK_DIV64; // 1.5MHz
    TMRA_Init(CM_TMRA_12, &stcTmraInit);
    /* Set function mode as capturing mode. */
    TMRA_SetFunc(CM_TMRA_12, TMRA_CH4, TMRA_FUNC_CAPT);
    /* Configures the capture condition. */
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_06, GPIO_FUNC_5);
    /* Configures IRQ */
    stcIrq.enIRQn      = INT098_IRQn;
    stcIrq.enIntSrc    = INT_SRC_TMRA_12_CMP;
    stcIrq.pfnCallback = &TMRA_IrqCallback;
    INTC_IrqSignIn(&stcIrq);
    NVIC_ClearPendingIRQ(stcIrq.enIRQn);
    NVIC_SetPriority(stcIrq.enIRQn, DDL_IRQ_PRIO_13);
    NVIC_EnableIRQ(stcIrq.enIRQn);
    /* MCU Peripheral registers write protected. */
    LL_PERIPH_WP(LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU);

    return RT_EOK;
}

static rt_err_t sbus_lowlevel_init(void)
{
    stc_gpio_init_t stcGpioInit;
    stc_usart_uart_init_t stcUartInit;
    stc_irq_signin_config_t stcIrqSignConfig;

    /* MCU Peripheral registers write unprotected */
    LL_PERIPH_WE(LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_EFM | LL_PERIPH_SRAM);
    /* Enable peripheral clock */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART8, ENABLE);
    /* Configure GPIO pin. */
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PullUp = PIN_PU_ON;
    GPIO_Init(GPIO_PORT_D, GPIO_PIN_02, &stcGpioInit);
    /* Configure USART RX pin. */
    GPIO_SetFunc(GPIO_PORT_D, GPIO_PIN_02, GPIO_FUNC_35);
    /* Initialize UART. */
    USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv = USART_CLK_DIV16;
    stcUartInit.u32Baudrate = 100000UL;
    stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32Parity = USART_PARITY_EVEN;
    stcUartInit.u32StopBit = USART_STOPBIT_2BIT;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    if (LL_OK != USART_UART_Init(CM_USART8, &stcUartInit, NULL))
        return RT_EIO;
    /* Register RX error IRQ handler && configure NVIC. */
    stcIrqSignConfig.enIRQn = INT104_IRQn;
    stcIrqSignConfig.enIntSrc = INT_SRC_USART8_EI;
    stcIrqSignConfig.pfnCallback = &USART_RxError_IrqCallback;
    INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_13);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
    /* Register RX full IRQ handler && configure NVIC. */
    stcIrqSignConfig.enIRQn = INT105_IRQn;
    stcIrqSignConfig.enIntSrc = INT_SRC_USART8_RI;
    stcIrqSignConfig.pfnCallback = &USART_RxFull_IrqCallback;
    INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_13);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
    /* MCU Peripheral registers write protected */
    LL_PERIPH_WP(LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_EFM | LL_PERIPH_SRAM);
    /* Enable RX function */
    USART_FuncCmd(CM_USART8, USART_RX, ENABLE);

    return RT_EOK;
}

rt_err_t rc_init(rc_dev_t dev)
{
    /* Starts TimerA. */
    TMRA_Start(CM_TMRA_12);
    /* Configures the capture condition. */
    TMRA_HWCaptureCondCmd(CM_TMRA_12, TMRA_CH4, TMRA_CAPT_COND_PWM_RISING, ENABLE);
    /* Enable the specified interrupts of TimerA. */
    TMRA_IntCmd(CM_TMRA_12, TMRA_INT_CMP_CH4, ENABLE);
    /* open interrupt */
    USART_FuncCmd(CM_USART8, USART_INT_RX, ENABLE);

    return RT_EOK;
}

static rt_err_t rc_control(rc_dev_t rc, int cmd, void* arg)
{
    switch (cmd) {
    case RC_CMD_CHECK_UPDATE: {
        uint8_t updated = 0;

        if (rc->config.protocol == RC_PROTOCOL_SBUS) {
            updated = sbus_data_ready(&sbus_decoder);
        } else if (rc->config.protocol == RC_PROTOCOL_PPM) {
            updated = ppm_data_ready(&ppm_decoder);
        }

        *(uint8_t*)arg = updated;
    } break;

    default:
        break;
    }

    return RT_EOK;
}

static rt_uint16_t rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val)
{
    uint16_t* index = chan_val;
    rt_uint16_t rb = 0;

    if (rc->config.protocol == RC_PROTOCOL_SBUS) {
        if (sbus_data_ready(&sbus_decoder) == 0) {
            /* no data received, just return */
            return 0;
        }

        sbus_lock(&sbus_decoder);

        for (uint8_t i = 0; i < min(rc->config.channel_num, sbus_decoder.rc_count); i++) {
            *(index++) = sbus_decoder.sbus_val[i];
            rb += 2;
        }
        sbus_data_clear(&sbus_decoder);

        sbus_unlock(&sbus_decoder);
    } else if (rc->config.protocol == RC_PROTOCOL_PPM) {
        if (ppm_data_ready(&ppm_decoder) == 0) {
            /* no data received, just return */
            return 0;
        }

        ppm_lock(&ppm_decoder);

        for (uint8_t i = 0; i < min(rc->config.channel_num, ppm_decoder.total_chan); i++) {
            if (chan_mask & (1 << i)) {
                *(index++) = ppm_decoder.ppm_val[i];
                rb += 2;
            }
        }
        ppm_data_clear(&ppm_decoder);

        ppm_unlock(&ppm_decoder);
    }

    return rb;
}

const static struct rc_ops rc_ops = {
    .rc_init = rc_init,
    .rc_config = NULL,
    .rc_control = rc_control,
    .rc_read = rc_read,
};

static struct rc_device rc_dev = {
    .config = RC_CONFIG_DEFAULT,
    .ops = &rc_ops,
};

rt_err_t drv_rc_init(void)
{
    /* init ppm decoder */
    RT_TRY(ppm_decoder_init(&ppm_decoder, PPM_DECODER_FREQUENCY));
    /* init ppm driver */
    RT_TRY(ppm_lowlevel_init());
    /* init sbus decoder */
    RT_TRY(sbus_decoder_init(&sbus_decoder));
    /* init sbus driver */
    RT_TRY(sbus_lowlevel_init());

    RT_CHECK(hal_rc_register(&rc_dev, "rc", RT_DEVICE_FLAG_RDWR, NULL));

    return RT_EOK;
}