/******************************************************************************
 * Copyright 2023 The Firmament Authors. All Rights Reserved.
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
#include "drv_adc.h"
#include "hal/adc/adc.h"

#define ADC_CONVERSION_TIMEOUT_MS 2

static struct adc_device adc0;
static struct rt_completion convert_cplt;

void adc3_eoc_handler(void)
{
    ADC_ClearStatus(CM_ADC3, ADC_FLAG_EOCA);
    /* inform the completion of adc convertion */
    rt_completion_done(&convert_cplt);
}

static rt_err_t adc_hw_init(void)
{
    stc_adc_init_t stcAdcInit;
    stc_gpio_init_t stcGpioInit;
    stc_irq_signin_config_t stcIrqSignConfig;

    LL_PERIPH_WE(LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU);

    /* 1. Enable ADC peripheral clock. */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC3, ENABLE);

    /* 2. Modify the default value depends on the application. Not needed here. */
    (void)ADC_StructInit(&stcAdcInit);

    /* 3. Initializes ADC. */
    (void)ADC_Init(CM_ADC3, &stcAdcInit);

    /* 4. ADC channel configuration. */
    /* 4.1 Set the ADC pin to analog input mode. */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_F, GPIO_PIN_03|GPIO_PIN_04|GPIO_PIN_05|GPIO_PIN_06, &stcGpioInit);

    /* 5. Set sampling time if needed. */
    ADC_SetSampleTime(CM_ADC3, ADC_CH4, 0x40U);
    ADC_SetSampleTime(CM_ADC3, ADC_CH15, 0x40U);
    ADC_SetSampleTime(CM_ADC3, ADC_CH14, 0x40U);
    ADC_SetSampleTime(CM_ADC3, ADC_CH9, 0x40U);

    /* 6. Conversion data average calculation function, if needed.
          Call ADC_ConvDataAverageChCmd() again to enable more average channels if needed. */
    ADC_ConvDataAverageConfig(CM_ADC3, ADC_AVG_CNT8);
    ADC_ConvDataAverageChCmd(CM_ADC3, ADC_CH4, ENABLE);
    ADC_ConvDataAverageChCmd(CM_ADC3, ADC_CH15, ENABLE);
    ADC_ConvDataAverageChCmd(CM_ADC3, ADC_CH14, ENABLE);
    ADC_ConvDataAverageChCmd(CM_ADC3, ADC_CH9, ENABLE);

    /* Register EOC irq */
    stcIrqSignConfig.enIRQn  = INT031_IRQn;
    stcIrqSignConfig.enIntSrc = INT_SRC_ADC3_EOCA;
    stcIrqSignConfig.pfnCallback = adc3_eoc_handler;
    INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_03);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
    
    LL_PERIPH_WP(LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU);
    return RT_EOK;
}

static rt_err_t enable(adc_dev_t adc_dev, uint8_t enable)
{
    if (enable == ADC_CMD_ENABLE) {
        ADC_ChCmd(CM_ADC3, ADC_SEQ_A, ADC_CH4, ENABLE);
        ADC_ChCmd(CM_ADC3, ADC_SEQ_A, ADC_CH15, ENABLE);
        ADC_ChCmd(CM_ADC3, ADC_SEQ_A, ADC_CH14, ENABLE);
        ADC_ChCmd(CM_ADC3, ADC_SEQ_A, ADC_CH9, ENABLE);
    } else if (enable == ADC_CMD_DISABLE) {
        ADC_ChCmd(CM_ADC3, ADC_SEQ_A, ADC_CH4, DISABLE);
        ADC_ChCmd(CM_ADC3, ADC_SEQ_A, ADC_CH15, DISABLE);
        ADC_ChCmd(CM_ADC3, ADC_SEQ_A, ADC_CH14, DISABLE);
        ADC_ChCmd(CM_ADC3, ADC_SEQ_A, ADC_CH9, DISABLE);
    } else {
        return RT_EINVAL;
    }

    return RT_EOK;
}

static rt_err_t measure(adc_dev_t adc_dev, uint32_t channel, uint32_t* mVolt)
{
    uint32_t adc_channel;
    uint16_t adcData;

    switch (channel) {
    case 0: /* BAT1_CURRENT */
        adc_channel = ADC_CH4;
        break;
    case 1: /* BAT1_VOLTAGE */
        adc_channel = ADC_CH15;
        break;
    case 2: /* BAT2_CURRENT */
        adc_channel = ADC_CH14;
        break;
    case 3: /* BAT2_VOLTAGE */
        adc_channel = ADC_CH9;
        break;
    default:
        return RT_EINVAL;
    }

    ADC_ChCmd(CM_ADC3, ADC_SEQ_A, adc_channel, ENABLE);
    ADC_IntCmd(CM_ADC3, ADC_INT_EOCA, ENABLE);
    ADC_Start(CM_ADC3);

    if (rt_completion_wait(&convert_cplt, TICKS_FROM_MS(ADC_CONVERSION_TIMEOUT_MS)) != RT_EOK) {
        return RT_ERROR;
    }

    /* Get any ADC value of sequence A channel that needed. */
    adcData = ADC_GetValue(CM_ADC3, adc_channel) + 1;
    *mVolt = adcData * 3300 / 4096;
    ADC_ChCmd(CM_ADC3, ADC_SEQ_A, adc_channel, DISABLE);
    return RT_EOK;
}

/* usart driver operations */
static const struct adc_ops _adc_ops = {
    .enable = enable,
    .measure = measure
};

rt_err_t drv_adc_init(void)
{
    RT_CHECK(adc_hw_init());

    rt_completion_init(&convert_cplt);

    adc0.ops = &_adc_ops;

    return hal_adc_register(&adc0, "adc0", RT_DEVICE_FLAG_RDONLY, RT_NULL);
}


