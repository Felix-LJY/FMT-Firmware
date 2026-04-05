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

#include "drv_gpio.h"
#include "hal/pin/pin.h"

#define PIN_STPORT(pin) ((uint32_t)(GPIO_BASE + (0x400u * PIN_PORT(pin))))
#define PIN_STPIN(pin)  ((uint16_t)(1u << PIN_NO(pin)))

static struct pin_device pin_device;

static void wl32_pin_write(pin_dev_t dev, rt_base_t pin, rt_base_t value)
{
    uint8_t  gpio_port;
    uint16_t gpio_pin;

    if (pin < PIN_MAX_NUM)
    {
        gpio_port = GPIO_PORT(pin);
        gpio_pin  = GPIO_PIN(pin);
        if (PIN_LOW == value)
        {
            GPIO_ResetPins(gpio_port, gpio_pin);
        }
        else
        {
            GPIO_SetPins(gpio_port, gpio_pin);
        }
    }
}

static int wl32_pin_read(pin_dev_t dev, rt_base_t pin)
{
    uint8_t  gpio_port;
    uint16_t gpio_pin;
    int value = PIN_LOW;

    if (pin < PIN_MAX_NUM)
    {
        gpio_port = GPIO_PORT(pin);
        gpio_pin  = GPIO_PIN(pin);
        if (PIN_RESET == GPIO_ReadInputPins(gpio_port, gpio_pin))
        {
            value = PIN_LOW;
        }
        else
        {
            value = PIN_HIGH;
        }
    }
    else
    {
        return -RT_EINVAL;
    }

    return value;
}

static void wl32_pin_mode(pin_dev_t dev, rt_base_t pin, rt_base_t mode, rt_base_t otype)
{
    
    stc_gpio_init_t stcGpioInit;

    LL_PERIPH_WE(LL_PERIPH_GPIO);

    if (pin >= PIN_MAX_NUM)
    {
        return;
    }

    GPIO_StructInit(&stcGpioInit);
    switch (mode)
    {
    case PIN_MODE_OUTPUT:
        stcGpioInit.u16PinDir        = PIN_DIR_OUT;
        stcGpioInit.u16PinOutputType = PIN_OUT_TYPE_CMOS;
        break;
    case PIN_MODE_INPUT:
        stcGpioInit.u16PinDir   = PIN_DIR_IN;
        break;
    case PIN_MODE_INPUT_PULLUP:
        stcGpioInit.u16PinDir   = PIN_DIR_IN;
        stcGpioInit.u16PullUp   = PIN_PU_ON;
        break;
    case PIN_MODE_INPUT_PULLDOWN:
        stcGpioInit.u16PinDir   = PIN_DIR_IN;
        stcGpioInit.u16PullUp   = PIN_PU_OFF;
#if defined (WL32F448) || defined (WL32F472)
        stcGpioInit.u16PullDown = PIN_PD_ON;
#endif
        break;
    case PIN_MODE_OUTPUT_OD:
        stcGpioInit.u16PinDir        = PIN_DIR_OUT;
        stcGpioInit.u16PinOutputType = PIN_OUT_TYPE_NMOS;
        break;
    default:
        break;
    }
    GPIO_Init(GPIO_PORT(pin), GPIO_PIN(pin), &stcGpioInit);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
}

const static struct pin_ops pin_ops = {
    .pin_mode = wl32_pin_mode,
    .pin_write = wl32_pin_write,
    .pin_read = wl32_pin_read,
};

rt_err_t drv_gpio_init(void)
{
    GPIO_REG_Unlock();
    pin_device.ops = &pin_ops;

    return hal_pin_register(&pin_device, "pin", RT_DEVICE_FLAG_RDWR, RT_NULL);
}


