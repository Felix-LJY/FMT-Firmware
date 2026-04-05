/******************************************************************************
 * Copyright 2020-2021 The Firmament Authors. All Rights Reserved.
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
#include "drv_i2c.h"
#include "hal/i2c/i2c.h"

// #define DRV_DBG(...) printf(__VA_ARGS__)
#define DRV_DBG(...)

/* We want to ensure the real-time performace, so the i2c timeout here is
 * relatively short */
#define I2C_TIMEOUT_US    (10000)
#define DRV_I2C_TIMEOUT   (0x40000UL)

#define I2C_UNIT                        (CM_I2C1)
#define I2C_FCG_USE                     (FCG1_PERIPH_I2C1)
#define I2C_BAUDRATE                    (400000UL)


/* Define port and pin for SDA and SCL */
#define I2C_SCL_PORT                    (GPIO_PORT_I)
#define I2C_SCL_PIN                     (GPIO_PIN_11)
#define I2C_SDA_PORT                    (GPIO_PORT_I)
#define I2C_SDA_PIN                     (GPIO_PIN_10)
#define I2C_GPIO_SCL_FUNC               (GPIO_FUNC_49)
#define I2C_GPIO_SDA_FUNC               (GPIO_FUNC_48)

struct wl32_i2c_bus {
    struct rt_i2c_bus parent;
    uint32_t i2c_periph;
};

static int32_t drv_I2C_Master_Receive(uint16_t u16DevAddr, uint8_t au8Data[], uint32_t u32Size, uint32_t u32Timeout)
{
    int32_t i32Ret;

    I2C_Cmd(CM_I2C1, ENABLE);
    I2C_SWResetCmd(CM_I2C1, ENABLE);
    I2C_SWResetCmd(CM_I2C1, DISABLE);
    i32Ret = I2C_Start(CM_I2C1, u32Timeout);
    if (LL_OK == i32Ret) {
        if (1UL == u32Size) {
            I2C_AckConfig(CM_I2C1, I2C_NACK);
        }

        i32Ret = I2C_TransAddr(CM_I2C1, u16DevAddr, I2C_DIR_RX, u32Timeout);

        if (LL_OK == i32Ret) {
            i32Ret = I2C_MasterReceiveDataAndStop(CM_I2C1, au8Data, u32Size, u32Timeout);
        }

        I2C_AckConfig(CM_I2C1, I2C_ACK);
    }

    if (LL_OK != i32Ret) {
        (void)I2C_Stop(CM_I2C1, u32Timeout);
    }
    I2C_Cmd(CM_I2C1, DISABLE);
    return i32Ret;
}
static int32_t drv_I2C_Master_Transmit(uint16_t u16DevAddr, const uint8_t au8Data[], uint32_t u32Size, uint32_t u32Timeout)
{
    int32_t i32Ret;
    I2C_Cmd(CM_I2C1, ENABLE);

    I2C_SWResetCmd(CM_I2C1, ENABLE);
    I2C_SWResetCmd(CM_I2C1, DISABLE);
    i32Ret = I2C_Start(CM_I2C1, u32Timeout);
    if (LL_OK == i32Ret) {
        i32Ret = I2C_TransAddr(CM_I2C1, u16DevAddr, I2C_DIR_TX, u32Timeout);
        if (LL_OK == i32Ret) {
            i32Ret = I2C_TransData(CM_I2C1, au8Data, u32Size, u32Timeout);
        }
    }

    (void)I2C_Stop(CM_I2C1, u32Timeout);
    I2C_Cmd(CM_I2C1, DISABLE);

    return i32Ret;
}

static rt_size_t i2c_master_transfer(struct rt_i2c_bus* bus, rt_uint16_t slave_addr,  struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg* msg;
    uint32_t msg_idx = 0;

    for (msg_idx = 0; msg_idx < num; msg_idx++) {
        msg = &msgs[msg_idx];
        uint16_t nbytes = msg->len;

        if (msg->flags & RT_I2C_RD) {
            /* start/restart read operation */
            drv_I2C_Master_Receive(slave_addr,msg->buf,nbytes,DRV_I2C_TIMEOUT);
        } else {
            drv_I2C_Master_Transmit(slave_addr,msg->buf,nbytes,DRV_I2C_TIMEOUT);
        }
    }
    return msg_idx;
}
static const struct rt_i2c_bus_device_ops i2c_bus_ops = {
    i2c_master_transfer,
    RT_NULL,
    RT_NULL
};

/* i2c bus instances */
static struct wl32_i2c_bus wl32_i2c0 = {
    .parent.ops = &i2c_bus_ops,
    .i2c_periph = (uint32_t)I2C_UNIT
};

/* i2c device instances */
static struct rt_i2c_device i2c0_dev0 = {
    .slave_addr = 0x45, /* AW2023 7 bit address */
    .flags = 0
};

static struct rt_i2c_device i2c0_dev1 = {
    .slave_addr = 0x28, /* MS4525 7 bit address */
    .flags = 0
};

static int32_t Master_Initialize(void)
{
    int32_t i32Ret;
    stc_i2c_init_t stcI2cInit;
    float32_t fErr;

    (void)I2C_DeInit(CM_I2C1);

    (void)I2C_StructInit(&stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV4;
    stcI2cInit.u32Baudrate = I2C_BAUDRATE;
    stcI2cInit.u32SclTime = 3UL;
    i32Ret = I2C_Init(CM_I2C1, &stcI2cInit, &fErr);

    I2C_BusWaitCmd(CM_I2C1, ENABLE);

    return i32Ret;
}
void i2c_hw_init(void)
{

    LL_PERIPH_WE(LL_PERIPH_GPIO | LL_PERIPH_FCG);
    /* Initialize I2C port*/
    GPIO_SetFunc(I2C_SCL_PORT, I2C_SCL_PIN, I2C_GPIO_SCL_FUNC);
    GPIO_SetFunc(I2C_SDA_PORT, I2C_SDA_PIN, I2C_GPIO_SDA_FUNC);
    /* Enable I2C Peripheral*/
    FCG_Fcg1PeriphClockCmd(I2C_FCG_USE, ENABLE);
    /* Initialize I2C peripheral and enable function*/
    if (LL_OK != Master_Initialize()) {
        /* Initialize error*/
        for (;;) {
        }
    }
    LL_PERIPH_WP(LL_PERIPH_GPIO | LL_PERIPH_FCG);
}
rt_err_t drv_i2c_init(void)
{
    /* i2c low-level initialization */
    i2c_hw_init();

    /* register i2c bus */
    RT_TRY(rt_i2c_bus_device_register(&wl32_i2c0.parent, "i2c0"));

    /* attach i2c devices */
    RT_TRY(rt_i2c_bus_attach_device(&i2c0_dev0, "i2c0_dev0", "i2c0", RT_NULL));

    /* attach i2c devices */
    RT_TRY(rt_i2c_bus_attach_device(&i2c0_dev1, "i2c0_dev1", "i2c0", RT_NULL));

    return RT_EOK;
}
