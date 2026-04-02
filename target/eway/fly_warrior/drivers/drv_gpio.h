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

#ifndef DRV_GPIO_H__
#define DRV_GPIO_H__

#include <firmament.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_PIN_INDEX(pin)             ((uint8_t)((pin) & 0x0F))
#define PIN_NUM(port, pin)              (((((port) & 0x0F) << 4) | ((pin) & 0x0F)))
#define GPIO_PORT(pin)                  ((uint8_t)(((pin) >> 4) & 0x0F))
#define GPIO_PIN(pin)                   ((uint16_t)(0x01U << GPIO_PIN_INDEX(pin)))

#define PIN_MAX_NUM                     (GPIO_PORT_I * 16)

rt_err_t drv_gpio_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_GPIO_H__ */
