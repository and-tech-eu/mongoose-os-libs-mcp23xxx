/*
 * Copyright 2019 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include "mgos_mcp23xxx.h"

#ifdef __cplusplus
extern "C" {
#endif

// MCP23xAA I2C address (x=S: SPI, x=0: I2C; AA=08: 8 ports, AA=17: 16 ports)
#define MGOS_MCP23XXX_I2C_ADDR    (0x20)

// MCP23XXX Registers, bank=0 mode (poweron default)
// For MCP23X08, Registers are A=reg and 8bit
// For MCP23X17, Registers are A=reg*2 and B=reg*2+1 and 16bit
#define MGOS_MCP23XXX_REG_IODIR      (0x00)
#define MGOS_MCP23XXX_REG_IPOL       (0x01)
#define MGOS_MCP23XXX_REG_GPINTEN    (0x02)
#define MGOS_MCP23XXX_REG_DEFVAL     (0x03)
#define MGOS_MCP23XXX_REG_INTCON     (0x04)
#define MGOS_MCP23XXX_REG_IOCON      (0x05)
#define MGOS_MCP23XXX_REG_GPPU       (0x06)
#define MGOS_MCP23XXX_REG_INTF       (0x07)
#define MGOS_MCP23XXX_REG_INTCAP     (0x08)
#define MGOS_MCP23XXX_REG_GPIO       (0x09)
#define MGOS_MCP23XXX_REG_OLAT       (0x0A)

struct mgos_mcp23xxx_cb {
  mgos_gpio_int_handler_f fn;
  void *                  fn_arg;
  double                  last;
  int                     debounce_ms;
  bool                    enabled;
  bool                    firing;
  enum mgos_gpio_int_mode mode;
};

struct mgos_mcp23xxx {
  struct mgos_i2c *       i2c;
  uint8_t                 i2caddr;

  uint8_t                 _w;        // Register width, MCP23x08: 1; MCP23x17: 2;
  uint16_t                _gpio;     // Previous known state of GPIO pins
  uint8_t                 num_gpios; // MCP23x08: 8; MCP23x17: 16
  int                     int_gpio;  // Interrupt pin from device (MCP23X17 always enables int mirroring)

  struct mgos_mcp23xxx_cb cb[16];
};

/* Mongoose OS initializer */
bool mgos_mcp23xxx_i2c_init(void);

#ifdef __cplusplus
}
#endif
