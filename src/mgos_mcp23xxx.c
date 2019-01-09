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

#include "mgos_mcp23xxx_internal.h"

bool mgos_mcp23xxx_print_state(struct mgos_mcp23xxx *dev) {
  uint8_t  n;
  uint16_t _gpio = 0, _iodir = 0, _gpinten = 0;
  char     gpio[17], iodir[17], gpinten[17];

  if (!dev) {
    return false;
  }
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPIO * dev->_w, dev->_w, (uint8_t *)&_gpio)) {
    return false;
  }
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IODIR * dev->_w, dev->_w, (uint8_t *)&_iodir)) {
    return false;
  }
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN * dev->_w, dev->_w, (uint8_t *)&_gpinten)) {
    return false;
  }
  for (n = 0; n < dev->num_gpios; n++) {
    gpio[dev->num_gpios - n - 1]    = (_gpio & (1 << n)) ? '1' : '0';
    iodir[dev->num_gpios - n - 1]   = (_iodir & (1 << n)) ? 'I' : 'O';
    gpinten[dev->num_gpios - n - 1] = (_gpinten & (1 << n)) ? '+' : '-';
  }
  gpio[dev->num_gpios] = iodir[dev->num_gpios] = gpinten[dev->num_gpios] = 0;
  if (dev->num_gpios == 8) {
    LOG(LL_INFO, ("gpio:    0x%02x %s", _gpio, gpio));
    LOG(LL_INFO, ("iodir:   0x%02x %s", _iodir, iodir));
    LOG(LL_INFO, ("gpinten: 0x%02x %s", _gpinten, gpinten));
  } else{
    LOG(LL_INFO, ("gpio:    0x%04x %s", _gpio, gpio));
    LOG(LL_INFO, ("iodir:   0x%04x %s", _iodir, iodir));
    LOG(LL_INFO, ("gpinten: 0x%04x %s", _gpinten, gpinten));
  }
  return true;
}

static bool mgos_mcp23xxx_read(struct mgos_mcp23xxx *dev) {
  uint16_t _gpio;
  uint16_t _val;

  if (!dev) {
    return false;
  }
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPIO * dev->_w, dev->_w, (uint8_t *)&_gpio)) {
    return false;
  }
  dev->_gpio = _gpio;

  // Flush INTF and INTCAP to ensure interrupts keep firing.
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_INTF * dev->_w, dev->_w, (uint8_t *)&_val)) {
    return false;
  }
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_INTCAP * dev->_w, dev->_w, (uint8_t *)&_val)) {
    return false;
  }
  return true;
}

static void mgos_mcp23xxx_irq(int pin, void *arg) {
  struct mgos_mcp23xxx *dev = (struct mgos_mcp23xxx *)arg;
  uint8_t  n;
  uint16_t _intf, _intcap, prev_gpio, this_gpio;

  // LOG(LL_DEBUG, ("Interrupt received on %d", pin));
  if (!dev) {
    return;
  }
  if (dev->int_gpio != pin) {
    return;
  }

  // Read INTF to see which pin(s) triggered the interrupt
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_INTF * dev->_w, dev->_w, (uint8_t *)&_intf)) {
    return;
  }
  // Read INTCAP to see the pin state at interrup ttime
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_INTCAP * dev->_w, dev->_w, (uint8_t *)&_intcap)) {
    return;
  }
  prev_gpio = dev->_gpio;
  this_gpio = _intcap;
  // LOG(LL_DEBUG, ("INTF=0x%04x INTCAP=0x%04x Previous GPIO=0x%04x", _intf, _intcap, prev_gpio));

  for (n = 0; n < dev->num_gpios; n++) {
    bool this_bit  = this_gpio & (1 << n);
    bool prev_bit  = prev_gpio & (1 << n);
    bool will_call = false;

    // INTF doesn't signal pin N caused the interrupt
    if (!(_intf && (1 << n))) {
      continue;
    }

    switch (dev->cb[n].mode) {
    case MGOS_GPIO_INT_LEVEL_LO:
      will_call = !this_bit;
      break;

    case MGOS_GPIO_INT_LEVEL_HI:
      will_call = this_bit;
      break;

    case MGOS_GPIO_INT_EDGE_POS:
      will_call = this_bit && !prev_bit;
      break;

    case MGOS_GPIO_INT_EDGE_NEG:
      will_call = !this_bit && prev_bit;
      break;

    case MGOS_GPIO_INT_EDGE_ANY:
      will_call = (this_bit != prev_bit);
      break;

    default:
      will_call = false;
    }
    // LOG(LL_DEBUG, ("GPIO[%u] this_bit=%u will_call=%u", n, this_bit, will_call));
    if (will_call && dev->cb[n].enabled) {
      dev->cb[n].firing = true;
      dev->cb[n].last   = mg_time();
      if (dev->cb[n].fn) {
        // LOG(LL_DEBUG, ("GPIO[%u] callback issued", n));
        dev->cb[n].fn(n, dev->cb[n].fn_arg);
      }
      dev->cb[n].firing = false;
    }
  }

  // Re-enable interrupts
  mgos_gpio_clear_int(dev->int_gpio);
  mgos_gpio_enable_int(dev->int_gpio);
  mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IOCON, 1, 1, 0);
  mgos_mcp23xxx_read(dev);

  return;
}

static struct mgos_mcp23xxx *mgos_mcp23xxx_create(struct mgos_i2c *i2c, uint8_t i2caddr, int int_gpio, uint8_t num_gpios) {
  struct mgos_mcp23xxx *dev = NULL;

  if (!i2c) {
    return NULL;
  }

  if (num_gpios != 8 && num_gpios != 16) {
    return NULL;
  }

  dev = calloc(1, sizeof(struct mgos_mcp23xxx));
  if (!dev) {
    return NULL;
  }

  memset(dev, 0, sizeof(struct mgos_mcp23xxx));
  dev->i2caddr   = i2caddr;
  dev->i2c       = i2c;
  dev->int_gpio  = int_gpio;
  dev->num_gpios = num_gpios;
  dev->_w        = dev->num_gpios == 16 ? 2 : 1;

  // Read and print current IO state
  mgos_mcp23xxx_read(dev);
  if (!mgos_mcp23xxx_print_state(dev)) {
    LOG(LL_ERROR, ("Could not read current state"));
    free(dev);
    return NULL;
  }

  // Install interrupt handler, if GPIO pin was specified.
  if (dev->int_gpio != -1) {
    LOG(LL_INFO, ("Installing interrupt handler on GPIO %d", dev->int_gpio));
    mgos_gpio_set_mode(dev->int_gpio, MGOS_GPIO_MODE_INPUT);
    mgos_gpio_set_pull(dev->int_gpio, MGOS_GPIO_PULL_UP);
    mgos_gpio_set_int_handler(dev->int_gpio, MGOS_GPIO_INT_EDGE_NEG, mgos_mcp23xxx_irq, dev);
    mgos_gpio_clear_int(dev->int_gpio);
    mgos_gpio_enable_int(dev->int_gpio);
  }

  // In all cases, set IOCON.MIRROR=1, IOCON.INTPOL=0
  if (!mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IOCON, 1, 1, 0)) {
    return false;
  }
  if (!mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IOCON, 6, 1, 1)) {
    return false;
  }

  LOG(LL_INFO, ("MCP230%s initialized at I2C 0x%02x", (dev->num_gpios == 8 ? "08" : "17"), dev->i2caddr));
  return dev;
}

struct mgos_mcp23xxx *mgos_mcp23017_create(struct mgos_i2c *i2c, uint8_t i2caddr, int int_gpio) {
  // Set up intA/intB mirroring.
  return mgos_mcp23xxx_create(i2c, i2caddr, int_gpio, 16);
}

struct mgos_mcp23xxx *mgos_mcp23008_create(struct mgos_i2c *i2c, uint8_t i2caddr, int int_gpio) {
  return mgos_mcp23xxx_create(i2c, i2caddr, int_gpio, 8);
}

bool mgos_mcp23xxx_destroy(struct mgos_mcp23xxx **dev) {
  if (!*dev) {
    return false;
  }

  // Disable and remove interrupts
  if ((*dev)->int_gpio != -1) {
    uint16_t _val;
    LOG(LL_INFO, ("Removing interrupt handler on GPIO %d", (*dev)->int_gpio));

    // Read INTF and INTCAP to clear any pending interrupt
    mgos_i2c_read_reg_n((*dev)->i2c, (*dev)->i2caddr, MGOS_MCP23XXX_REG_INTF * (*dev)->_w, (*dev)->_w, (uint8_t *)&_val);
    mgos_i2c_read_reg_n((*dev)->i2c, (*dev)->i2caddr, MGOS_MCP23XXX_REG_INTCAP * (*dev)->_w, (*dev)->_w, (uint8_t *)&_val);

    // Remove interrupt handler on MCU
    mgos_gpio_disable_int((*dev)->int_gpio);
    mgos_gpio_clear_int((*dev)->int_gpio);
    mgos_gpio_remove_int_handler((*dev)->int_gpio, NULL, NULL);
  }

  free(*dev);
  *dev = NULL;
  return true;
}

bool mgos_mcp23xxx_gpio_set_mode(struct mgos_mcp23xxx *dev, int pin, enum mgos_gpio_mode mode) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  // IODIR=1
  if (mode == MGOS_GPIO_MODE_INPUT) {
    if (dev->num_gpios == 8) {
      if (!mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IODIR, pin, 1, 1)) {
        return false;
      }
    } else {
      if (!mgos_i2c_setbits_reg_w(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IODIR * 2, pin, 1, 1)) {
        return false;
      }
    }
  } else {
    if (!mgos_mcp23xxx_gpio_set_pull(dev, pin, MGOS_GPIO_PULL_NONE)) {
      return false;
    }

    // IODIR=1, INTCON=0, GPINTEN=0
    if (dev->num_gpios == 8) {
      if (!mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IODIR, pin, 1, 0)) {
        return false;
      }
      if (!mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_INTCON, pin, 1, 0)) {
        return false;
      }
      if (!mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN, pin, 1, 0)) {
        return false;
      }
    } else {
      if (!mgos_i2c_setbits_reg_w(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IODIR * 2, pin, 1, 0)) {
        return false;
      }
      if (!mgos_i2c_setbits_reg_w(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_INTCON * 2, pin, 1, 0)) {
        return false;
      }
      if (!mgos_i2c_setbits_reg_w(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN * 2, pin, 1, 0)) {
        return false;
      }
    }
  }

  return true;
}

bool mgos_mcp23xxx_gpio_set_pull(struct mgos_mcp23xxx *dev, int pin, enum mgos_gpio_pull_type pull) {
  uint16_t gppu;

  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  switch (pull) {
  case MGOS_GPIO_PULL_DOWN:
    return false;

  case MGOS_GPIO_PULL_UP:
    gppu = 1;
    break;

  default:   // NONE
    gppu = 0;
    break;
  }

  if (dev->num_gpios == 8) {
    if (!mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPPU, pin, 1, gppu)) {
      return false;
    }
  } else {
    if (!mgos_i2c_setbits_reg_w(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPPU * 2, pin, 1, gppu)) {
      return false;
    }
  }
  return true;
}

bool mgos_mcp23xxx_gpio_setup_input(struct mgos_mcp23xxx *dev, int pin, enum mgos_gpio_pull_type pull_type) {
  if (!mgos_mcp23xxx_gpio_set_pull(dev, pin, pull_type)) {
    return false;
  }

  return mgos_mcp23xxx_gpio_set_mode(dev, pin, MGOS_GPIO_MODE_INPUT);
}

bool mgos_mcp23xxx_gpio_setup_output(struct mgos_mcp23xxx *dev, int pin, bool level) {
  mgos_mcp23xxx_gpio_write(dev, pin, level);
  return mgos_mcp23xxx_gpio_set_mode(dev, pin, MGOS_GPIO_MODE_OUTPUT);
}

bool mgos_mcp23xxx_gpio_read(struct mgos_mcp23xxx *dev, int pin) {
  uint16_t _gpio;

  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPIO * dev->_w, dev->_w, (uint8_t *)&_gpio)) {
    return false;
  }
  dev->_gpio = _gpio;
  return (_gpio & (1 << pin)) > 0;
}

void mgos_mcp23xxx_gpio_write(struct mgos_mcp23xxx *dev, int pin, bool level) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return;
  }

  if (dev->num_gpios == 8) {
    if (!mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPIO, pin, 1, level)) {
      return;
    }
  } else {
    if (!mgos_i2c_setbits_reg_w(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPIO * 2, pin, 1, level)) {
      return;
    }
  }
  if (level) {
    dev->_gpio |= (1 << pin);
  } else {
    dev->_gpio &= ~(1 << pin);
  }
  return;
}

bool mgos_mcp23xxx_gpio_toggle(struct mgos_mcp23xxx *dev, int pin) {
  bool level;

  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  level = mgos_mcp23xxx_gpio_read(dev, pin);
  mgos_mcp23xxx_gpio_write(dev, pin, !level);
  return !level;
}

bool mgos_mcp23xxx_gpio_set_int_handler(struct mgos_mcp23xxx *dev, int pin, enum mgos_gpio_int_mode mode, mgos_gpio_int_handler_f cb, void *arg) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  dev->cb[pin].fn      = cb;
  dev->cb[pin].fn_arg  = arg;
  dev->cb[pin].mode    = mode;
  dev->cb[pin].enabled = true;
  return mgos_mcp23xxx_gpio_enable_int(dev, pin);
}

bool mgos_mcp23xxx_gpio_enable_int(struct mgos_mcp23xxx *dev, int pin) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  if (dev->num_gpios == 8) {
    if (!mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN, pin, 1, true)) {
      return false;
    }
  } else {
    if (!mgos_i2c_setbits_reg_w(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN * 2, pin, 1, true)) {
      return false;
    }
  }
  dev->cb[pin].enabled = true;
  return true;
}

bool mgos_mcp23xxx_gpio_disable_int(struct mgos_mcp23xxx *dev, int pin) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  if (dev->num_gpios == 8) {
    if (!mgos_i2c_setbits_reg_b(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN, pin, 1, false)) {
      return false;
    }
  } else {
    if (!mgos_i2c_setbits_reg_w(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN * 2, pin, 1, false)) {
      return false;
    }
  }
  dev->cb[pin].enabled = false;
  return true;
}

void mgos_mcp23xxx_gpio_clear_int(struct mgos_mcp23xxx *dev, int pin) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return;
  }
  dev->cb[pin].firing = false;
  dev->cb[pin].last   = 0.f;
  return;
}

void mgos_mcp23xxx_gpio_remove_int_handler(struct mgos_mcp23xxx *dev, int pin, mgos_gpio_int_handler_f *old_cb, void **old_arg) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return;
  }

  mgos_mcp23xxx_gpio_disable_int(dev, pin);
  dev->cb[pin].fn     = NULL;
  dev->cb[pin].fn_arg = NULL;
  dev->cb[pin].firing = false;
  return;

  (void)old_cb;
  (void)old_arg;
}

bool mgos_mcp23xxx_gpio_set_button_handler(struct mgos_mcp23xxx *dev, int pin, enum mgos_gpio_pull_type pull_type, enum mgos_gpio_int_mode int_mode, int debounce_ms, mgos_gpio_int_handler_f cb, void *arg) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  if (!mgos_mcp23xxx_gpio_setup_input(dev, pin, pull_type)) {
    return false;
  }

  dev->cb[pin].debounce_ms = debounce_ms;
  return mgos_mcp23xxx_gpio_set_int_handler(dev, pin, int_mode, cb, arg);
}

// Mongoose OS library initialization
bool mgos_mcp23xxx_init(void) {
  return true;
}
