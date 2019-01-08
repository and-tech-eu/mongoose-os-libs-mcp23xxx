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

void mgos_mcp23xxx_print_state(struct mgos_mcp23xxx *dev) {
  uint8_t n;
  char    s[17], i[17];

  if (!dev) {
    return;
  }
  for (n = 0; n < dev->num_gpios; n++) {
    s[dev->num_gpios - n - 1] = (dev->_state & (1 << n)) ? '1' : '0';
    i[dev->num_gpios - n - 1] = (dev->_io & (1 << n)) ? 'I' : 'O';
  }
  s[dev->num_gpios] = i[dev->num_gpios] = 0;
  if (dev->num_gpios == 8) {
    LOG(LL_INFO, ("state=0x%02x %s; io=0x%02x %s", dev->_state, s, dev->_io, i));
  } else{
    LOG(LL_INFO, ("state=0x%04x %s; io=0x%04x %s", dev->_state, s, dev->_io, i));
  }
}

static bool mgos_mcp23xxx_read(struct mgos_mcp23xxx *dev) {
  uint16_t state, io;

  if (!dev) {
    return false;
  }
  if (dev->num_gpios == 8) {
    if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPIO, 1, (uint8_t *)&state)) {
      return false;
    }
    if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IODIR, 1, (uint8_t *)&io)) {
      return false;
    }
  } else {
    if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPIO * 2, 2, (uint8_t *)&state)) {
      return false;
    }
    if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IODIR * 2, 2, (uint8_t *)&io)) {
      return false;
    }
  }
  dev->_state = state;
  dev->_io    = io;
  return true;
}

static bool mgos_mcp23xxx_write(struct mgos_mcp23xxx *dev) {
  if (!dev) {
    return false;
  }
  if (dev->num_gpios == 8) {
    if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPIO, 1, (uint8_t *)&dev->_state)) {
      return false;
    }
    if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IODIR, 1, (uint8_t *)&dev->_io)) {
      return false;
    }
  } else {
    if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPIO * 2, 2, (uint8_t *)&dev->_state)) {
      return false;
    }
    if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IODIR * 2, 2, (uint8_t *)&dev->_io)) {
      return false;
    }
  }
  return true;
}

static void mgos_mcp23xxx_irq(int pin, void *arg) {
  struct mgos_mcp23xxx *dev = (struct mgos_mcp23xxx *)arg;
  uint8_t  n;
  uint16_t prev_state, this_state;

  if (!dev) {
    return;
  }
  if (dev->int_gpio != pin) {
    return;
  }
  prev_state = dev->_state;
  mgos_mcp23xxx_read(dev);
  this_state = dev->_state;

  if (prev_state == this_state) {
    return;
  }

  for (n = 0; n < dev->num_gpios; n++) {
    bool prev_bit  = prev_state & (1 << n);
    bool this_bit  = this_state & (1 << n);
    bool will_call = false;

    // Do not interrupt for output ports
    if (!(dev->_io & (1 << n))) {
      continue;
    }

    switch (dev->cb[n].mode) {
    case MGOS_GPIO_INT_EDGE_POS:
      if (!prev_bit && this_bit) {
        will_call = true;
      }
      break;

    case MGOS_GPIO_INT_EDGE_NEG:
      if (prev_bit && !this_bit) {
        will_call = true;
      }
      break;

    case MGOS_GPIO_INT_EDGE_ANY:
      if (prev_bit != this_bit) {
        will_call = true;
      }
      break;

    case MGOS_GPIO_INT_LEVEL_HI:
      if (this_bit) {
        will_call = true;
      }
      break;

    case MGOS_GPIO_INT_LEVEL_LO:
      if (!this_bit) {
        will_call = true;
      }
      break;

    default:
      will_call = false;
    }
    // LOG(LL_DEBUG, ("GPIO[%u] prev_bit=%u this_bit=%u will_call=%u", n, prev_bit, this_bit, will_call));
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

  // Clear the interrupt by reading the state again after T(ir) transpires.
  mgos_usleep(5);
  mgos_mcp23xxx_read(dev);
  return;
}

static struct mgos_mcp23xxx *mgos_mcp23xxx_create(struct mgos_i2c *i2c, uint8_t i2caddr, int int_gpio, uint8_t num_gpios) {
  struct mgos_mcp23xxx *dev = NULL;

  if (!i2c) {
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

  dev->_io = 0x0000;      // Set all pins to OUTPUT
  // Read current IO state, assuming all pins are OUTPUT
  if (!mgos_mcp23xxx_read(dev)) {
    LOG(LL_ERROR, ("Could not read state from MCP23XXX"));
    free(dev);
    return NULL;
  }

  // Install interrupt handler, if GPIO pin was specified.
  if (dev->int_gpio != -1) {
    uint8_t val;
    LOG(LL_INFO, ("Installing interrupt handler on GPIO %d", dev->int_gpio));
    mgos_gpio_set_mode(dev->int_gpio, MGOS_GPIO_MODE_INPUT);
    mgos_gpio_set_pull(dev->int_gpio, MGOS_GPIO_PULL_UP);
    mgos_gpio_set_int_handler(dev->int_gpio, MGOS_GPIO_INT_EDGE_NEG, mgos_mcp23xxx_irq, dev);
    mgos_gpio_clear_int(dev->int_gpio);
    mgos_gpio_enable_int(dev->int_gpio);

    // Set IOCON.MIRROR, IOCON.INTPOL
    if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_IOCON, 1, (uint8_t *)&val)) {
      return false;
    }
    val |= 0x40;  // MIRROR=1
    val &= ~0x02; // INTPOL=0
    if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPIO, 1, (uint8_t *)&val)) {
      return false;
    }
  }
  LOG(LL_INFO, ("MCP230%s initialized at I2C 0x%02x", (dev->num_gpios == 8 ? "08" : "17"), dev->i2caddr));
  mgos_mcp23xxx_write(dev);
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

  free(*dev);
  *dev = NULL;
  return true;
}

bool mgos_mcp23xxx_gpio_set_mode(struct mgos_mcp23xxx *dev, int pin, enum mgos_gpio_mode mode) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  if (mode == MGOS_GPIO_MODE_INPUT) {
    dev->_io |= (1 << pin);
    // TODO(pim): Must also set INTCON (0 == compare to previous GPIO) and GPINTEN (1)
    if (dev->num_gpios == 8) {
      uint8_t val;
      if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_INTCON, 1, (uint8_t *)&val)) {
        return false;
      }
      val &= ~(1 << pin);
      if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_INTCON, 1, (uint8_t *)&val)) {
        return false;
      }

      if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN, 1, (uint8_t *)&val)) {
        return false;
      }
      val |= 1 << pin;
      if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN, 1, (uint8_t *)&val)) {
        return false;
      }
    } else {
      uint16_t val;
      if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_INTCON * 2, 2, (uint8_t *)&val)) {
        return false;
      }
      val &= ~(1 << pin);
      if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_INTCON * 2, 2, (uint8_t *)&val)) {
        return false;
      }
      if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN * 2, 2, (uint8_t *)&val)) {
        return false;
      }
      val |= 1 << pin;
      if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN * 2, 2, (uint8_t *)&val)) {
        return false;
      }
    }
  } else {
    if (!mgos_mcp23xxx_gpio_set_pull(dev, pin, MGOS_GPIO_PULL_NONE)) {
      return false;
    }
    // Must also clear GPINTEN
    if (dev->num_gpios == 8) {
      uint8_t val;
      if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN, 1, (uint8_t *)&val)) {
        return false;
      }
      val &= ~(1 << pin);
      if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN, 1, (uint8_t *)&val)) {
        return false;
      }
    } else {
      uint16_t val;
      if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN * 2, 2, (uint8_t *)&val)) {
        return false;
      }
      val &= ~(1 << pin);
      if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPINTEN * 2, 2, (uint8_t *)&val)) {
        return false;
      }
    }
    dev->_io &= ~(1 << pin);
  }

  mgos_mcp23xxx_write(dev);

  return true;
}

bool mgos_mcp23xxx_gpio_set_pull(struct mgos_mcp23xxx *dev, int pin, enum mgos_gpio_pull_type pull) {
  uint16_t gppu;

  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  if (pull == MGOS_GPIO_PULL_DOWN) {
    return false;
  }

  if (dev->num_gpios == 8) {
    if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPPU, 1, (uint8_t *)&gppu)) {
      return false;
    }
  } else {
    if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPPU * 2, 2, (uint8_t *)&gppu)) {
      return false;
    }
  }

  if (pull == MGOS_GPIO_PULL_UP) {
    gppu |= (1 << pin);
  } else {
    gppu &= ~(1 << pin);
  }

  if (dev->num_gpios == 8) {
    if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPPU, 1, (uint8_t *)&gppu)) {
      return false;
    }
  } else {
    if (!mgos_i2c_write_reg_n(dev->i2c, dev->i2caddr, MGOS_MCP23XXX_REG_GPPU * 2, 2, (uint8_t *)&gppu)) {
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
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  dev->_io &= ~(1 << pin);
  mgos_mcp23xxx_gpio_write(dev, pin, level);
  return true;
}

bool mgos_mcp23xxx_gpio_read(struct mgos_mcp23xxx *dev, int pin) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  mgos_mcp23xxx_read(dev);
  return (dev->_state & (1 << pin)) > 0;
}

void mgos_mcp23xxx_gpio_write(struct mgos_mcp23xxx *dev, int pin, bool level) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return;
  }
  if (level) {
    dev->_state |= (1 << pin);
  } else {
    dev->_state &= ~(1 << pin);
  }
  mgos_mcp23xxx_write(dev);

  return;
}

bool mgos_mcp23xxx_gpio_toggle(struct mgos_mcp23xxx *dev, int pin) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  dev->_state ^= (1 << pin);
  mgos_mcp23xxx_write(dev);
  return (dev->_state & (1 << pin)) > 0;
}

bool mgos_mcp23xxx_gpio_set_int_handler(struct mgos_mcp23xxx *dev, int pin, enum mgos_gpio_int_mode mode, mgos_gpio_int_handler_f cb, void *arg) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  dev->cb[pin].fn      = cb;
  dev->cb[pin].fn_arg  = arg;
  dev->cb[pin].mode    = mode;
  dev->cb[pin].enabled = true;
  return true;

  (void)mode;
}

bool mgos_mcp23xxx_gpio_enable_int(struct mgos_mcp23xxx *dev, int pin) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
  }
  dev->cb[pin].enabled = true;
  return true;
}

bool mgos_mcp23xxx_gpio_disable_int(struct mgos_mcp23xxx *dev, int pin) {
  if (!dev || pin < 0 || pin >= dev->num_gpios) {
    return false;
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
  dev->cb[pin].fn      = NULL;
  dev->cb[pin].fn_arg  = NULL;
  dev->cb[pin].firing  = false;
  dev->cb[pin].enabled = false;
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
