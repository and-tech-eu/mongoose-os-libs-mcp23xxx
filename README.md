# MCP23008/MCP23017 I2C and MCP23S08/MCP23S17 SPI Driver

A Mongoose library for MCP23XXX, a popular and cheap set of GPIO extenders using
either I2C (MCP32**0**XX) or SPI (MCP32**S**XX)
The MCP23008 is an 8-port device, and the MCP23017 is a 16-port device, but they are
otherwise identical.

**NOTE**: SPI driver is TODO at this point. Contact the author for details.

## Implementation details

The MCP230XX is a bi-directional GPIO with input pullup capability.
Each input pin can be explicitly set as INPUT or OUTPUT pin. For INPUT pins,
the pin can be left floating or pulled up with a 100kOhm internal resistor.

Interrupts are handled with this driver. This allows for callbacks to be set
for INPUT pin state changes.

**NOTE**: For simplicity reasons, on the MCP23x17 chips, `intA` and `intB`
are combined, so it does not matter which of the pins are used on the chip,
any port/pin state change will fire an interrupt, and only one pin needs to
be used on the MCU to handle all 16 ports.

## API Description

To start, `mgos_mcp23xxx_create()` is called with the correct `I2C` bus and
address (by default 0x20), and optionally a GPIO pin on the microcontroller that
serves as an interrupt pin, to detect MCP230XX input state changes.

**NOTE:** When the driver starts, it polls the current state from the chip
without changing any ports. The benefit of this is that the MCU can safely
reboot without loss of the GPIO state in MCP230XX.

The API follows `mgos_gpio.h` closely, enabling ports to be set as input or output.
For input ports, `mgos_mcp23xxx_gpio_set_pull()` can be called with either
`MGOS_GPIO_PULL_NONE` or `MGOS_GPIO_PULL_UP`, but not with `MGOS_GPIO_PULL_DOWN`
which returns an error, as the chip does not support input pulldowns.

Notably, `mgos_mcp23xxx_gpio_set_int_handler()` and `mgos_mcp23xxx_gpio_set_button_handler()`
work identically to the `mgos_gpio_*()` variants.

## Example application

```
#include "mgos.h"
#include "mgos_config.h"
#include "mgos_mcp23xxx.h"

static void button_cb(int pin, void *user_data) {
  struct mgos_mcp23xxx *d = (struct mgos_mcp23xxx *)user_data;
  LOG(LL_INFO, ("GPIO[%d] callback, value=%d", pin, mgos_mcp23xxx_gpio_read(d, pin)));
  mgos_mcp23xxx_gpio_toggle(d, pin+8);
}

enum mgos_app_init_result mgos_app_init(void) {
  struct mgos_mcp23xxx *d;
  int i;

  if (!(d = mgos_mcp23017_create(mgos_i2c_get_global(), mgos_sys_config_get_mcp23xxx_i2caddr(),
                                mgos_sys_config_get_mcp23xxx_int_gpio()))) {
    LOG(LL_ERROR, ("Could not create MCP230XX"));
    return MGOS_APP_INIT_ERROR;
  }

  for(i=0; i<4; i++) mgos_mcp23xxx_gpio_setup_input(d, i, MGOS_GPIO_PULL_UP);
  for(i=8; i<16; i++) mgos_mcp23xxx_gpio_set_mode(d, i, MGOS_GPIO_MODE_OUTPUT);

  mgos_mcp23xxx_gpio_set_button_handler(d, 0, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_NEG, 10, button_cb, d);
  mgos_mcp23xxx_gpio_set_button_handler(d, 1, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_POS, 10, button_cb, d);
  mgos_mcp23xxx_gpio_set_button_handler(d, 2, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_ANY, 10, button_cb, d);
  mgos_mcp23xxx_gpio_set_button_handler(d, 3, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_ANY, 10, button_cb, d);

  return MGOS_APP_INIT_SUCCESS;
}
```

# Disclaimer

This project is not an official Google project. It is not supported by Google
and Google specifically disclaims all warranties as to its quality,
merchantability, or fitness for a particular purpose.
