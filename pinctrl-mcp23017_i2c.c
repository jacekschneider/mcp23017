#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/export.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <asm/byteorder.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>

#include "pinctrl-mcp23017_i2c.h"

/* Registers are all 8 bits wide.
 *
 * The mcp23s17 has twice as many bits, and can be configured to work
 * with either 16 bit registers or with two adjacent 8 bit banks.
 */
#define MCP_IODIR	0x00		/* init/reset:  all ones */
#define MCP_IPOL	0x01
#define MCP_GPINTEN	0x02
#define MCP_DEFVAL	0x03
#define MCP_INTCON	0x04
#define MCP_IOCON	0x05
#	define IOCON_MIRROR	(1 << 6)
#	define IOCON_SEQOP	(1 << 5)
#	define IOCON_HAEN	(1 << 3)
#	define IOCON_ODR	(1 << 2)
#	define IOCON_INTPOL	(1 << 1)
#	define IOCON_INTCC	(1)
#define MCP_GPPU	0x06
#define MCP_INTF	0x07
#define MCP_INTCAP	0x08
#define MCP_GPIO	0x09
#define MCP_OLAT	0x0a

static const struct reg_default mcp23017_defaults[] = 
{
    {.reg = MCP_IODIR << 1, .def = 0xffff},
    {.reg = MCP_IPOL << 1, .def = 0x0000},
    {.reg = MCP_GPINTEN << 1, .def = 0x0000},
    {.reg = MCP_DEFVAL << 1, .def = 0x0000},
    {.reg = MCP_INTCON << 1, .def = 0x0000},
    {.reg = MCP_IOCON << 1, .def = 0x0000},
    {.reg = MCP_GPPU << 1, .def = 0x0000},
    {.reg = MCP_OLAT << 1, .def = 0x0000}
};

static const struct regmap_range mcp23017_volatile_range = 
{
    .range_min = MCP_INTF << 1,
    .range_max = MCP_GPIO << 1,
};

static const struct regmap_access_table mcp23017_volatile_table =
{
    .yes_ranges = &mcp23017_volatile_range,
    .n_yes_ranges = 1,
};

static const struct regmap_range mcp23017_precious_range = 
{
    .range_min = MCP_INTCAP << 1,
    .range_max = MCP_GPIO << 1,
};

static const struct regmap_access_table mcp23017_precious_table =
{
    .yes_ranges = &mcp23017_precious_range,
    .n_yes_ranges = 1,
};

const struct regmap_config mcp23017_regmap = 
{
    .reg_bits = 8,
    .val_bits = 16,

    .reg_stride = 2,
    .max_register = MCP_OLAT << 1,
    .volatile_table = &mcp23017_volatile_table,
    .precious_table = &mcp23017_precious_table,
    .reg_defaults = mcp23017_defaults,
    .num_reg_defaults = ARRAY_SIZE(mcp23017_defaults),
    .cache_type = REGCACHE_FLAT,
    .val_format_endian = REGMAP_ENDIAN_LITTLE,
    .disable_locking = true, // protected by mcp_lock()
};
EXPORT_SYMBOL_GPL(mcp23017_regmap);

static int mcp_read(struct mcp23017 *mcp, unsigned int reg, unsigned int *val)
{
    return regmap_read(mcp->regmap, reg << mcp->reg_shift, val);
}

static int mcp_write(struct mcp23017 *mcp, unsigned int reg, unsigned int val)
{
    return regmap_write(mcp->regmap, reg << mcp->reg_shift, val);
}

static int mcp_update_bits(struct mcp23017 *mcp, unsigned int reg, unsigned int mask, unsigned int val)
{
    return regmap_update_bits(mcp->regmap, reg << mcp->reg_shift, mask, val);
}

static int mcp_update_bit(struct mcp23017 *mcp, unsigned int reg, unsigned int pin, bool enabled)
{
    u16 mask = BIT(pin);
    return regmap_update_bits(mcp->regmap, reg, mask, enabled ? mask : 0);
}

static const struct pinctrl_pin_desc mcp23017_pins[] =
{
    PINCTRL_PIN(0, "gpio0");
    PINCTRL_PIN(1, "gpio1");
    PINCTRL_PIN(2, "gpio2");
    PINCTRL_PIN(3, "gpio3");
    PINCTRL_PIN(4, "gpio4");
    PINCTRL_PIN(5, "gpio5");
    PINCTRL_PIN(6, "gpio6");
    PINCTRL_PIN(7, "gpio7");
    PINCTRL_PIN(8, "gpio8");
    PINCTRL_PIN(9, "gpio9");
    PINCTRL_PIN(10, "gpio10");
    PINCTRL_PIN(11, "gpio11");
    PINCTRL_PIN(12, "gpio12");
    PINCTRL_PIN(13, "gpio13");
    PINCTRL_PIN(14, "gpio14");
    PINCTRL_PIN(15, "gpio15");
};

static int mcp_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
    return 0;
}

static const char* mcp_pinctrl_get_group_name(struct pinctrl_dev *pctldev, unsigned int group)
{
    return NULL;
}

static int mcp_pinctrl_get_group_pins(struct pinctrl_dev *pctldev, unsigned int group, const unsigned int **pins, unsigned int *num_pins)
{
    return -ENOTSUPP;
}

static const struct pinctrl_ops mcp_pinctrl_ops =
{
    .get_group_count = mcp_pinctrl_get_group_count,
    .get_group_name = mcp_pinctrl_get_group_name,
    .get_group_pins = mcp_pinctrl_get_group_pins,
    #ifdef CONFIG_OF
    .dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
    .dt_free_map = pinconf_generic_dt_free_map,
    #endif
};

