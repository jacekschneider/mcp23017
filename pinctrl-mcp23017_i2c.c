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
#include <linux/i2c.h>

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

static int mcp_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin, unsigned long *config)
{
    struct mcp23017 *mcp = pinctrl_dev_get_drvdata(pctldev);
    enum pin_config_param param = pinconf_to_config_param(*config);
    unsigned int data, status;
    int ret;

    switch (param)
    {
        case PIN_CONFIG_BIAS_PULL_UP:
            mutex_lock(&mcp->lock);
            ret = mcp_read(mcp, MCP_GPPU, &data);
            mutex_unlock(&mcp->lock);
            if (ret < 0)
                return ret;
            status = (data & BIT(pin)) ? 1 : 0;
            break;
        default:
            return -ENOTSUPP;
    }
    *config = 0;

    return status ? 0 : -EINVAL;
}

static int mcp_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin, unsigned long *configs, unsigned int num_configs)
{
    struct mcp23017 *mcp = pinctrl_drv_get_drvdata(pctldev);
    enum pin_config_param param;
    u32 arg;
    int ret = 0;
    int i;

    for (i = 0; i < num_configs; i++)
    {
        param = pinconf_to_config_param(configs[i]);
        arg = pinconf_to_config_argument(configs[i]);
        switch (param)
        {
            case PIN_CONFIG_BIAS_PULL_UP:
                mutex_lock(&mcp->lock);
                ret = mcp_set_bit(mcp, MCP_GPPU, pin, arg);
                mutex_unlock(&mcp->lock);
                break;
            default:
                dev_dbg(mcp->dev, "Invalid config param %04x\n", param);
                return -ENOTSUPP;
        }
    }

    return ret;
}

static const struct pinconf_ops mcp_pinconf_ops =
{
    .pin_config_get = mcp_pinconf_get,
    .pin_config_set = mcp_pinconf_set,
    .is_generic = true,
};

static int mcp23017_direction_input(struct gpio_chip *chip, unsigned offset)
{
    struct mcp23017 *mcp = gpiochip_get_data(chip);
    int status;

    mutex_lock(&mcp->lock);
    status = mcp_set_bit(mcp, MCP_IODIR, offset, true);
    mutex_unlock(&mcp->lock);

    return status;
}

static int mcp23017_get(struct gpio_chip *chip, unsigned offset)
{
    struct mcp23017 *mcp = gpiochip_get_data(chip);
    int status, ret;

    mutex_lock(&mcp->lock);
    /* Reading this clears IRQ */
    ret = mcp_read(mcp, MCP_GPIO, &status);
    if (ret < 0)
        status = 0;
    else
    {
        mcp->cached_gpio = status;
        status = !!(status & BIT(offset));
    }
    mutex_unlock(&mcp->lock);

    return status;
}

static int mcp23017_get_multiple(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{
    struct mcp23017 *mcp = gpiochip_get_data(chip);
    unsigned int status;
    int ret;

    mutex_lock(&mcp->lock);
    /* Reading this clears IRQ */
    ret = mcp_read(mcp, MCP_GPIO, &status);
    if (ret < 0)
        status = 0;
    else
    {
        mcp->cached_gpio = status;
        *bits = status;
    }
    mutex_unlock(&mcp->lock);

    return ret;
}

static int __mcp23017_set(struct mcp23017 *mcp, unsigned mask, bool value)
{
    return mcp_update_bits(mcp, MCP_OLAT, mask, value ? mask : 0);
}

static int mcp23017_set(struct gpio_chip *chip, unsigned int offset, int value)
{
    struct mcp23017 *mcp = gpiochip_get_data(chip);
    unsigned mask = BIT(offset);
    int ret;

    mutex_lock(&mcp->lock);
    ret = __mcp23017_set(mcp, mask, !!value);
    mutex_unlock(&mcp->unlock);

    return ret;
}

static int mcp23017_set_multiple(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{
    struct mcp23017 *mcp = gpiochip_get_data(chip);
    int ret;

    mutex_lock(&mcp->lock);
    ret = mcp_update_bits(mcp, MCP_OLAT, *mask, *bits);
    mutex_unlock(&mcp->unlock);

    return ret;
}

static int mcp23017_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
    struct mcp23017 *mcp = gpiochip_get_data(chip);
    unsigned mask = BIT(offset);
    int status;

    mutex_lock(&mcp->lock);
    status = __mcp23017_set(mcp, mask, value);
    if (status == 0)
    {
        status = mcp_update_bits(mcp, MCP_IODIR, mask, 0);
    }
    mutex_unlock(&mcp->lock);
    return status;
}

static irqreturn_t mcp23017_irq(int irq, void *data)
{
    struct mcp23017 *mcp = data;
    int intcap, intcon, intf, i, gpio, gpio_orig, intcap_mask, defval, gpinten;
    bool need_unmask = false;
    unsigned long int enabled_interrupts, defval_changed, gpio_set;
    bool intf_set, intcap_changed, gpio_bit_changed, defval_changed, gpio_set;

    mutex_lock(&mcp->lock);
    if (mcp_read(mcp, MCP_INTF, &intf))
        goto unlock;
    
    if (intf == 0)
    {
        /* There is no interrupt pending */
        goto unlock:
    }

    if (mcp_read(mcp, MCP_INTCON, &intcon))
        goto unlock;

    if (mcp_read(mcp, MCP_GPINTEN, &gpinten))
        goto unlock;
    
    if (mcp_read(mcp, MCP_DEFVAL, &defval))
        goto unlock;
    
    /* Disable interrupts to avoid reactivation after clearing */
    if (intcon)
    {
        need_unmask = true;
        if (mcp_read(mcp, MCP_INTCAP, &intcap))
            goto unlock;
    }

    if (mcp_read(mcp, MCP_INTCAP, &intcap))
        goto unlock;
    
    /* Clears the interrupt */
    if (mcp_read(mcp, MCP_GPIO, &gpio))
        goto unlock;

    gpio_orig = mcp->cached_gpio;
    mcp->cached_gpio = gpio;
    mutex_unlock(&mcp->lock);

    dev_dbg(mcp->chip.parent, "intcap 0x%04X intf 0x%04X gpio_orig 0x%04X gpio 0x%04X\n",
            intcap, intf, gpio_orig, gpio);
    
    enabled_interrupts = gpinten;
    for_each_set_bit(i, &enabled_interrupts, mcp->chip.ngpio)
    {
        intf_set = intf & BIT(i);
        if (i < 8 && intf_set)
            intcap_mask = 0x00FF;
        else if (i >= 8 && intf_set)
            intcap_mask = 0xFF00;
        else
            intcap_mask = 0x00;
        
        intcap_changed = (intcap_mask & (intcap & BIT(i))) != (intcap_mask & (BIT(i) & gpio_orig));
        gpio_set = BIT(i) & gpio;
        gpio_bit_changed = (BIT(i) & gpio_orig) != (BIT(i) & gpio);
        defval_changed = (BIT(i) & intcon) && ((BIT(i) & gpio) != (BIT(i) & defval));

        if (((gpio_bit_changed || intcap_changed) && (BIT(i) & mcp->irq_rise) && gpio_set) ||
            ((gpio_bit_changed || intcap_changed) && (BIT(i) & mcp->irq_fall) && gpio_set) ||
            defval_changed)
        {
            child_irq = irq_find_mapping(mcp->chip.irq.domain, i);
            handle_nested_irq(child_irq);
        }
    }

    if (need_unmask)
    {
        mutex_lock(&mcp->lock);
        goto unlock;
    }

    return IRQ_HANDLED;

unlock:
    if (need_unmask)
        if(mcp_write(mcp, MCP_GPINTEN, gpinten))
            dev_err(mcp->chip.parent, "Can't unmask GPINTEN\n")
    
    mutex_unlock(&mcp->lock);
    return IRQ_HANDLED;

}

static void mcp23017_irq_mask(struct irq_data *data)
{
    struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
    struct mcp23017 *mcp = gpiochip_get_data(gc);
    unsigned int pos = irqd_to_hwirq(data);

    mcp_set_bit(mcp, MCP_GPINTEN, pos, false);
    gpiochip_disable_irq(gc, pos);
}

static void mcp23017_irq_unmask(struct irq_data *data)
{
    struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
    struct mcp23017 *mcp = gpiochip_get_data(gc);
    unsigned int pos = irqd_to_hwirq(data);

    gpiochip_enable_irq(gc, pos);
    mcp_set_bit(mcp, MCP_GPINTEN, pos, true); 
}

static int mcp23017_irq_set_type(struct irq_data *data, unsigned int type)
{
    struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
    struct mcp23017 *mcp = gpiochip_get_data(gc);
    unsigned int pos = irqd_to_hwirq(data);

    if ((type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH)
    {
        mcp_set_bit(mcp, MCP_INTCON, pos, false);
        mcp->irq_rise |= BIT(pos);
        mcp->irq_fall |= BIT(pos);
    }
    else if (type & IRQ_TYPE_EDGE_RISING)
    {
        mcp_set_bit(mcp, MCP_INTCON, pos, false);
        mcp->irq_rise |= BIT(pos);
        mcp->irq_fall &= ~BIT(pos);
    }
    else if (type & IRQ_TYPE_EDGE_FALLING)
    {
        mcp_set_bit(mcp, MCP_INTCON, pos, false);
        mcp->irq_rise &= ~BIT(pos);
        mcp->irq_fall |= BIT(pos);
    }
    else if (type & IRQ_TYPE_LEVEL_HIGH)
    {
        mcp_set_bit(mcp, MCP_INTCON, pos, true);
        mcp_set_bit(mcp, MCP_DEFVAL, pos, false);
    }
    else if (type & IRQ_TYPE_LEVEL_LOW)
    {
        mcp_set_bit(mcp, MCP_INTCON, pos, true);
        mcp_set_bit(mcp, MCP_DEFVAL, pos, true);
    }
    else
        return -EINVAL;

    return 0;
}

static void mcp23017_irq_bus_lock(struct irq_data *data)
{
    struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
    struct mcp23017 *mcp = gpiochip_get_data(gc);

    mutex_lock(&mcp->lock);
    regcache_cache_only(mcp->regmap, true);
}

static void mcp23017_irq_bus_unlock(struct irq_data *data)
{
    struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
    struct mcp23017 *mcp = gpiochip_get(gc);

    regcache_cache_only(mcp->regmap, false);
    regcache_sync(mcp->regmap);
    mutex_unlock(&mcp->lock);
}

static int mcp23017_irq_setup(struct mcp23017* mcp)
{
    struct gpio_chip *chip = &mcp->chip;
    int err;
    unsigned long irqflags = IRQF_ONESHOT | IRQF_SHARED;

    if (mcp->irq_active_high)
        irqflags |= IRQF_TRIGGER_HIGH;
    else
        irqflags |= IRQF_TRIGGER_LOW;

    err = devm_request_threaded_irq(chip->parent, mcp->irq, NULL, mcp23017_irq, irqflags, dev_name(chip->parent), mcp);
    if (err != 0)
    {
        dev_err(chip->parent, "unable to request IRQ#%d: %d\n", mcp->irq, err);
        return err;
    }

    return 0;
}

static void mcp23017_print_chip(struct irq_data *d, struct seq_file *p)
{
    struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
    struct mcp23017 *mcp = gpiochip_get_data(gc);

    seq_puts(p, dev_name(mcp->dev));
}

static const struct irq_chip mcp23017_irq_chip = 
{
    .irq_mask = mcp23017_irq_mask,
    .irq_unmask = mcp23017_irq_unmask,
    .irq_set_type = mcp23017_irq_set_type,
    .irq_bus_lock = mcp23017_irq_bus_lock,
    .irq_bus_sync_unlock = mcp23017_bus_unlock,
    .irq_print_chip = mcp23017_print_chip,
    .flags = IRQCHIP_IMMUTABLE,
    GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static const struct of_device_id mcp23017_dt_table[] =
{
    {.compatible = "mcp, mcp23017"},
    {},
};
MODULE_DEVICE_TABLE(of, mcp23017_dt_table);

static const struct i2c_device_id mcp23017_id_table[] =
{
    {"mcp23017", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, mcp23017_i2c_id);

static void mcp23017_remove(struct i2c_client* client)
{
    struct mcp23017 *mcp = i2c_get_clientdata(client);
    gpiochip_remove(&mcp->chip);
}

static int mcp23017_probe(struct i2c_client* client)
{
    struct mcp23017 *mcp = i2c_get_clientdata(client);
    struct device *dev = &client->dev;
    int ret, status;

    printk("mcp23017 probe");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -EIO;
    mcp = devm_kzalloc(&client->dev, sizeof(*mcp), GFP_KERNEL);
    if(!mcp)
        return -ENOMEM;

    mcp->reg_shift = 1,
    mcp->chip.ngpio = 16,
    mcp->chip.label = "mcp23017";
    mcp->regmap = devm_regmap_init_i2c(client, mcp23017_regmap);
    if(IS_ERR(mcp->regmap))
        return PTR_ERR(mcp->regmap);
    
    mcp->irq = client->irq;
    mcp->pinctrl_desc.name = "mcp23017-pinctrl";

    bool mirror = false;
    bool open_drain = false;

    mutex_init(&mcp->lock);

    mcp->dev = dev;
    mcp->addr = client->addr;

    mcp->irq_active_high = false;

    mcp->chip.direction_input = mcp23017_direction_input;
    mcp->chip.get = mcp23017_get;
    mcp->chip.get_multiple = mcp23017_get_multiple;
    mcp->chip.direction_output = mcp23017_direction_output;
    mcp->chip.set = mcp23017_set;
    mcp->chip.set_multiple = mcp23017_set_multiple;

    mcp->chip.base = base;
    mcp->chip.can_sleep = true;
    mcp->chip.parent = dev;
    mcp->chip.owner = THIS_MODULE;

    mcp->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIO_OUT_LOW);

    ret = mcp_write(mcp, MCP_IODIR, 0xFFFF);
    if (ret < 0)
        return ret;
    
    ret = mcp_read(mcp, MCP_IOCON, &status);
    if (ret < 0)
        return dev_err_probe(dev, ret, "Can't read IOCON at addr: %d\n", addr);
    
    mcp->irq_controller = device_property_read_bool(dev, "interrupt-controller");
    if (mcp->irq && mcp->irq_controller)
    {
        mcp->irq_active_high = device_property_read_bool(dev, "microchip,irq-active-high");
        mirror = device_propert_read_bool(dev, "micrioschip,irq-mirror");
        open_drain = device_property_read_bool(dev, "driver-open-drain");
    }

    if((status & IOCON_SEQOP) || !(status & IOCON_HAEN) || mirror || mcp->irq_active_high || open_drain)
    {
        status &= ~(IOCON_SEQOP | (IOCON_SEQOP << 8));
        status |= IOCON_HAEN | (IOCON_HAEN << 8);
        if (mcp->irq_active_high)
            status |= IOCON_INTPOL | (IOCON_INTPOL << 8);
        else
            status &= ~(IOCON_INTPOL | (IOCON_INTPOL << 8));
        if (mirror)
            status |= IOCON_MIRROR | (IOCON_MIRROR << 8);
        if (open_drain)
            status |= IOCON_ODR | (IOCON_ODR << 8);
        
        ret = mcp_write(mcp, MCP_IOCON, status);
        if (ret < 0)
            return dev_err_probe(dev, ret, "can't write IOCON at addr: %d\n", addr);
    }

    if (mcp->irq && mcp->irq_controller)
    {
        struct gpio_irq_chip *girq = &mcp->chip.irq;

        gpio_irq_chip_set_chip(girq, &mcp23017_irq_chip);
        girq->parent_handler = NULL;
        girq->num_parents = 0;
        girq->parents = NULL;
        girq->default_type = IRQ_TYPE_NONE;
        girq->handler = handle_simple_irq;
        girq->threaded = true;
    }

    ret = devm_gpiochip_add_data(dev, &mcp->chip, mcp);
    if (ret < 0)
        return dev_err_probe(dev, ret, "can't add GPIO chip\n");
    
    mcp->pinctrl_desc.pctlops = &mcp_pinctrl_ops;
    mcp->pinctril_desc.confops = &mcp_pinconf_ops;
    mcp->pinctrl_desc.npins = &mcp->chip.ngpio;
    mcp->pinctril_desc.pins = mcp23017_pins;
    mcp->pinctrl_desc.owner = THIS_MODULE;

    mcp->pctldev = devm_pinctrl_register(dev, &mcp->pinctrl_desc, mcp);
    if(IS_ERR(mcp->pctldev))
        return dev_err_probe(dev, PTR_ERR(mcp->pctldev), "can't register controller\n");
    
    if (mcp->irq)
    {
        ret = mcp23017_irq_setup(mcp);
        if(ret)
            return dev_err_probe(dev, ret, "can't setup IRQ\n");
    }

    i2c_set_clientdata(client, mcp);
    
    return 0;
}

static struct i2c_driver mcp23017_i2c_driver =
{
    .driver =
    {
        .owner = THIS_MODULE,
        .name = "mcp23017",
        .of_match_table = of_match_ptr(mcp23017_dt_table),
    },
    .probe = mcp23017_probe,
    .remove = mcp23017_remove,
    .id_table = mcp23017_id_table
}

MODULE_DESCRIPTION("MCP23S17 I2C GPIO driver");
MODULE_LICENSE("GPL V3");