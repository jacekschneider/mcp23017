#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/types.h>

struct device;
struct regmap;
struct pinctrl_dev;

struct mcp23017_info
{
    const struct regmap_config *regmap;
    const char *label;
    u16 ngpio;
    bool reg_shift;
}

struct mcp23017
{
    u8 addr;
    bool irq_active_high;
    bool reg_shift;

    u16 irq_rise;
    u16 irq_fall;
    int irq;
    bool irq_controller;
    int cached_gpio;

    struct mutex lock;

    struct gpio_chip chip;

    struct regmap *regmap;

    struct device *dev;

    struct pinctrl_dev *pctldev;
    struct pinctrl_desc pinctrl_desc;
    struct gpio_desc *reset_gpio;
}

extern const struct regmap_config mcp23017_regmap;
