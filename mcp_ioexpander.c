#include "linux/module.h"
#include "linux/init.h"
#include "linux/i2c.h"
#include "linux/mutex.h"
#include "linux/device.h"
#include "linux/list.h"
#include "linux/gpio/driver.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jacek Schneider");
MODULE_DESCRIPTION("mcp23017 driver");

#define GPA		0x9
#define GPB		0x19

#define OLATA	0x0A
#define OLATB	0x1A
#define IPOLA	0x01
#define IPOLB	0x11

#define IODIRA	0x00
#define IODIRB	0x10

#define INTCAPA	0x8
#define INTCAPB	0x18

#define INPUT 1
#define OUTPUT 0

struct mcp23017_dev
{
    struct i2c_client*  client;
    struct gpio_chip chip;
};

static inline struct mcp23017_dev *to_mcp23017_dev(struct gpio_chip *gc)
{
	return container_of(gc, struct mcp23017_dev, chip);
}

static int mcp_gpio_get_value(struct gpio_chip *chip, unsigned offset) 
{
    printk("Getting value at offset %d", offset);
    s32 value;
    struct mcp23017_dev* mcp = to_mcp23017_dev(chip);
    unsigned bank = offset / 8;
    unsigned bit = offset % 8;

    u8 reg_intcap = (bank == 0) ? INTCAPA : INTCAPB;
    value = i2c_smbus_read_byte_data(mcp->client, reg_intcap);
    return (value >= 0) ? (value >> bit) & 0x1 : 0;
}

static int mcp_set(struct mcp23017_dev *mcp, unsigned offset, int val)
{
	s32 value;

	unsigned bank = offset / 8 ;
	u8 reg_gpio = (bank == 0) ? GPA : GPB;
	unsigned bit = offset % 8 ;

	value = i2c_smbus_read_byte_data(mcp->client, reg_gpio);
	if (value >= 0) {
		if (val)
			value |= 1 << bit;
		else
			value &= ~(1 << bit);

		return i2c_smbus_write_byte_data(mcp->client, reg_gpio, value);
	}

	return value;
}

static void mcp_gpio_set_value(struct gpio_chip *chip, unsigned offset, int val)
{
    printk("Seting value at offset %d with value %d", offset, val);
	struct mcp23017_dev *mcp = to_mcp23017_dev(chip);
	mcp_set(mcp, offset, val);
}

/*
 * direction = 1 => input
 * direction = 0 => output
 */
static int mcp_direction(struct gpio_chip *gc, unsigned offset,
                                unsigned direction, int val)
{
	struct mcp23017_dev *mcp = to_mcp23017_dev(gc);
	unsigned bank = offset / 8 ;
	unsigned bit = offset % 8 ;
	u8 reg_iodir = (bank == 0) ? IODIRA : IODIRB;
	s32 iodirval = i2c_smbus_read_byte_data(mcp->client, reg_iodir);

	if (direction)
		iodirval |= 1 << bit;
	else
		iodirval &= ~(1 << bit);

	i2c_smbus_write_byte_data(mcp->client, reg_iodir, iodirval);
	if (direction)
		return iodirval ;
	else
		return mcp_set(mcp, offset, val);    
}

static int mcp_gpio_input_direction(struct gpio_chip *chip, unsigned offset)
{
    return mcp_direction(chip, offset, INPUT, 0);
}
static int mcp_gpio_output_direction(struct gpio_chip *chip, unsigned offset, int val)
{
    return mcp_direction(chip, offset, OUTPUT, val);
}

static int mcp23017_probe(struct i2c_client* client, const struct i2c_device_id *id){
    printk("Mcp23017 probe\n");
    struct mcp23017_dev* mcp;

    if(!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -EIO;
    mcp = devm_kzalloc(&client->dev, sizeof(*mcp), GFP_KERNEL);
    if(!mcp)
        return -ENOMEM;

    mcp->chip.label = client->name;
    mcp->chip.base  = -1;
    mcp->chip.owner = THIS_MODULE;
    mcp->chip.ngpio = 16;
    mcp->chip.can_sleep = true;
    mcp->chip.get = mcp_gpio_get_value;
    mcp->chip.set = mcp_gpio_set_value;
    mcp->chip.direction_input = mcp_gpio_input_direction;
    mcp->chip.direction_output = mcp_gpio_output_direction;
    mcp->client = client;
    i2c_set_clientdata(client, mcp);

    return gpiochip_add(&mcp->chip);
};

static int mcp23017_remove(struct i2c_client* client){
    struct mcp23017_dev *mcp;
	mcp = i2c_get_clientdata(client);
	gpiochip_remove(&mcp->chip);
    return 0;
};

static const struct of_device_id mcp23017_dt_ids[] = 
{
    { .compatible = "waveshare,mcp23017" },
    {},
};
MODULE_DEVICE_TABLE(of, mcp23017_dt_ids);

static const struct i2c_device_id mcp23017_id[] = 
{
    {"mcp23017", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, mcp23017_id);

static struct i2c_driver mcp23017_i2c_driver = 
{
    .driver =
    {
        .owner = THIS_MODULE,
        .name = "mcp23017",
        .of_match_table = of_match_ptr(mcp23017_dt_ids),
    },
    .probe = mcp23017_probe,
    .remove = mcp23017_remove,
    .id_table = mcp23017_id,
};



module_i2c_driver(mcp23017_i2c_driver)







