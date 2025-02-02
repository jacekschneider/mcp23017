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

#define REG_GPA     0x12
#define REG_GPB     0x13

#define REG_OLATA   0x14
#define REG_OLATB   0x15
#define REG_IPOLA   0x02
#define REG_IPOLB   0x03

#define REG_IODIRA  0x00
#define REG_IODIRB  0x01

#define REG_INTCAPA 0x10
#define REG_INTCAPB 0x11

#define INPUT 1
#define OUTPUT 0

struct mcp23017_dev
{
    struct i2c_client*  client;
    struct gpio_chip chip;
};

static s32 mcp_read_byte(const struct i2c_client *client, u8 byte)
{
    s32 value = i2c_smbus_read_byte_data(client, byte);

    if (value < 0) {
        printk(KERN_ERR "mcp23017: Failed to read register 0x%02x, error: %d\n", byte, value);
        return value;
    }
    printk(KERN_DEBUG "mcp23017: Read register 0x%02x, value: 0x%02x\n", byte, value);
    return value;
}

static s32 mcp_write_byte(const struct i2c_client *client, u8 command, u8 value)
{
    s32 result = i2c_smbus_write_byte_data(client, command, value);

    if (result < 0) {
        printk(KERN_ERR "mcp23017: Failed to write 0x%02x to register 0x%02x, error: %d\n",
               value, command, result);
    } else {
        printk(KERN_DEBUG "mcp23017: Wrote 0x%02x to register 0x%02x\n", value, command);
    }
    return result;
}

static inline struct mcp23017_dev *to_mcp23017_dev(struct gpio_chip *gc)
{
	return container_of(gc, struct mcp23017_dev, chip);
}

static int mcp_gpio_get_value(struct gpio_chip *chip, unsigned offset) 
{
    s32 value;
    struct mcp23017_dev* mcp = to_mcp23017_dev(chip);
    unsigned bank = offset / 8;
    unsigned bit = offset % 8;
    u8 reg_intcap = (bank == 0) ? REG_GPA : REG_GPB;

    printk("Getting value at offset %d", offset);
    value = mcp_read_byte(mcp->client, reg_intcap);
    return (value >= 0) ? (value >> bit) & 0x1 : 0;
}

static int mcp_set(struct mcp23017_dev *mcp, unsigned offset, int val)
{
	s32 value;
	unsigned bank = offset / 8 ;
	u8 reg_gpio = (bank == 0) ? REG_GPA : REG_GPB;
	unsigned bit = offset % 8 ;

	value = mcp_read_byte(mcp->client, reg_gpio);
	if (value >= 0) {
		if (val)
			value |= 1 << bit;
		else
			value &= ~(1 << bit);

		return mcp_write_byte(mcp->client, reg_gpio, value);
	}

	return value;
}

static void mcp_gpio_set_value(struct gpio_chip *chip, unsigned offset, int val)
{
    struct mcp23017_dev *mcp = to_mcp23017_dev(chip);

    printk("Seting value at offset %d with value %d", offset, val);
	mcp_set(mcp, offset, val);
}


static int mcp_direction(struct gpio_chip *gc, unsigned offset, unsigned direction, int val)
{
    struct mcp23017_dev *mcp = to_mcp23017_dev(gc);
    unsigned bank = offset / 8;
    unsigned bit = offset % 8;
    u8 reg_iodir = (bank == 0) ? REG_IODIRA : REG_IODIRB;
    s32 iodirval = mcp_read_byte(mcp->client, reg_iodir);
    int err;

    if (iodirval < 0)
        return iodirval;

    if (direction)
        iodirval |= 1 << bit;
    else
        iodirval &= ~(1 << bit);

    err = mcp_write_byte(mcp->client, reg_iodir, iodirval);
    if (err < 0)
        return err;

    return direction ? iodirval : mcp_set(mcp, offset, val);
}

static int mcp_set_gpio_input_direction(struct gpio_chip *chip, unsigned offset)
{
    return mcp_direction(chip, offset, INPUT, 0);
}
static int mcp_set_gpio_output_direction(struct gpio_chip *chip, unsigned offset, int val)
{
    return mcp_direction(chip, offset, OUTPUT, val);
}

static int mcp_get_gpio_direction(struct gpio_chip *chip, unsigned offset)
{
    struct mcp23017_dev *mcp = to_mcp23017_dev(chip);
    unsigned bank = offset / 8;
    unsigned bit = offset % 8;
    u8 reg_iodir = (bank == 0) ? REG_IODIRA : REG_IODIRB;
    s32 iodirvalue = mcp_read_byte(mcp->client, reg_iodir);

    if (iodirvalue < 0)
        return iodirvalue;

    return (iodirvalue & (1 << bit)) ? 1 : 0;
    
}

static int mcp23017_probe(struct i2c_client* client, const struct i2c_device_id *id){
    struct mcp23017_dev* mcp;

    printk("Mcp23017 probe\n");
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
    mcp->chip.direction_input = mcp_set_gpio_input_direction;
    mcp->chip.direction_output = mcp_set_gpio_output_direction;
    mcp->chip.get_direction = mcp_get_gpio_direction;
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







