/*
 GPIO over standard parallel port Linux driver.
 2022 © Jan Bobrowski
*/

//#define DEBUG
#define DRV_NAME "gpio-parport"
#define pr_fmt(fmt) DRV_NAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>

#include <linux/parport.h>

#include <linux/platform_device.h>
#include <linux/gpio/driver.h>

static const char *pin_names[] = {
	"d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", // data:o
	"err", "slin", "pe", "ack", "busy", // status:i
	"stb", "afd", "init", "sel" // control:i/o (pullup)
};

#define M(r, b) (0b ## b << r ## Shift)
enum {
	DataWidth = 8,
	StatusWidth = 5,
	ControlWidth = 4,

	DataShift = 0,
	StatusShift = DataWidth,
	ControlShift = StatusShift + StatusWidth,

	DataMask = M(Data, 11111111),
	StatusMask = M(Status, 11111),
	ControlMask = M(Control, 1111),

	Direction = StatusMask, // Mark status pins as inputs.
	Inverted = M(Status, 10000) | M(Control, 1011)
};
#undef M

struct gpio_parport {
	struct list_head list;
	unsigned direction;
	struct parport *port;
	struct pardevice *pp_device;
	struct platform_device *gpio_device;
	struct gpio_chip chip;
};

static void set_bits(struct gpio_chip *gc, unsigned mask, unsigned bits)
{
	struct gpio_parport *slot = gpiochip_get_data(gc);
	struct parport *port = slot->pp_device->port;

	bits ^= Inverted;
	bits &= mask;
	if (mask & DataMask) {
		u8 cmask = ~mask;
		if (cmask)
			bits = (bits & mask) | (parport_read_data(port) & cmask);
		parport_write_data(port, bits);
	}
	if (mask & ControlMask) {
		parport_frob_control(port, mask >> ControlShift, bits >> ControlShift);
	}
}

static unsigned get_bits(struct gpio_chip *gc, unsigned mask)
{
	struct gpio_parport *slot = gpiochip_get_data(gc);
	struct parport *port = slot->pp_device->port;
	unsigned bits = 0;

	if (mask & DataMask)
		bits |= parport_read_data(port) & DataMask;
	if (mask & StatusMask)
		bits |= parport_read_status(port) >> 3 << StatusShift & StatusMask;
	if (mask & ControlMask)
		bits |= parport_read_control(port) << ControlShift & ControlMask;
	bits ^= Inverted;
	return bits & mask;
}

static int get_direction(struct gpio_chip *gc, unsigned offset)
{
	struct gpio_parport *slot = gpiochip_get_data(gc);
	int dir = slot->direction >> offset & 1;
	pr_debug("%s(%u) %s\n", __func__, offset, dir ? "in" : "out");
	return dir ? GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
}

static int change_direction(struct gpio_chip *gc, unsigned mask, unsigned dir, unsigned bits)
{
	struct gpio_parport *slot = gpiochip_get_data(gc);
	unsigned direction = dir | (slot->direction & ~mask);
	unsigned change = slot->direction ^ direction;
	if (change & ~ControlMask) {
		pr_debug("Can't change\n");
		return -EINVAL;
	}
	slot->direction = direction;
	set_bits(gc, mask, bits);
	return 0;
}

static int direction_input(struct gpio_chip *gc, unsigned int offset)
{
	unsigned mask = 1 << offset;
	pr_debug("%s(%u)\n", __func__, offset);
	return change_direction(gc, mask, mask, mask);
}

static int direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	unsigned mask = 1 << offset;
	pr_debug("%s(%u,%d)\n", __func__, offset, value);
	return change_direction(gc, mask, 0, value ? mask : 0);
}

static void set(struct gpio_chip *gc, unsigned int offset, int value)
{
	unsigned mask = 1 << offset;
	pr_debug("%s(%u,%d)\n", __func__, offset, value);
	set_bits(gc, mask, value ? mask : 0);
}

static void set_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits)
{
	pr_debug("%s(mask:%02lx bits:%02lx)\n", __func__, *mask, *bits);
	set_bits(gc, *mask, *bits);
}

static int get(struct gpio_chip *gc, unsigned int offset)
{
	pr_debug("%s(%u)\n", __func__, offset);
	return !!get_bits(gc, 1 << offset);
}

static int get_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits)
{
	pr_debug("%s(%02lx)\n", __func__, *mask);
	*bits = get_bits(gc, *mask);
	return 0;
}

static const struct gpio_chip gpio_chip_template = {
	.label = DRV_NAME,
	.get_direction = get_direction,
	.direction_input = direction_input,
	.direction_output = direction_output,
	.get = get,
	.set = set,
	.get_multiple = get_multiple,
	.set_multiple = set_multiple,
	.base = -1,
	.ngpio = ARRAY_SIZE(pin_names),
	.names = pin_names
};

static LIST_HEAD(slots);

static struct gpio_parport *slot_by_port(struct parport *port)
{
	struct gpio_parport *slot;
	list_for_each_entry(slot, &slots, list) {
		if (slot->port == port)
			return slot;
	}
	return 0;
}

#ifdef CONFIG_OF
static char *param_parport = "";
#else
static char *param_parport = "0";
#endif
module_param_named(parport, param_parport, charp, 0664);
MODULE_PARM_DESC(parports, "Comma-separated list of parport numbers to claim, or \"every\".");


static int is_parport_ok(int number)
{
	char *p = param_parport;
	if (strcmp(p, "every") == 0)
		return true;
	while (*p) {
		char *e;
		unsigned long v = simple_strtoul(p, &e, 0);
		if ((*e && *e != ',') || v > INT_MAX) {
			pr_err("Should be a parport number: \"%s\"\n", p);
			e = strchr(e, ',');
			if (!e)
				break;
		} else if (v == number)
			return true;
		if (!*e)
			break;
		p = e + 1;
	}
	return false;
}

static struct platform_device *create_device(int id)
{
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc(DRV_NAME, id);
	if (!pdev)
		return ERR_PTR(-ENOMEM);

	ret = platform_device_add(pdev);
	if (ret) {
		platform_device_put(pdev);
		return ERR_PTR(ret);
	}

	pr_debug("of: %s\n", of_node_full_name(pdev->dev.of_node));

	return pdev;
}

static int create_gpio(struct gpio_parport *slot)
{
	static const struct pardev_cb par_dev_cb = {
//		.preempt = preempt,
//		.private = slot
	};
	int ret;

	slot->pp_device = parport_register_dev_model(slot->port, DRV_NAME, &par_dev_cb, slot->port->number);
	if (!slot->pp_device) {
		pr_debug("parport_register_dev_model() failed\n");
		return -ENODEV;
	}

	slot->chip = gpio_chip_template;
	slot->chip.parent = &slot->gpio_device->dev;
	ret = gpiochip_add_data(&slot->chip, slot);
	if (ret) {
		pr_debug("gpiochip_add_data() failed\n");
		goto err;
	}

	ret = parport_claim(slot->pp_device);
	if (ret) {
		pr_warn("Can't claim %s\n", slot->port->name);
		goto err;
	}

	pr_info("Attached %s to %s\n", dev_name(&slot->gpio_device->dev), slot->port->name);
	return 0;

err:
	parport_unregister_device(slot->pp_device);
	slot->pp_device = NULL;
	return ret;
}

static void attach(struct parport *port)
{
	struct gpio_parport *slot;
	struct platform_device *pdev;
	int id = port->number;
	int ret;

	pr_debug("%s(%s)\n", __func__, port->name);

	pdev = create_device(id);
	if (IS_ERR(pdev)) {
		pr_err("Can't create dev %s.%d\n", DRV_NAME, id);
		return;
	}

	if (!pdev->dev.of_node && !is_parport_ok(id)) {
unreg:
		platform_device_unregister(pdev);
		return;
	}

	slot = kzalloc(sizeof *slot, GFP_KERNEL);
	if (!slot)
		return;
	slot->direction = Direction;
	slot->port = port;
	slot->gpio_device = pdev;
	platform_set_drvdata(pdev, slot);

	ret = create_gpio(slot);
	if (ret < 0) {
		kfree(slot);
		goto unreg;
	}

	list_add(&slot->list, &slots);

	pr_debug("%s end\n", __func__);
}

static void detach(struct parport *port)
{
	struct gpio_parport *slot;

	pr_debug("%s(%s)\n", __func__, port->name);

	slot = slot_by_port(port);
	if (!slot)
		return;

	pr_info("Detaching %s from %s\n", dev_name(&slot->gpio_device->dev), slot->port->name);
	list_del(&slot->list);
	gpiochip_remove(&slot->chip);
	parport_release(slot->pp_device);
	parport_unregister_device(slot->pp_device);
	platform_device_unregister(slot->gpio_device);
	kfree(slot);

	pr_debug("%s end\n", __func__);
}

static struct parport_driver parport_driver = {
	.name = DRV_NAME,
	.match_port = attach,
	.detach = detach,
	.devmodel = true
};

static int __init gpio_parport_init(void)
{
	int ret;

	pr_debug("%s()\n", __func__);

	pr_debug("Waiting for: %s\n", *param_parport ? param_parport : "none");

	ret = parport_register_driver(&parport_driver);
	if (ret) {
		pr_err("Unable to register with parport driver: %pe\n", ERR_PTR(ret));
		return ret;
	}

	pr_debug("%s end\n", __func__);
	return ret;
}

static void __exit gpio_parport_exit(void)
{
	pr_debug("%s()\n", __func__);

	parport_unregister_driver(&parport_driver);

	pr_debug("%s end\n", __func__);
}

module_init(gpio_parport_init);
module_exit(gpio_parport_exit);

MODULE_AUTHOR("Jan Bobrowski <jb@torinak.com>");
MODULE_LICENSE("GPL");
