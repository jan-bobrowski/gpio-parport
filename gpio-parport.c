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

static inline struct gpio_parport *slot_alloc(void)
{
	struct gpio_parport *slot = kzalloc(sizeof *slot, GFP_KERNEL);
	if (slot) {
		slot->direction = Direction;
	}
	return slot;
}

static inline void slot_free(struct gpio_parport *slot)
{
	kfree(slot);
}

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
static char *param_parport = "parport0";
#endif
module_param_named(parport, param_parport, charp, 0664);
MODULE_PARM_DESC(parports, "Comma-separated list of parport names to claim, or \"every\".");

static int is_parport_ok(const char *name)
{
	int len = strlen(name);
	char *p = param_parport;
	if (strcmp(p, "every") == 0)
		return true;
	while (*p) {
		if (strncmp(p, name, len) == 0 && (!p[len] || p[len] == ','))
			return true;
		p = strchr(p, ',');
		if (!p) break;
		p++;
	}
	return false;
}

static void disassoc(struct gpio_parport *slot);

static void attach(struct parport *port)
{
	struct gpio_parport *slot;

	pr_debug("%s(%s)\n", __func__, port->name);

	slot = slot_alloc();
	if (slot)
		slot->port = port;
	list_add(&slot->list, &slots);

	platform_device_register_simple(DRV_NAME, PLATFORM_DEVID_AUTO, NULL, 0);

	pr_debug("%s end\n", __func__);
}

static void detach(struct parport *port)
{
	struct gpio_parport *slot;

	pr_debug("%s(%s)\n", __func__, port->name);

	slot = slot_by_port(port);
	if (!slot)
		return;

	list_del_init(&slot->list);
	if (slot->gpio_device) {
		disassoc(slot);
		parport_unregister_device(slot->pp_device);
		slot->port = NULL;
		slot->pp_device = NULL;
	} else {
		slot_free(slot);
	}

	pr_debug("%s end\n", __func__);
}

static struct parport_driver parport_driver = {
	.name = DRV_NAME,
	.match_port = attach,
	.detach = detach,
	.devmodel = true
};

static int probe(struct platform_device *pdev)
{
	static const struct pardev_cb par_dev_cb = {
//		.preempt = preempt,
//		.private = slot
	};
	struct gpio_parport *slot;
	const char *parport_name;
	const char *prop_parport;
	int ret;

	pr_debug("%s(%s)\n", __func__, dev_name(&pdev->dev));

	prop_parport = NULL;
	if (pdev->dev.of_node) {
		struct device_node *node = pdev->dev.of_node;
		ret = of_property_read_string(node, "parport", &prop_parport);
		if (ret) {
			pr_warn("No parport property: %pe\n", ERR_PTR(ret));
			return -ENODEV;
		}
		pr_debug("parport = \"%s\"", prop_parport);
	}

	list_for_each_entry(slot, &slots, list) {
		if (slot->gpio_device)
			continue;
		parport_name = slot->port->name;
		if (prop_parport ? strcmp(prop_parport, parport_name) == 0 : is_parport_ok(parport_name))
			goto found;
	}
	return -ENODEV;

found:
	pr_debug("Found %s!\n", parport_name);

	slot->pp_device = parport_register_dev_model(slot->port, DRV_NAME, &par_dev_cb, pdev->id);
	if (!slot->pp_device) {
		pr_debug("parport_register_dev_model() failed\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, slot);

	slot->chip = gpio_chip_template;
	slot->chip.parent = &pdev->dev;
	ret = gpiochip_add_data(&slot->chip, slot);
	if (ret) {
		pr_debug("gpiochip_add_data() failed\n");
		goto err;
	}

	ret = parport_claim(slot->pp_device);
	if (ret) {
		pr_warn("Can't claim %s\n", parport_name);
		goto err;
	}

	slot->gpio_device = pdev;
	pr_info("Attached %s to %s\n", dev_name(&pdev->dev), parport_name);
	pr_debug("%s end\n", __func__);
	return 0;

err:
	parport_unregister_device(slot->pp_device);
	slot->pp_device = NULL;
	return ret;
}

static int remove(struct platform_device *pdev)
{
	struct gpio_parport *slot;

	pr_debug("%s()\n", __func__);

	slot = platform_get_drvdata(pdev);
	if (!slot->pp_device) {
		slot_free(slot);
	} else {
		disassoc(slot);
		parport_unregister_device(slot->pp_device);
		slot->gpio_device = NULL;
	}

	pr_debug("%s end\n", __func__);
	return 0;
}

static void disassoc(struct gpio_parport *slot)
{
	struct platform_device *pdev = slot->gpio_device;
	pr_info("Detaching %s from %s\n", dev_name(&pdev->dev), slot->port->name);
	gpiochip_remove(&slot->chip);
	parport_release(slot->pp_device);
}

#ifdef CONFIG_OF
static const struct of_device_id of_match_table[] = {
	{ .compatible = DRV_NAME },
	{},
};
MODULE_DEVICE_TABLE(of, of_match_table);
#endif

static struct platform_driver gpio_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(of_match_table)
	},
	.probe = probe,
	.remove = remove
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

	ret = platform_driver_register(&gpio_driver);
	if (ret) {
		pr_err("Unable to register GPIO driver: %pe\n", ERR_PTR(ret));
		parport_unregister_driver(&parport_driver);
	}

	pr_debug("%s end\n", __func__);
	return ret;
}

static void __exit gpio_parport_exit(void)
{
	pr_debug("%s()\n", __func__);

	platform_driver_unregister(&gpio_driver);
	parport_unregister_driver(&parport_driver);

	pr_debug("%s end\n", __func__);
}

module_init(gpio_parport_init);
module_exit(gpio_parport_exit);

MODULE_AUTHOR("Jan Bobrowski <jb@torinak.com>");
MODULE_LICENSE("GPL");
