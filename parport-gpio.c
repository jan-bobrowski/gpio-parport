/*
 GPIO over standard parallel port Linux driver.
 2022 © Jan Bobrowski
*/

#define DEBUG
#define DRV_NAME "parport-gpio"
#define pr_fmt(fmt) DRV_NAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>

#include <linux/parport.h>

#include <linux/platform_device.h>
#include <linux/gpio/driver.h>

#define elemof(T) (sizeof T/sizeof*T)

static const char *pin_names[] = {
	"d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", // data:o
	"err", "slin", "pe", "ack", "busy", // status:i
	"stb", "afd", "init", "sel" // control:i/o (pullup)
};

enum {
	DataWidth = 8,
	StatusWidth = 5,
	ControlWidth = 4,

	DataShift = 0,
	StatusShift = DataWidth,
	ControlShift = StatusShift + StatusWidth,

	DataMask    = 0b00000000011111111,
	StatusMask  = 0b00001111100000000,
	ControlMask = 0b11110000000000000,

	Direction = DataMask | ControlMask,
	Inverted = 0b10000 << StatusShift | 0b1011 << ControlShift
};

struct parport_gpio {
	struct list_head list;
	int id; // parport number
	unsigned direction;
	struct pardevice *par_device;
	struct platform_device *gpio_device;
	struct gpio_chip gpio;
};

static LIST_HEAD(slots);

static inline struct parport_gpio *slot_alloc(int id)
{
	struct parport_gpio *slot = kzalloc(sizeof *slot, GFP_KERNEL);
	if (slot) {
		slot->id = id;
		slot->direction = Direction;
	}
	return slot;
}

static inline void slot_free(struct parport_gpio *slot)
{
	kfree(slot);
}

static struct parport_gpio *slot_by_port(struct parport *port)
{
	struct parport_gpio *slot;
	list_for_each_entry(slot, &slots, list) {
		if (slot->par_device->port == port)
			return slot;
	}
	return 0;
}

static struct parport_gpio *slot_by_id(int id)
{
	struct parport_gpio *slot;
	list_for_each_entry(slot, &slots, list) {
		if (slot->id == id)
			return slot;
	}
	return 0;
}

static void set_bits(struct gpio_chip *gc, unsigned mask, unsigned bits)
{
	struct parport_gpio *slot = gpiochip_get_data(gc);
	struct parport *port = slot->par_device->port;

	bits ^= Inverted;
	bits &= mask;
	if (mask & DataMask) {
		u8 cmask = ~mask;
		if (cmask & 0xff)
			bits = (bits & mask) | (parport_read_data(port) & cmask);
		parport_write_data(port, bits);
	}
	if (mask & ControlMask) {
		mask >>= ControlShift;
		bits >>= ControlShift;
		parport_frob_control(port, mask, bits);
	}
}

static unsigned get_bits(struct gpio_chip *gc, unsigned mask)
{
	struct parport_gpio *slot = gpiochip_get_data(gc);
	struct parport *port = slot->par_device->port;
	unsigned bits = 0;

	if (mask & DataMask)
		bits |= parport_read_data(port);
	if (mask & StatusMask)
		bits |= parport_read_status(port) << StatusShift;
	if (mask & ControlMask)
		bits |= parport_read_control(port) << ControlShift;
	bits ^= Inverted;
	return bits & mask;
}

static int get_direction(struct gpio_chip *gc, unsigned offset)
{
	struct parport_gpio *slot = gpiochip_get_data(gc);
	pr_devel("%s(%u)\n", __func__, offset);
	return (slot->direction >> offset & 1) ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int change_direction(struct gpio_chip *gc, unsigned mask, unsigned dir, unsigned bits)
{
	struct parport_gpio *slot = gpiochip_get_data(gc);
	unsigned direction = dir | (slot->direction & ~mask);
	unsigned change = slot->direction ^ direction;
	if (change & ~ControlMask)
		return -EINVAL;
	set_bits(gc, change, bits);
	return 0;
}

static int direction_input(struct gpio_chip *gc, unsigned int offset)
{
	unsigned mask = 1 << offset;
	pr_devel("%s(%u)\n", __func__, offset);
	return change_direction(gc, mask, 0, mask);
}

static int direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	unsigned mask = 1 << offset;
	pr_devel("%s(%u)\n", __func__, offset);
	return change_direction(gc, mask, mask, value ? mask : 0);
}

static void set(struct gpio_chip *gc, unsigned int offset, int value)
{
	unsigned mask = 1 << offset;
	pr_devel("%s(%u,%d)\n", __func__, offset, value);
	set_bits(gc, mask, value ? mask : 0);
}

static void set_multiple(struct gpio_chip *gc, unsigned long *mask_ptr, unsigned long *bits_ptr)
{
	u8 mask = *mask_ptr & 0xff;
	u8 bits = *bits_ptr;
	pr_devel("%s() mask:%02x bits:%02x\n", __func__, mask, bits);
	if (mask)
		set_bits(gc, mask, bits);
}

static int get(struct gpio_chip *gc, unsigned int offset)
{
	pr_devel("%s(%u)\n", __func__, offset);
	return !!get_bits(gc, 1 << offset);
}

static int get_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits)
{
	pr_devel("%s()\n", __func__);
	*bits = get_bits(gc, *mask);
	return 0;
}

/*
 parport_gpio_init {
  attach {alloc, add to list}
  probe {}
 }

 parport_gpio_exit {
  detach {
   remove {}
   del from list, free
  }
 }
*/

static char *param_parport[8];

module_param_array_named(parport, param_parport, charp, NULL, 0);
MODULE_PARM_DESC(parport, "Comma-separated list of parport numbers to claim.");

static unsigned long parports_to_claim = 0;

static void attach(struct parport *port)
{
	struct parport_gpio *slot;
	static const struct pardev_cb par_dev_cb = {
//		.preempt = preempt,
//		.private = slot
	};
	struct pardevice *pardev;
	struct platform_device *pdev;
	int id;

	pr_devel("%s(%s)\n", __func__, port->name);

	if (!(parports_to_claim & 1<<port->number))
		return;

	id = port->number;
	pardev = parport_register_dev_model(port, DRV_NAME, &par_dev_cb, id);
	if (!pardev) {
		pr_err("parport_register_dev_model() failed\n");
		return;
	}

	pdev = platform_device_register_simple(DRV_NAME, id, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("Error registering device\n");
		goto err_unreg_pardev;
	}

	slot = slot_alloc(id);
	if (!slot) {
		goto err_unreg_pdev;
	}

	slot->par_device = pardev;
	slot->gpio_device = pdev;
	list_add(&slot->list, &slots);

	pr_info("Attached %s.%d to parport%d\n", DRV_NAME, id, port->number);
	pr_devel("%s end\n", __func__);
	return;

err_unreg_pdev:
	platform_device_unregister(pdev);
err_unreg_pardev:
	parport_unregister_device(pardev);
	pr_devel("%s end\n", __func__);
}

static void detach(struct parport *port)
{
	struct parport_gpio *slot;

	pr_devel("%s()\n", __func__);

	slot = slot_by_port(port);
	if (slot) {
		pr_info("Detaching %s.%d from parport%d\n", DRV_NAME, slot->id, port->number);
		parport_release(slot->par_device);
		parport_unregister_device(slot->par_device); // calls remove()
		slot->par_device = 0;
		platform_device_unregister(slot->gpio_device);
		list_del_init(&slot->list);
		slot_free(slot);
	}

	pr_devel("%s end\n", __func__);
}

static int probe(struct platform_device *pdev)
{
	struct parport_gpio *slot;
	int ret;

	static struct gpio_chip gc = {
		.label = DRV_NAME,
		.get_direction = get_direction,
		.direction_input = direction_input,
		.direction_output = direction_output,
		.get = get,
		.get_multiple = get_multiple,
		.set = set,
		.set_multiple = set_multiple,
		.base = -1,
		.ngpio = elemof(pin_names),
		.names = pin_names
	};

	pr_devel("%s() pdev->id = %d\n", __func__, pdev->id);

	slot = slot_by_id(pdev->id);
	if (!slot) {
		pr_err("No slot having id %d\n", pdev->id);
		return -ENODEV;
	}
	platform_set_drvdata(pdev, slot);

	slot->gpio = gc;

	ret = gpiochip_add_data(&slot->gpio, slot);
	if (ret < 0)
		goto err_free;

	parport_claim_or_block(slot->par_device);

	list_add(&slot->list, &slots);

	pr_devel("%s end\n", __func__);
	return 0;

err_free:
	slot_free(slot);
//err:
	return ret;
}

static int remove(struct platform_device *pdev)
{
	struct parport_gpio *slot;

	pr_devel("%s()\n", __func__);

	slot = platform_get_drvdata(pdev);
	gpiochip_remove(&slot->gpio);
	slot->gpio_device = 0;

	pr_devel("%s end\n", __func__);
	return 0;
}

static struct parport_driver parport_driver = {
	.name = DRV_NAME,
	.match_port = attach,
	.detach = detach,
	.devmodel = true
};

static const struct of_device_id of_match_table[] = {
	{ .compatible = DRV_NAME },
	{},
};
MODULE_DEVICE_TABLE(of, of_match_table);

static struct platform_driver gpio_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_table,
	},
	.probe = probe,
	.remove = remove
};

static int __init parport_gpio_init(void)
{
	int ret;
	int i;

	pr_devel("%s()\n", __func__);

	for (i = 0; param_parport[i]; i++) {
		char *s = param_parport[i];
		char *e;
		unsigned long v = simple_strtoul(s, &e, 0);
		if (*e || v >= 8*sizeof(parports_to_claim)) {
			pr_err("Should be a parport number: \"%s\"\n", s);
			return -ENODEV;
		}
		parports_to_claim |= 1 << v;
	}

	if (!parports_to_claim)
		parports_to_claim = 1 << 0; // parport0 by default

	if (parport_register_driver(&parport_driver)) {
		pr_err("Unable to register with parport driver\n");
		return -EIO;
	}

	ret = platform_driver_register(&gpio_driver);
	if (ret)
		goto err_unreg_parport_driver;

	pr_devel("%s end\n", __func__);
	return 0;

err_unreg_parport_driver:
	parport_unregister_driver(&parport_driver);
	return ret;
}

static void __exit parport_gpio_exit(void)
{
	pr_devel("%s()\n", __func__);

	parport_unregister_driver(&parport_driver);
	platform_driver_unregister(&gpio_driver);

	pr_devel("%s end\n", __func__);
}

module_init(parport_gpio_init);
module_exit(parport_gpio_exit);

MODULE_AUTHOR("Jan Bobrowski <jb@torinak.com>");
MODULE_LICENSE("GPL");
