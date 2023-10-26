// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bouffalo Lab SoC pinctrl+GPIO+external IRQ driver
 *
 * Based on: pinctrl-apple-gpio.c
 * Copyright (C) The Asahi Linux Contributors
 * Copyright (C) 2020 Corellium LLC
 *
 * Based on: pinctrl-pistachio.c
 * Copyright (C) 2014 Imagination Technologies Ltd.
 * Copyright (C) 2014 Google, Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include "pinctrl-utils.h"
#include "core.h"
#include "pinmux.h"

struct bflb_gpio_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctldev;

	void __iomem *base;
	struct regmap *map;

	struct pinctrl_desc pinctrl_desc;
	struct gpio_chip gpio_chip;

	void *irqsunmasked;
	u8 irqgrps[];
};

//Register indexing
#define REG_GPIO(x)              (4 * (x))

//Register map
#define REG_GPIOx_MODE           GENMASK(31, 30)
#define REG_GPIOx_I              BIT(28)
#define REG_GPIOx_CLR            BIT(26)
#define REG_GPIOx_SET            BIT(25)
#define REG_GPIOx_O              BIT(24)
#define REG_GPIOx_INT_MASK       BIT(22)
#define REG_GPIOx_INT_STAT       BIT(21)
#define REG_GPIOx_INT_CLR        BIT(20)
#define REG_GPIOx_INT_MODE_SET   GENMASK(29, 16)
#define REG_GPIOx_FUNC_SEL       GENMASK(12, 8)
#define REG_GPIOx_OE             BIT(6)
#define REG_GPIOx_PD             BIT(5)
#define REG_GPIOx_PU             BIT(4)
#define REG_GPIOx_DRV            GENMASK(3, 2)
#define REG_GPIOx_SMT            BIT(1)
#define REG_GPIOx_IE             BIT(0)

//Interrupt trigger modes
#define BFLB_IRQ_MODE_SYNC_EDGE_FALLING  0
#define BFLB_IRQ_MODE_SYNC_EDGE_RISING   1
#define BFLB_IRQ_MODE_SYNC_LEVEL_LOW     2
#define BFLB_IRQ_MODE_SYNC_LEVEL_HIGH    3
#define BFLB_IRQ_MODE_SYNC_EDGE_BOTH     4

#define BFLB_IRQ_MODE_ASYNC_EDGE_FALLING 8
#define BFLB_IRQ_MODE_ASYNC_EDGE_RISING  9
#define BFLB_IRQ_MODE_ASYNC_LEVEL_LOW    10
#define BFLB_IRQ_MODE_ASYNC_LEVEL_HIGH   11

static const char * const pinmux_functions[] = {
	//AH: As taken from smaeul's pinctrl-bflb.c for U-Boot
	[0]	    = "sdh",
	[1]	    = "spi0",
	[2]	    = "flash",
	[3]	    = "i2s",
	[4]	    = "pdm",
	[5]	    = "i2c0",
	[6]	    = "i2c1",
	[7]	    = "uart",
	[8]	    = "emac",
	[9]	    = "cam",
	[10]	= "analog",
	[11]	= "gpio",
	[16]	= "pwm0",
	[17]	= "pwm1",
	[18]	= "spi1",	// mm_spi
	[19]	= "i2c2",	// mm_i2c0
	[20]	= "i2c3",	// mm_i2c1
	[21]	= "mm_uart",
	[22]	= "dbi_b",
	[23]	= "dbi_c",
	[24]	= "dpi",
	[25]	= "jtag_lp",
	[26]	= "jtag_m0",
	[27]	= "jtag_d0",
	[31]	= "clock",
};

struct regmap_config regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.cache_type = REGCACHE_FLAT,
	.max_register = 512 * sizeof(u32),
	.num_reg_defaults_raw = 512,
	.use_relaxed_mmio = true,
	.use_raw_spinlock = true,
};

//AH: Set raw gpio config register bits based on mask
static void bflb_gpio_set_reg(struct bflb_gpio_pinctrl *pctl, unsigned int pin,
		u32 mask, u32 value)
{
	regmap_update_bits(pctl->map, REG_GPIO(pin), mask, value);
}

//AH: Get raw gpio config register bits
static u32 bflb_gpio_get_reg(struct bflb_gpio_pinctrl *pctl, unsigned int pin)
{
	int ret;
	u32 val;

	ret = regmap_read(pctl->map, REG_GPIO(pin), &val);

	if (ret)
		return 0;

	return val;
}

/* Pin controller functions */

static const struct pinctrl_ops bflb_gpio_pinctrl_ops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name   = pinctrl_generic_get_group_name,
	.get_group_pins   = pinctrl_generic_get_group_pins,
	.dt_node_to_map   = pinconf_generic_dt_node_to_map_group,
	.dt_free_map      = pinconf_generic_dt_free_map,
};

/* Pin multiplexer functions */

//AH: Configure gpio modes and features
static int bflb_gpio_pinmux_set(struct pinctrl_dev *pctldev, unsigned int func,
		unsigned int group)
{
	struct bflb_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	bflb_gpio_set_reg(pctl, group, REG_GPIOx_FUNC_SEL,
		FIELD_PREP(REG_GPIOx_FUNC_SEL, func));

	dev_dbg(pctl->dev, "Pin %u set to function %u", group, func);

	return 0;
}

static const struct pinmux_ops bflb_gpio_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = bflb_gpio_pinmux_set,
	.strict = true,
};

/* GPIO chip functions */

//AH: Get the current gpio direction
static int bflb_gpio_get_direction(struct gpio_chip *chip,
		unsigned int offset)
{
	struct bflb_gpio_pinctrl *pctl = gpiochip_get_data(chip);
	unsigned int reg = bflb_gpio_get_reg(pctl, offset);

	if (FIELD_GET(REG_GPIOx_OE, reg) == 1 &&
			FIELD_GET(REG_GPIOx_IE, reg) == 0) {
		return GPIO_LINE_DIRECTION_OUT;
	} else if (FIELD_GET(REG_GPIOx_IE, reg) == 1 &&
			FIELD_GET(REG_GPIOx_OE, reg) == 0) {
		return GPIO_LINE_DIRECTION_IN;
	}

	return -EIO;
}

//AH: Get the incoming input or outgoing output value for the specified GPIO
static int bflb_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct bflb_gpio_pinctrl *pctl = gpiochip_get_data(chip);
	unsigned int reg = bflb_gpio_get_reg(pctl, offset);

	if (FIELD_GET(REG_GPIOx_OE, reg) == 1 &&
			FIELD_GET(REG_GPIOx_IE, reg) == 0) {
		reg = readl_relaxed(pctl->base + REG_GPIO(offset));
		return !!(reg & REG_GPIOx_O);
	} else if (FIELD_GET(REG_GPIOx_IE, reg) == 1 &&
			FIELD_GET(REG_GPIOx_OE, reg) == 0) {
		reg = readl_relaxed(pctl->base + REG_GPIO(offset));
		return !!(reg & REG_GPIOx_I);
	}

	return -EIO;
}

//AH: Set the specified GPIO's output state
static void bflb_gpio_set(struct gpio_chip *chip, unsigned int offset,
		int value)
{
	struct bflb_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	bflb_gpio_set_reg(pctl, offset, REG_GPIOx_O, value ?
		FIELD_PREP(REG_GPIOx_O, 1) : 0);

	dev_dbg(pctl->dev, "Pin %u set to value %u", offset, value);
}

//AH: Set the specified gpio direction to input
static int bflb_gpio_direction_input(struct gpio_chip *chip,
		unsigned int offset)
{
	struct bflb_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	bflb_gpio_set_reg(pctl, offset, REG_GPIOx_OE | REG_GPIOx_IE | REG_GPIOx_SMT,
		FIELD_PREP(REG_GPIOx_OE, 0) | FIELD_PREP(REG_GPIOx_IE, 1) |
		FIELD_PREP(REG_GPIOx_SMT, 1));

	dev_dbg(pctl->dev, "Pin %u set to direction input", offset);

	return 0;
}

//AH: Set the specified gpio direction to output
static int bflb_gpio_direction_output(struct gpio_chip *chip,
		unsigned int offset, int value)
{
	struct bflb_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	bflb_gpio_set_reg(pctl, offset, REG_GPIOx_PD | REG_GPIOx_PU | REG_GPIOx_OE |
		REG_GPIOx_IE | REG_GPIOx_SMT | REG_GPIOx_MODE,
		FIELD_PREP(REG_GPIOx_PD, 0) | FIELD_PREP(REG_GPIOx_PU, 0) |
		FIELD_PREP(REG_GPIOx_OE, 1) | FIELD_PREP(REG_GPIOx_IE, 0) |
		FIELD_PREP(REG_GPIOx_SMT, 1) | FIELD_PREP(REG_GPIOx_MODE, 0));

	dev_dbg(pctl->dev, "Pin %u set to direction output", offset);

	bflb_gpio_set(chip, offset, value); //Set the initially passed value

	return 0;
}

//AH: Configure pin electrical characteristics
static int bflb_gpio_set_config(struct gpio_chip *chip, unsigned int offset,
		unsigned long config)
{
	struct bflb_gpio_pinctrl *pctl = gpiochip_get_data(chip);
	enum pin_config_param param = pinconf_to_config_param(config);
	unsigned int arg = pinconf_to_config_argument(config);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		bflb_gpio_set_reg(pctl, offset, REG_GPIOx_PD | REG_GPIOx_PU,
			FIELD_PREP(REG_GPIOx_PD, 0) | FIELD_PREP(REG_GPIOx_PU, 0));
		break;

	case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
		if (arg) {
			bflb_gpio_set_reg(pctl, offset, REG_GPIOx_PD | REG_GPIOx_PU,
				FIELD_PREP(REG_GPIOx_PD, 0) | FIELD_PREP(REG_GPIOx_PU, 0));
		}
		break;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (arg) {
			bflb_gpio_set_reg(pctl, offset, REG_GPIOx_PD | REG_GPIOx_PU,
				FIELD_PREP(REG_GPIOx_PD, 1) | FIELD_PREP(REG_GPIOx_PU, 0));
		}
		break;

	case PIN_CONFIG_BIAS_PULL_UP:
		if (arg) {
			bflb_gpio_set_reg(pctl, offset, REG_GPIOx_PD | REG_GPIOx_PU,
				FIELD_PREP(REG_GPIOx_PD, 0) | FIELD_PREP(REG_GPIOx_PU, 1));
		}
		break;

	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		bflb_gpio_set_reg(pctl, offset, REG_GPIOx_PD | REG_GPIOx_PU |
			REG_GPIOx_IE | REG_GPIOx_OE, FIELD_PREP(REG_GPIOx_PD, 0) |
			FIELD_PREP(REG_GPIOx_PU, 0) | FIELD_PREP(REG_GPIOx_IE, 0) |
			FIELD_PREP(REG_GPIOx_OE, 0));
		break;

	case PIN_CONFIG_INPUT_ENABLE:
		bflb_gpio_set_reg(pctl, offset, REG_GPIOx_IE,
			FIELD_PREP(REG_GPIOx_IE, !!arg));
		break;

	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		bflb_gpio_set_reg(pctl, offset, REG_GPIOx_SMT,
			FIELD_PREP(REG_GPIOx_SMT, !!arg));
		break;

	case PIN_CONFIG_OUTPUT:
		bflb_gpio_set_reg(pctl, offset, REG_GPIOx_OE | REG_GPIOx_O,
			FIELD_PREP(REG_GPIOx_OE, 1) | FIELD_PREP(REG_GPIOx_O, !!arg));
		break;

	case PIN_CONFIG_OUTPUT_ENABLE:
		bflb_gpio_set_reg(pctl, offset, REG_GPIOx_OE,
			FIELD_PREP(REG_GPIOx_OE, 1));
		break;

	default: return -ENOTSUPP;
	}

	dev_dbg(pctl->dev, "Pin %u config set to %lu (param %u, arg %u)", offset,
		config, param, arg);

	return 0;
}

/* IRQ chip functions */

//AH: Clear the interrupt for the specified GPIO
static void bflb_gpio_irq_ack(struct irq_data *data)
{
	struct bflb_gpio_pinctrl *pctl =
			gpiochip_get_data(irq_data_get_irq_chip_data(data));

	bflb_gpio_set_reg(pctl, data->hwirq, REG_GPIOx_INT_CLR,
	    FIELD_PREP(REG_GPIOx_INT_CLR, 1));

	bflb_gpio_set_reg(pctl, data->hwirq, REG_GPIOx_INT_CLR,
	    FIELD_PREP(REG_GPIOx_INT_CLR, 0));

	dev_dbg(pctl->dev, "Pin %lu IRQ ACK", data->hwirq);
}

//AH: Find the correct value for the type of interrupts we want to receive
//for a GPIO
static unsigned int bflb_gpio_irq_type(unsigned int type)
{
	unsigned int selected;

	switch (type & IRQ_TYPE_SENSE_MASK) {

	case IRQ_TYPE_EDGE_RISING:
		selected = BFLB_IRQ_MODE_SYNC_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		selected = BFLB_IRQ_MODE_SYNC_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		selected = BFLB_IRQ_MODE_SYNC_EDGE_BOTH;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		selected = BFLB_IRQ_MODE_SYNC_LEVEL_HIGH;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		selected = BFLB_IRQ_MODE_SYNC_LEVEL_LOW;
		break;

	//No "off" available on BL808, set to default IRQ_TYPE_EDGE_FALLING and
	//then we'll need to mask
	default:
		selected = BFLB_IRQ_MODE_SYNC_EDGE_FALLING;
		break;
	}

	return selected;
}

//AH: Disable the specified GPIO's interrupt
static void bflb_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct bflb_gpio_pinctrl *pctl = gpiochip_get_data(gc);

	bflb_gpio_set_reg(pctl, data->hwirq, REG_GPIOx_INT_MASK,
	    FIELD_PREP(REG_GPIOx_INT_MASK, 1));

	clear_bit(data->hwirq, pctl->irqsunmasked);
	gpiochip_disable_irq(gc, data->hwirq);

	dev_dbg(pctl->dev, "Pin %lu IRQ Mask", data->hwirq);
}

//AH: Enable the specified GPIO's interrupt
static void bflb_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct bflb_gpio_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int irqtype = bflb_gpio_irq_type(irqd_get_trigger_type(data));

	gpiochip_enable_irq(gc, data->hwirq);
	set_bit(data->hwirq, pctl->irqsunmasked);

	bflb_gpio_set_reg(pctl, data->hwirq, REG_GPIOx_INT_MASK |
		REG_GPIOx_INT_MODE_SET, FIELD_PREP(REG_GPIOx_INT_MASK, 0) |
		FIELD_PREP(REG_GPIOx_INT_MODE_SET, irqtype));

	dev_dbg(pctl->dev, "Pin %lu IRQ Unmask", data->hwirq);
}

//AH: Initialise the specified GPIO's interrupt
static unsigned int bflb_gpio_irq_startup(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct bflb_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	bflb_gpio_set_reg(pctl, data->hwirq, REG_GPIOx_INT_CLR,
	    FIELD_PREP(REG_GPIOx_INT_CLR, 1));

	bflb_gpio_set_reg(pctl, data->hwirq, REG_GPIOx_INT_CLR,
	    FIELD_PREP(REG_GPIOx_INT_CLR, 0));

	bflb_gpio_irq_unmask(data);

	dev_dbg(pctl->dev, "Pin %lu IRQ Started", data->hwirq);

	return 0;
}

//AH: Set the specified GPIO's interrupt mode
static int bflb_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct bflb_gpio_pinctrl *pctl =
			gpiochip_get_data(irq_data_get_irq_chip_data(data));

	unsigned int irqtype = bflb_gpio_irq_type(type);

	if (irqtype == 0)
		return -EINVAL;

	bflb_gpio_set_reg(pctl, data->hwirq, REG_GPIOx_INT_MODE_SET,
		FIELD_PREP(REG_GPIOx_INT_MODE_SET, irqtype));

	if (type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(data, handle_level_irq);
	else
		irq_set_handler_locked(data, handle_edge_irq);

	dev_dbg(pctl->dev, "Pin %lu IRQ type set to %u", data->hwirq, irqtype);

	return 0;
}

//AH: Handle GPIO interrupts on this controller
static void bflb_gpio_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u8 *grpp = irq_desc_get_handler_data(desc);
	struct bflb_gpio_pinctrl *pctl;
	unsigned int pinh;
	unsigned long reg;
	struct gpio_chip *gc;

	pctl = container_of(grpp - *grpp, typeof(*pctl), irqgrps[0]);
	gc = &pctl->gpio_chip;

	chained_irq_enter(chip, desc);

	//We must go through each individual GPIO register to read its interrupt
	//status. There is no gpio_cfg128+ helper register for interrupts
	//(looking at BL808 RM)
	for (pinh = 0; pinh < gc->ngpio; pinh += 1) {
		if (test_bit(pinh, pctl->irqsunmasked)) {
			dev_dbg(pctl->dev, "Reading IRQ status of pin %u", pinh);

			reg = readl_relaxed(pctl->base + REG_GPIO(pinh));

			if (reg & REG_GPIOx_INT_STAT) {
				generic_handle_domain_irq(gc->irq.domain, pinh);
				dev_dbg(pctl->dev, "Pin %u IRQ Fire", pinh);
			}
		} else {
			dev_dbg(pctl->dev, "Ignoring IRQ status of masked pin %u", pinh);
		}
	}

	chained_irq_exit(chip, desc);
}

static const struct irq_chip bflb_gpio_irqchip = {
	.name			= "bflb-gpio",
	.irq_startup    = bflb_gpio_irq_startup,
	.irq_ack		= bflb_gpio_irq_ack,
	.irq_mask		= bflb_gpio_irq_mask,
	.irq_unmask		= bflb_gpio_irq_unmask,
	.irq_set_type	= bflb_gpio_irq_set_type,
	.flags			= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int bflb_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	int ret;
	struct bflb_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	ret = pinctrl_gpio_request(chip->base + offset);

	if (ret)
		return ret;

	bflb_gpio_set_reg(pctl, offset, REG_GPIOx_FUNC_SEL,
		FIELD_PREP(REG_GPIOx_FUNC_SEL, 11/*SWGPIO*/));

	dev_dbg(pctl->dev, "Pin %u set to function GPIO as part of request",
			offset);

	return 0;
}

/* Probe & register */

static int bflb_gpio_register(struct bflb_gpio_pinctrl *pctl)
{
	struct gpio_irq_chip *girq = &pctl->gpio_chip.irq;
	void **irq_data = NULL;
	int ret;

	pctl->gpio_chip.label = dev_name(pctl->dev);
	pctl->gpio_chip.request = bflb_gpio_request;
	pctl->gpio_chip.free = gpiochip_generic_free;
	pctl->gpio_chip.get_direction = bflb_gpio_get_direction;
	pctl->gpio_chip.direction_input = bflb_gpio_direction_input;
	pctl->gpio_chip.direction_output = bflb_gpio_direction_output;
	pctl->gpio_chip.get = bflb_gpio_get;
	pctl->gpio_chip.set = bflb_gpio_set;
	pctl->gpio_chip.set_config = bflb_gpio_set_config;
	pctl->gpio_chip.base = -1;
	pctl->gpio_chip.ngpio = pctl->pinctrl_desc.npins;
	pctl->gpio_chip.parent = pctl->dev;

	if (girq->num_parents) {
		int i;

		gpio_irq_chip_set_chip(girq, &bflb_gpio_irqchip);

		girq->parent_handler = bflb_gpio_irq_handler;

		girq->parents = kmalloc_array(girq->num_parents, sizeof(*girq->parents),
				GFP_KERNEL);
		irq_data = kmalloc_array(girq->num_parents, sizeof(*irq_data),
				GFP_KERNEL);

		if (!girq->parents || !irq_data) {
			ret = -ENOMEM;
			goto out_free_irq_data;
		}

		for (i = 0; i < girq->num_parents; i++) {
			ret = platform_get_irq(to_platform_device(pctl->dev), i);

			if (ret < 0)
				goto out_free_irq_data;

			girq->parents[i] = ret;
			pctl->irqgrps[i] = i;
			irq_data[i] = &pctl->irqgrps[i];
		}

		girq->parent_handler_data_array = irq_data;
		girq->per_parent_data = true;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_level_irq;
	}

	ret = devm_gpiochip_add_data(pctl->dev, &pctl->gpio_chip, pctl);

out_free_irq_data:
	kfree(girq->parents);
	kfree(irq_data);

	return ret;
}

static int bflb_gpio_pinctrl_probe(struct platform_device *pdev)
{
	struct bflb_gpio_pinctrl *pctl;
	struct pinctrl_pin_desc *pins;

	unsigned int npins;
	const char **pin_names;
	unsigned int *pin_nums;
	unsigned int i, nirqs = 0;
	int res;

	if (of_property_read_bool(pdev->dev.of_node, "interrupt-controller")) {
		res = platform_irq_count(pdev);
		if (res > 0)
			nirqs = res;
	}

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);

	if (!pctl)
		return -ENOMEM;

	pctl->dev = &pdev->dev;
	pctl->gpio_chip.irq.num_parents = nirqs;

	dev_set_drvdata(&pdev->dev, pctl);

	if (of_property_read_u32(pdev->dev.of_node, "bflb,npins", &npins))
		return dev_err_probe(&pdev->dev, -EINVAL,
				"bflb,npins property not found\n");

	pctl->irqsunmasked = devm_bitmap_zalloc(&pdev->dev, npins, GFP_KERNEL);

	pins = devm_kmalloc_array(&pdev->dev, npins, sizeof(pins[0]), GFP_KERNEL);
	pin_names = devm_kmalloc_array(&pdev->dev, npins, sizeof(pin_names[0]),
			GFP_KERNEL);
	pin_nums = devm_kmalloc_array(&pdev->dev, npins, sizeof(pin_nums[0]),
			GFP_KERNEL);

	if (!pins || !pin_names || !pin_nums)
		return -ENOMEM;

	pctl->base = devm_platform_ioremap_resource(pdev, 0);

	if (IS_ERR(pctl->base))
		return PTR_ERR(pctl->base);

	pctl->map = devm_regmap_init_mmio(&pdev->dev, pctl->base, &regmap_config);

	if (IS_ERR(pctl->map))
		return dev_err_probe(&pdev->dev, PTR_ERR(pctl->map),
				"Failed to create regmap\n");

	for (i = 0; i < npins; i++) {
		pins[i].number = i;
		pins[i].name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "GPIO%u", i);
		pins[i].drv_data = pctl;
		pin_names[i] = pins[i].name;
		pin_nums[i] = i;
	}

	pctl->pinctrl_desc.name = dev_name(pctl->dev);
	pctl->pinctrl_desc.pins = pins;
	pctl->pinctrl_desc.npins = npins;
	pctl->pinctrl_desc.pctlops = &bflb_gpio_pinctrl_ops;
	pctl->pinctrl_desc.pmxops = &bflb_gpio_pinmux_ops;

	pctl->pctldev =	devm_pinctrl_register(&pdev->dev, &pctl->pinctrl_desc,
			pctl);

	if (IS_ERR(pctl->pctldev))
		return dev_err_probe(&pdev->dev, PTR_ERR(pctl->pctldev),
				"Failed to register pinctrl device.\n");

	for (i = 0; i < npins; i++) {
		res = pinctrl_generic_add_group(pctl->pctldev,
				pins[i].name, pin_nums + i, 1, pctl);

		dev_dbg(&pdev->dev, "Registered pin %s with numeric %u",
				pins[i].name, i);

		if (res < 0)
			return dev_err_probe(pctl->dev, res, "Failed to register group");
	}

	for (i = 0; i < ARRAY_SIZE(pinmux_functions); ++i) {
		if (pinmux_functions[i]) {
			res = pinmux_generic_add_function(pctl->pctldev,
					pinmux_functions[i], pin_names, npins, pctl);

			dev_dbg(&pdev->dev, "Registered function %s with numeric %u",
					pinmux_functions[i], i);

			if (res < 0)
				return dev_err_probe(pctl->dev, res,
						"Failed to register function.");
		}
	}

	dev_info(&pdev->dev, "Bouffalo Lab pinctrl+GPIO(+interrupt) controller - "
			"Registered %lu function(s) for %u pin(s)",

	ARRAY_SIZE(pinmux_functions), npins);

	return bflb_gpio_register(pctl);
}

static const struct of_device_id bflb_gpio_pinctrl_of_match[] = {
	{ .compatible = "bflb,pinctrl", },
	{ }
};

MODULE_DEVICE_TABLE(of, bflb_gpio_pinctrl_of_match);

static struct platform_driver bflb_gpio_pinctrl_driver = {
	.driver = {
		.name = "bflb-gpio-pinctrl",
		.of_match_table = bflb_gpio_pinctrl_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = bflb_gpio_pinctrl_probe,
};

module_platform_driver(bflb_gpio_pinctrl_driver);

MODULE_DESCRIPTION("Bouffalo BL808 pinctrl/GPIO driver");
MODULE_AUTHOR("Alexander Horner <contact@alexhorner.cc>");
MODULE_LICENSE("GPL v2");
