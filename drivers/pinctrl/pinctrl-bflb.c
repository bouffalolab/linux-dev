// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bouffalo Lab SoC pinctrl + GPIO + external IRQ controller driver
 *
 * 2023.11 - BL808
 *
 * Copyright (C) 2016-2023
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
#include <linux/pinctrl/pinconf.h>
#include <dt-bindings/pinctrl/pinctrl-bflb.h>
#include "pinctrl-utils.h"
#include "core.h"
#include "pinmux.h"
#include "pinconf.h"

struct bflb_pinfunction {
	const char *name;
	const char **groups;
	unsigned int ngroups;
	/* capacity of group memory */
	unsigned int ncap;
	/* the value to be programmed into register */
	u32 val;
};

struct bflb_pinconf {
	unsigned long *configs;
	unsigned int nconfigs;
};

struct bflb_pingroup {
	const char *name;
	unsigned int *pins;
	size_t npins;
	/*
	 * this is built when driver probes device and only used in dt_node_to_map
	 * to accelerate the process.
	 */
	struct bflb_pinconf *pinconf;
	/* the function value of the group */
	u32 fn_val;
};

struct bflb_pinctrl_info {
	struct device *dev;
	struct pinctrl_dev *pctldev;

	void __iomem *base;
	struct regmap *map;

	struct pinctrl_desc pinctrl_desc;
	struct gpio_chip gpio_chip;

	/* pin groups and functions */
	struct bflb_pingroup *groups;
	int ngroups;
	struct bflb_pinfunction *functions;
	int nfunctions;

	void *unmasked_irqs;
};

/* register indexing */
#define REG_PIN(x)              (4 * (x))

/* register map */
#define REG_PIN_MODE_MASK      GENMASK(31, 30)
#define REG_PIN_IN_HIGH        BIT(28)
#define REG_PIN_CLR            BIT(26)
#define REG_PIN_SET            BIT(25)
#define REG_PIN_OUT_HIGH       BIT(24)
#define REG_PIN_INT_MASK       BIT(22)
#define REG_PIN_INT_STAT       BIT(21)
#define REG_PIN_INT_CLR        BIT(20)
#define REG_PIN_INT_MODE_SET   GENMASK(19, 16)
#define REG_PIN_FUNC_SEL       GENMASK(12, 8)
#define REG_PIN_OE             BIT(6)
#define REG_PIN_PD             BIT(5)
#define REG_PIN_PU             BIT(4)
#define REG_PIN_DRV            GENMASK(3, 2)
#define REG_PIN_SMT            BIT(1)
#define REG_PIN_IE             BIT(0)

/* interrupt trigger modes */
#define BFLB_IRQ_MODE_SYNC_EDGE_FALLING  0
#define BFLB_IRQ_MODE_SYNC_EDGE_RISING   1
#define BFLB_IRQ_MODE_SYNC_LEVEL_LOW     2
#define BFLB_IRQ_MODE_SYNC_LEVEL_HIGH    3
#define BFLB_IRQ_MODE_SYNC_EDGE_BOTH     4

#define BFLB_IRQ_MODE_ASYNC_EDGE_FALLING 8
#define BFLB_IRQ_MODE_ASYNC_EDGE_RISING  9
#define BFLB_IRQ_MODE_ASYNC_LEVEL_LOW    10
#define BFLB_IRQ_MODE_ASYNC_LEVEL_HIGH   11

static const struct pinctrl_pin_desc bl808_pins[] = {
	PINCTRL_PIN(0, "PIN0"),   PINCTRL_PIN(1, "PIN1"),
	PINCTRL_PIN(2, "PIN2"),   PINCTRL_PIN(3, "PIN3"),
	PINCTRL_PIN(4, "PIN4"),   PINCTRL_PIN(5, "PIN5"),
	PINCTRL_PIN(6, "PIN6"),   PINCTRL_PIN(7, "PIN7"),
	PINCTRL_PIN(8, "PIN8"),   PINCTRL_PIN(9, "PIN9"),
	PINCTRL_PIN(10, "PIN10"), PINCTRL_PIN(11, "PIN11"),
	PINCTRL_PIN(12, "PIN12"), PINCTRL_PIN(13, "PIN13"),
	PINCTRL_PIN(14, "PIN14"), PINCTRL_PIN(15, "PIN15"),
	PINCTRL_PIN(16, "PIN16"), PINCTRL_PIN(17, "PIN17"),
	PINCTRL_PIN(18, "PIN18"), PINCTRL_PIN(19, "PIN19"),
	PINCTRL_PIN(20, "PIN20"), PINCTRL_PIN(21, "PIN21"),
	PINCTRL_PIN(22, "PIN22"), PINCTRL_PIN(23, "PIN23"),
	PINCTRL_PIN(24, "PIN24"), PINCTRL_PIN(25, "PIN25"),
	PINCTRL_PIN(26, "PIN26"), PINCTRL_PIN(27, "PIN27"),
	PINCTRL_PIN(28, "PIN28"), PINCTRL_PIN(29, "PIN29"),
	PINCTRL_PIN(30, "PIN30"), PINCTRL_PIN(31, "PIN31"),
	PINCTRL_PIN(32, "PIN32"), PINCTRL_PIN(33, "PIN33"),
	PINCTRL_PIN(34, "PIN34"), PINCTRL_PIN(35, "PIN35"),
	PINCTRL_PIN(36, "PIN36"), PINCTRL_PIN(37, "PIN37"),
	PINCTRL_PIN(38, "PIN38"), PINCTRL_PIN(39, "PIN39"),
	PINCTRL_PIN(40, "PIN40"), PINCTRL_PIN(41, "PIN41"),
	PINCTRL_PIN(42, "PIN42"), PINCTRL_PIN(43, "PIN43"),
	PINCTRL_PIN(44, "PIN44"), PINCTRL_PIN(45, "PIN45")
};

struct bflb_function_desc {
	unsigned int val;
	const char *name;
};

static const struct bflb_function_desc bl808_functions[] = {
	{BFLB_PINMUX_FUNC_SDH,			"sdh"},
	{BFLB_PINMUX_FUNC_SPI0,			"spi0"},
	{BFLB_PINMUX_FUNC_QSPI_FLASH,	"qspi_flash"},
	{BFLB_PINMUX_FUNC_I2S,			"i2s"},
	{BFLB_PINMUX_FUNC_PDM,			"pdm"},
	{BFLB_PINMUX_FUNC_I2C0,			"i2c0"},
	{BFLB_PINMUX_FUNC_I2C1,			"i2c1"},
	{BFLB_PINMUX_FUNC_UART,			"uart"},
	{BFLB_PINMUX_FUNC_EMAC,			"emac"},
	{BFLB_PINMUX_FUNC_CAM,			"cam"},
	{BFLB_PINMUX_FUNC_ANALOG,		"analog"},
	{BFLB_PINMUX_FUNC_GPIO,			"gpio"},
	{BFLB_PINMUX_FUNC_RF_ADDA,		"rf_adda"},
	{BFLB_PINMUX_FUNC_SCAN_TEST,	"scan_test"},
	{BFLB_PINMUX_FUNC_AUDIO_TEST,	"audio_test"},
	{BFLB_PINMUX_FUNC_DEBUG,		"debug"},
	{BFLB_PINMUX_FUNC_PWM0,			"pwm0"},
	{BFLB_PINMUX_FUNC_PWM1,			"pwm1"},
	{BFLB_PINMUX_FUNC_MM_SPI,		"mm_spi"},
	{BFLB_PINMUX_FUNC_MM_I2C0,		"mm_i2c0"},
	{BFLB_PINMUX_FUNC_MM_I2C1,		"mm_i2c1"},
	{BFLB_PINMUX_FUNC_MM_UART,		"mm_uart"},
	{BFLB_PINMUX_FUNC_DBI_B,		"dbi_b"},
	{BFLB_PINMUX_FUNC_DBI_C,		"dbi_c"},
	{BFLB_PINMUX_FUNC_DPI,			"dpi"},
	{BFLB_PINMUX_FUNC_JTAG_LP,		"jtag_lp"},
	{BFLB_PINMUX_FUNC_JTAG_M0,		"jtag_m0"},
	{BFLB_PINMUX_FUNC_JTAG_D0,		"jtag_d0"},
	{BFLB_PINMUX_FUNC_RF_EXT_PA,	"rf_ext_pa"},
	{BFLB_PINMUX_FUNC_ANT_SW,		"ant_switch"},
	{BFLB_PINMUX_FUNC_USB_TEST,		"usb_test"},
	{BFLB_PINMUX_FUNC_CLOCK,		"clock"},
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

static int bflb_pinctrl_get_group_count(struct pinctrl_dev *pctldev)
{
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);

	return bpctl->ngroups;
}

static const char *bflb_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
		unsigned int selector)
{
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);

	return bpctl->groups[selector].name;
}

static int bflb_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
		unsigned int selector, const unsigned int **pins, unsigned int *num_pins)
{
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);

	*pins = bpctl->groups[selector].pins;
	*num_pins = bpctl->groups[selector].npins;
	return 0;
}

static struct bflb_pingroup *bflb_pinctrl_name_to_group(
	struct bflb_pinctrl_info *bpctrl, const char *name)
{
	int i;
	struct bflb_pingroup *grp;

	for (i = 0; i < bpctrl->ngroups; i++) {
		grp = &bpctrl->groups[i];
		if (!strcmp(name, grp->name))
			return grp;
	}
	return NULL;
}

static struct bflb_pinfunction *bflb_pinctrl_parse_function(
		struct bflb_pinctrl_info *bpctl, u32 func_val)
{
	int i;

	for (i = 0; i < bpctl->nfunctions; i++) {
		if (func_val == bpctl->functions[i].val)
			return &bpctl->functions[i];
	}

	return NULL;
}

static int bflb_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
		struct device_node *np, struct pinctrl_map **map, unsigned int *num_maps)
{
	int err;
	struct bflb_pingroup *grp;
	struct bflb_pinfunction *func;
	unsigned int i, reserve, map_idx = 0;
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);

	*map = NULL;
	*num_maps = 0;

	grp = bflb_pinctrl_name_to_group(bpctl, np->name);
	if (!grp) {
		dev_err(bpctl->dev, "unable to find group for node %pOFn\n", np);
		return -EINVAL;
	}

	/* pin group mux map + pin config map x npins */
	reserve = 1 + grp->npins;
	err = pinctrl_utils_reserve_map(pctldev, map, num_maps, &map_idx, reserve);
	if (err)
		return err;

	func = bflb_pinctrl_parse_function(bpctl, grp->fn_val);
	err = pinctrl_utils_add_map_mux(pctldev, map, num_maps, &map_idx, grp->name,
									func->name);
	if (err)
		goto exit;

	for (i = 0; i < grp->npins; i++) {
		struct bflb_pinconf *conf = &grp->pinconf[i];
		const char *pin_name = pin_get_name(pctldev, grp->pins[i]);

		err = pinctrl_utils_add_map_configs(pctldev, map, num_maps, &map_idx,
				pin_name, conf->configs, conf->nconfigs,
				PIN_MAP_TYPE_CONFIGS_PIN);
		if (err)
			goto exit;
	}

	*num_maps = map_idx;
	return 0;

exit:
	pinctrl_utils_free_map(pctldev, *map, map_idx);
	return err;
}

/* pin controller functions */
static const struct pinctrl_ops bflb_pinctrl_ops = {
	.get_groups_count = bflb_pinctrl_get_group_count,
	.get_group_name   = bflb_pinctrl_get_group_name,
	.get_group_pins   = bflb_pinctrl_get_group_pins,
	.dt_node_to_map   = bflb_pinctrl_dt_node_to_map,
	.dt_free_map      = pinctrl_utils_free_map,
};

/* pin multiplexer functions */
static int bflb_pinmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);

	return bpctl->nfunctions;
}

static const char *bflb_pinmux_get_function_name(struct pinctrl_dev *pctldev,
					  unsigned int selector)
{
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);

	return bpctl->functions[selector].name;
}

static int bflb_pinmux_get_function_groups(struct pinctrl_dev *pctldev,
		unsigned int selector, const char * const **groups, unsigned int *num_groups)
{
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);
	struct bflb_pinfunction *func = &bpctl->functions[selector];

	*groups = func->groups;
	*num_groups = func->ngroups;
	return 0;
}

static int bflb_pinmux_set(struct pinctrl_dev *pctldev, unsigned int selector,
		unsigned int group)
{
	unsigned int i;
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);
	struct bflb_pinfunction *func = &bpctl->functions[selector];
	struct bflb_pingroup *grp = &bpctl->groups[group];

	for (i = 0; i < grp->npins; i++) {
		regmap_update_bits(bpctl->map, REG_PIN(grp->pins[i]), REG_PIN_FUNC_SEL,
				FIELD_PREP(REG_PIN_FUNC_SEL, func->val));

		dev_dbg(bpctl->dev, "PIN%u set to function %u", grp->pins[i], func->val);
	}
	return 0;
}

static int bflb_pinmux_gpio_request(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range, unsigned int offset)
{
	int err;
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);

	err = regmap_update_bits(bpctl->map, REG_PIN(offset), REG_PIN_FUNC_SEL,
			FIELD_PREP(REG_PIN_FUNC_SEL, BFLB_PINMUX_FUNC_GPIO));
	dev_dbg(bpctl->dev, "PIN%u is set to function GPIO", offset);
	return err;
}

static void bflb_pinmux_gpio_free(struct pinctrl_dev *pctldev,
				   struct pinctrl_gpio_range *range,
				   unsigned int offset)
{
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(bpctl->dev, "PIN%u function GPIO is freed", offset);
}

static int bflb_pinmux_gpio_set_direction(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range, unsigned int offset, bool input)
{
	int err;
	unsigned long mask, val, reg = REG_PIN(offset);
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);

	if (input) {
		mask = REG_PIN_OE | REG_PIN_IE | REG_PIN_SMT;
		val = FIELD_PREP(REG_PIN_OE, 0) | FIELD_PREP(REG_PIN_IE, 1) |
				FIELD_PREP(REG_PIN_SMT, 1);
		err = regmap_update_bits(bpctl->map, reg, mask, val);
	} else {
		mask = REG_PIN_OE | REG_PIN_IE | REG_PIN_SMT | REG_PIN_MODE_MASK,
		val = FIELD_PREP(REG_PIN_PD, 0) | FIELD_PREP(REG_PIN_PU, 0) |
				FIELD_PREP(REG_PIN_OE, 1) | FIELD_PREP(REG_PIN_IE, 0) |
				FIELD_PREP(REG_PIN_SMT, 1) | FIELD_PREP(REG_PIN_MODE_MASK, 0);
		err = regmap_update_bits(bpctl->map, reg, mask, val);
	}
	dev_dbg(bpctl->dev, "PIN%u set to direction %s",
			offset, input ? "input" : "output");
	return err;
}

static const struct pinmux_ops bflb_pinmux_ops = {
	.get_functions_count = bflb_pinmux_get_functions_count,
	.get_function_name = bflb_pinmux_get_function_name,
	.get_function_groups = bflb_pinmux_get_function_groups,
	.set_mux = bflb_pinmux_set,
	.gpio_request_enable = bflb_pinmux_gpio_request,
	.gpio_disable_free = bflb_pinmux_gpio_free,
	.gpio_set_direction = bflb_pinmux_gpio_set_direction,
	.strict = true,
};

static int bflb_pinconf_get(struct pinctrl_dev *pctldev,
			       unsigned int pin, unsigned long *config)
{
	u32 arg;
	unsigned int val;
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);

	regmap_read(bpctl->map, REG_PIN(pin), &val);
	switch (param) {
	case PIN_CONFIG_OUTPUT_ENABLE:
		arg = FIELD_GET(REG_PIN_OE, val);
		break;

	case PIN_CONFIG_OUTPUT:
		arg = FIELD_GET(REG_PIN_OUT_HIGH, val);
		break;

	case PIN_CONFIG_INPUT_ENABLE:
		arg = FIELD_GET(REG_PIN_IE, val);
		break;

	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		arg = FIELD_GET(REG_PIN_SMT, val);
		break;

	case PIN_CONFIG_BIAS_PULL_UP:
		arg = FIELD_GET(REG_PIN_PU, val);
		break;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		arg = FIELD_GET(REG_PIN_PD, val);
		break;

	case PIN_CONFIG_BIAS_DISABLE:
		arg = !FIELD_GET(REG_PIN_PD | REG_PIN_PU, val);
		break;

	case PIN_CONFIG_DRIVE_STRENGTH:
		arg = FIELD_GET(REG_PIN_DRV, val);
		break;

	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int bflb_pinconf_set(struct pinctrl_dev *pctldev,
		unsigned int pin, unsigned long *configs, unsigned int num_configs)
{
	u32 arg;
	unsigned int i, reg;
	enum pin_config_param param;
	struct bflb_pinctrl_info *bpctl = pinctrl_dev_get_drvdata(pctldev);

	reg = REG_PIN(pin);
	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		dev_dbg(bpctl->dev, "PIN%u config set to %lu (param %u, arg %u)",
				pin, configs[i], param, arg);
		switch (param) {
		case PIN_CONFIG_OUTPUT:
			regmap_update_bits(bpctl->map, reg, REG_PIN_OE | REG_PIN_OUT_HIGH,
				FIELD_PREP(REG_PIN_OE, 1) | FIELD_PREP(REG_PIN_OUT_HIGH, !!arg));
			break;

		case PIN_CONFIG_OUTPUT_ENABLE:
			regmap_update_bits(bpctl->map, reg, REG_PIN_OE,
					FIELD_PREP(REG_PIN_OE, !!arg));
			break;

		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			regmap_update_bits(bpctl->map, reg, REG_PIN_SMT,
					FIELD_PREP(REG_PIN_SMT, !!arg));
			break;

		case PIN_CONFIG_INPUT_ENABLE:
			regmap_update_bits(bpctl->map, reg, REG_PIN_IE,
					FIELD_PREP(REG_PIN_IE, !!arg));
			break;

		case PIN_CONFIG_BIAS_PULL_UP:
			if (arg) {
				regmap_update_bits(bpctl->map, reg, REG_PIN_PU | REG_PIN_PD,
						FIELD_PREP(REG_PIN_PU, 1) | FIELD_PREP(REG_PIN_PD, 0));
			}
			break;

		case PIN_CONFIG_BIAS_PULL_DOWN:
			if (arg) {
				regmap_update_bits(bpctl->map, reg, REG_PIN_PU | REG_PIN_PD,
						FIELD_PREP(REG_PIN_PD, 1) | FIELD_PREP(REG_PIN_PU, 0));
			}
			break;

		case PIN_CONFIG_BIAS_DISABLE:
			if (arg) {
				regmap_update_bits(bpctl->map, reg, REG_PIN_PU | REG_PIN_PD,
						FIELD_PREP(REG_PIN_PD, 0) | FIELD_PREP(REG_PIN_PU, 0));
			}
			break;

		case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
			if (arg) {
				regmap_update_bits(bpctl->map, reg, REG_PIN_PU | REG_PIN_PD,
						FIELD_PREP(REG_PIN_PD, 0) | FIELD_PREP(REG_PIN_PU, 0));
			}
			break;

		case PIN_CONFIG_DRIVE_STRENGTH:
			regmap_update_bits(bpctl->map, reg, REG_PIN_DRV,
					FIELD_PREP(REG_PIN_DRV, arg));
			break;

		case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
			if (arg) {
				regmap_update_bits(bpctl->map, reg, REG_PIN_PD | REG_PIN_PU |
						REG_PIN_IE | REG_PIN_OE, FIELD_PREP(REG_PIN_PD, 0) |
						FIELD_PREP(REG_PIN_PU, 0) | FIELD_PREP(REG_PIN_IE, 0) |
						FIELD_PREP(REG_PIN_OE, 0));
			}
			break;

		default:
			return -ENOTSUPP;
		}
	}
	return 0;
}

static const struct pinconf_ops bflb_pinconf_ops = {
	.pin_config_set = bflb_pinconf_set,
	.pin_config_get = bflb_pinconf_get,
	.is_generic = true,
};

/* GPIO chip functions */
static int bflb_gpio_get_direction(struct gpio_chip *chip,
		unsigned int offset)
{
	unsigned int reg;
	struct bflb_pinctrl_info *pctl = gpiochip_get_data(chip);

	regmap_read(pctl->map, REG_PIN(offset), &reg);
	if (FIELD_GET(REG_PIN_OE, reg) == 1 &&
			FIELD_GET(REG_PIN_IE, reg) == 0) {
		return GPIO_LINE_DIRECTION_OUT;
	} else if (FIELD_GET(REG_PIN_IE, reg) == 1 &&
			FIELD_GET(REG_PIN_OE, reg) == 0) {
		return GPIO_LINE_DIRECTION_IN;
	}

	return -EIO;
}

static int bflb_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	unsigned int reg;
	struct bflb_pinctrl_info *pctl = gpiochip_get_data(chip);

	regmap_read(pctl->map, REG_PIN(offset), &reg);
	if (FIELD_GET(REG_PIN_OE, reg) == 1 &&
			FIELD_GET(REG_PIN_IE, reg) == 0) {
		reg = readl_relaxed(pctl->base + REG_PIN(offset));
		return !!(reg & REG_PIN_OUT_HIGH);
	} else if (FIELD_GET(REG_PIN_IE, reg) == 1 &&
			FIELD_GET(REG_PIN_OE, reg) == 0) {
		reg = readl_relaxed(pctl->base + REG_PIN(offset));
		return !!(reg & REG_PIN_IN_HIGH);
	}

	return -EIO;
}

/*
 * Note: offset is the local GPIO number of the current chip, not the
 * global GPIO number (which is chip->base + offset), nor pin number.
 * Thus GPIO set function does not work if the pin-base in gpio-ranges
 * (gpio-ranges = <&pinctrl gpio-base pin-base num>)is not zero, because
 * the wrong register would be programmed.
 *
 * The rigorous method:
 *		gpio = chip->base + offset;
 *		pin = gpio_to_pin(gpio);
 *		reg_addr = pin_to_reg(pin);
 *		write register(reg_addr, val);
 */
static void bflb_gpio_set(struct gpio_chip *chip, unsigned int offset,
		int value)
{
	struct bflb_pinctrl_info *bpctl = gpiochip_get_data(chip);

	regmap_update_bits(bpctl->map, REG_PIN(offset), REG_PIN_OUT_HIGH,
						FIELD_PREP(REG_PIN_OUT_HIGH, !!value));
	dev_dbg(bpctl->dev, "PIN%u set to value %u", offset, value);
}

static int bflb_gpio_direction_input(struct gpio_chip *chip,
		unsigned int offset)
{
	int err;
	struct bflb_pinctrl_info *bpctl = gpiochip_get_data(chip);

	err = pinctrl_gpio_direction_input(chip->base + offset);
	dev_dbg(bpctl->dev, "GPIO%u set to direction input", chip->base + offset);
	return err;
}

static int bflb_gpio_direction_output(struct gpio_chip *chip,
		unsigned int offset, int value)
{
	int err;
	struct bflb_pinctrl_info *pctl = gpiochip_get_data(chip);

	err = pinctrl_gpio_direction_output(chip->base + offset);
	if (err)
		return err;
	bflb_gpio_set(chip, offset, value);
	dev_dbg(pctl->dev, "GPIO%u set to direction output", chip->base + offset);
	return 0;
}

static void bflb_gpio_irq_ack(struct irq_data *data)
{
	irq_hw_number_t irq = irqd_to_hwirq(data);
	struct bflb_pinctrl_info *bpctl =
			gpiochip_get_data(irq_data_get_irq_chip_data(data));

	regmap_update_bits(bpctl->map, REG_PIN(irq), REG_PIN_INT_CLR,
			FIELD_PREP(REG_PIN_INT_CLR, 1));
	regmap_update_bits(bpctl->map, REG_PIN(irq), REG_PIN_INT_CLR,
			FIELD_PREP(REG_PIN_INT_CLR, 0));

	dev_dbg(bpctl->dev, "GPIO %lu IRQ ACK", irq);
}

static unsigned int bflb_gpio_irq_type(unsigned int type)
{
	unsigned int mode;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		mode = BFLB_IRQ_MODE_SYNC_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		mode = BFLB_IRQ_MODE_SYNC_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		mode = BFLB_IRQ_MODE_SYNC_EDGE_BOTH;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		mode = BFLB_IRQ_MODE_SYNC_LEVEL_HIGH;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		mode = BFLB_IRQ_MODE_SYNC_LEVEL_LOW;
		break;

	default:
		mode = BFLB_IRQ_MODE_SYNC_EDGE_FALLING;
		break;
	}

	return mode;
}

static void bflb_gpio_irq_mask(struct irq_data *data)
{
	irq_hw_number_t irq = irqd_to_hwirq(data);
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct bflb_pinctrl_info *bpctl = gpiochip_get_data(gc);

	regmap_update_bits(bpctl->map, REG_PIN(irq), REG_PIN_INT_MASK,
		FIELD_PREP(REG_PIN_INT_MASK, 1));
	clear_bit(irq, bpctl->unmasked_irqs);
	gpiochip_disable_irq(gc, irq);

	dev_dbg(bpctl->dev, "GPIO%lu IRQ masked", irq);
}

static void bflb_gpio_irq_unmask(struct irq_data *data)
{
	irq_hw_number_t irq = irqd_to_hwirq(data);
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct bflb_pinctrl_info *bpctl = gpiochip_get_data(gc);
	unsigned int irqtype = bflb_gpio_irq_type(irqd_get_trigger_type(data));

	gpiochip_enable_irq(gc, irq);
	set_bit(irq, bpctl->unmasked_irqs);
	regmap_update_bits(bpctl->map, REG_PIN(irq), REG_PIN_INT_MASK |
		REG_PIN_INT_MODE_SET, FIELD_PREP(REG_PIN_INT_MASK, 0) |
		FIELD_PREP(REG_PIN_INT_MODE_SET, irqtype));

	dev_dbg(bpctl->dev, "GPIO%lu IRQ unmasked", irq);
}

static unsigned int bflb_gpio_irq_startup(struct irq_data *data)
{
	irq_hw_number_t irq = irqd_to_hwirq(data);
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct bflb_pinctrl_info *bpctl = gpiochip_get_data(chip);

	regmap_update_bits(bpctl->map, REG_PIN(irq), REG_PIN_INT_CLR,
			FIELD_PREP(REG_PIN_INT_CLR, 1));
	regmap_update_bits(bpctl->map, REG_PIN(irq), REG_PIN_INT_CLR,
			FIELD_PREP(REG_PIN_INT_CLR, 0));
	bflb_gpio_irq_unmask(data);

	dev_dbg(bpctl->dev, "PIN%lu IRQ started", irq);
	return 0;
}

static int bflb_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct bflb_pinctrl_info *bpctl;
	irq_hw_number_t irq = irqd_to_hwirq(data);
	unsigned int irqtype = bflb_gpio_irq_type(type);

	if (irqtype == 0)
		return -EINVAL;

	bpctl = gpiochip_get_data(irq_data_get_irq_chip_data(data));
	regmap_update_bits(bpctl->map, REG_PIN(irq), REG_PIN_INT_MODE_SET,
			FIELD_PREP(REG_PIN_INT_MODE_SET, irqtype));
	if (type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(data, handle_level_irq);
	else
		irq_set_handler_locked(data, handle_edge_irq);

	dev_dbg(bpctl->dev, "GPIO%lu IRQ type set to %u", irq, irqtype);
	return 0;
}

/* handle GPIO interrupts on this controller */
static void bflb_gpio_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct bflb_pinctrl_info *pctl = irq_desc_get_handler_data(desc);
	unsigned int gpio;
	unsigned long reg;
	struct gpio_chip *gc;

	gc = &pctl->gpio_chip;
	chained_irq_enter(chip, desc);
	/*
	 * we must go through each individual GPIO register to read its interrupt
	 * status. There is no helper register for all pending interrupts.
	 */
	for (gpio = 0; gpio < gc->ngpio; gpio++) {
		if (test_bit(gpio, pctl->unmasked_irqs)) {
			dev_dbg(pctl->dev, "reading IRQ status of GPIO%u", gpio);

			reg = readl_relaxed(pctl->base + REG_PIN(gpio));
			if (reg & REG_PIN_INT_STAT) {
				generic_handle_domain_irq(gc->irq.domain, gpio);
				dev_dbg(pctl->dev, "GPIO%u IRQ is set", gpio);
			}
		} else {
			dev_dbg(pctl->dev, "ignoring IRQ status of masked GPIO%u", gpio);
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

static int bflb_gpio_register(struct bflb_pinctrl_info *pctl)
{
	int ret;
	struct device *dev = pctl->dev;
	struct gpio_irq_chip *girq = &pctl->gpio_chip.irq;
	struct platform_device *pdev = to_platform_device(dev);
	unsigned int npins = pctl->pinctrl_desc.npins;

	pctl->unmasked_irqs = devm_bitmap_zalloc(&pdev->dev, npins, GFP_KERNEL);
	if (!pctl->unmasked_irqs)
		return -ENOMEM;

	pctl->gpio_chip.label = dev_name(dev);
	pctl->gpio_chip.request = gpiochip_generic_request;
	pctl->gpio_chip.free = gpiochip_generic_free;
	pctl->gpio_chip.get_direction = bflb_gpio_get_direction;
	pctl->gpio_chip.direction_input = bflb_gpio_direction_input;
	pctl->gpio_chip.direction_output = bflb_gpio_direction_output;
	pctl->gpio_chip.get = bflb_gpio_get;
	pctl->gpio_chip.set = bflb_gpio_set;
	pctl->gpio_chip.set_config = gpiochip_generic_config;
	pctl->gpio_chip.base = -1;
	pctl->gpio_chip.ngpio = npins;
	pctl->gpio_chip.parent = dev;

	if (of_property_present(dev_of_node(dev), "interrupt-controller")) {
		ret = platform_irq_count(pdev);
		if (ret > 0)
			girq->num_parents = ret;
		else
			girq->num_parents = 0;
	}

	if (girq->num_parents) {
		int i;

		gpio_irq_chip_set_chip(girq, &bflb_gpio_irqchip);
		girq->parent_handler = bflb_gpio_irq_handler;

		girq->parents = devm_kmalloc_array(dev, girq->num_parents,
				sizeof(*girq->parents), GFP_KERNEL);
		if (!girq->parents)
			return -ENOMEM;

		for (i = 0; i < girq->num_parents; i++) {
			ret = platform_get_irq(pdev, i);
			if (ret < 0)
				return ret;

			girq->parents[i] = ret;
		}

		girq->parent_handler_data = pctl;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_level_irq;
	}

	return devm_gpiochip_add_data(dev, &pctl->gpio_chip, pctl);
}

static int bflb_pinctrl_parse_group(struct bflb_pinctrl_info *bpctl,
		struct bflb_pingroup *grp, struct device_node *np, int *has_func)
{
	int err, size;
	unsigned int i;
	const __be32 *list;

	*has_func = 1;
	grp->fn_val = BFLB_PINMUX_FUNC_NONE;
	if (of_property_read_u32(np, "bflb,pin-function", &grp->fn_val))
		*has_func = 0;

	grp->name = np->name;
	/* the binding format is bflb,pins = <bank pin config> */
	list = of_get_property(np, "bflb,pins", &size);
	if (!list)
		return dev_err_probe(bpctl->dev, -EINVAL, "bflb,pins is not found\n");

	size /= sizeof(*list);
	if (!size || size % 3)
		return dev_err_probe(bpctl->dev, -EINVAL, "invalid binding format\n");

	grp->npins = size / 3;
	grp->pins = devm_kcalloc(bpctl->dev, grp->npins, sizeof(*grp->pins),
							 GFP_KERNEL);
	if (!grp->pins)
		return -ENOMEM;
	grp->pinconf = devm_kcalloc(bpctl->dev, grp->npins, sizeof(*grp->pinconf),
								GFP_KERNEL);
	if (!grp->pinconf)
		return -ENOMEM;

	dev_dbg(bpctl->dev, "parsing group %s, npins %lu\n", grp->name, grp->npins);
	/* parse each pin of this group */
	for (i = 0; i < grp->npins; i++) {
		phandle cfg_phandle;
		struct device_node *np_config;

		/* now the bank number is ignored */
		list++;
		/* here is the pin number */
		grp->pins[i] = be32_to_cpu(*list++);
		/* and phandle of the pin configurations */
		cfg_phandle = be32_to_cpup(list++);
		np_config = of_find_node_by_phandle(cfg_phandle);
		if (!np_config) {
			return dev_err_probe(bpctl->dev, -EINVAL,
					"invalid pin cfg phandle %u\n", cfg_phandle);
		}

		err = pinconf_generic_parse_dt_config(np_config, NULL,
				&grp->pinconf[i].configs, &grp->pinconf[i].nconfigs);
		of_node_put(np_config);
		if (err)
			return err;
	}
	return 0;
}

static int bflb_pinctrl_function_add_group(struct bflb_pinctrl_info *bpctl,
		struct bflb_pinfunction *func, struct bflb_pingroup *grp)
{
	if (func->ngroups >= func->ncap) {
		func->ncap++;
		func->groups = devm_krealloc_array(bpctl->dev, func->groups,
						func->ncap, sizeof(*func->groups), GFP_KERNEL);
		if (!func->groups)
			return -ENOMEM;
	}

	func->groups[func->ngroups++] = grp->name;
	return 0;
}

static int bflb_pinctrl_create_functions(struct bflb_pinctrl_info *bpctl)
{
	int i;

	bpctl->nfunctions = ARRAY_SIZE(bl808_functions);
	bpctl->functions = devm_kcalloc(bpctl->dev, bpctl->nfunctions,
									sizeof(*bpctl->functions), GFP_KERNEL);
	if (!bpctl->functions)
		return -ENOMEM;

	for (i = 0; i < bpctl->nfunctions; i++) {
		bpctl->functions[i].name = bl808_functions[i].name;
		bpctl->functions[i].val = bl808_functions[i].val;
		bpctl->functions[i].ngroups = 0;
		bpctl->functions[i].ncap = 0;
	}
	return 0;
}

static int bflb_pinctrl_parse_dt(struct bflb_pinctrl_info *bpctl)
{
	int err, ngroups = 0;
	struct device_node *child;
	struct device *dev = bpctl->dev;
	struct bflb_pingroup *grp;
	struct bflb_pinfunction *func;
	struct device_node *np = dev_of_node(dev);

	err = bflb_pinctrl_create_functions(bpctl);
	if (err)
		return err;
	/*
	 * allocate memory with possible max size, there might be some bad configs
	 * in device tree though.
	 */
	for_each_child_of_node(np, child) {
		if (of_property_present(child, "bflb,pins"))
			ngroups++;
	}
	bpctl->groups = devm_kcalloc(dev, ngroups, sizeof(*bpctl->groups),
									GFP_KERNEL);
	if (!bpctl->groups)
		return -ENOMEM;

	bpctl->ngroups = 0;
	for_each_child_of_node(np, child) {
		/* try to parse the pin group */
		int has_func = 0;

		if (!of_property_present(child, "bflb,pins"))
			continue;

		grp = &bpctl->groups[bpctl->ngroups];
		err = bflb_pinctrl_parse_group(bpctl, grp, child, &has_func);
		if (err) {
			dev_err(bpctl->dev, "failed to parse group %s\n", child->name);
			return err;
		}

		bpctl->ngroups++;
		/* adding a group without function is fine */
		if (!has_func)
			continue;

		func = bflb_pinctrl_parse_function(bpctl, grp->fn_val);
		if (IS_ERR_OR_NULL(func)) {
			dev_err(bpctl->dev, "wrong pinmux value 0x%x for group %s\n",
					grp->fn_val, grp->name);
			return -EINVAL;
		}

		err = bflb_pinctrl_function_add_group(bpctl, func, grp);
		if (err) {
			return dev_err_probe(bpctl->dev, err,
					"function %s failed to add group %s\n", func->name, grp->name);
		}
	}

	return 0;
}

static int bflb_pinctrl_gpio_probe(struct platform_device *pdev)
{
	int err;
	struct bflb_pinctrl_info *pctl;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	pctl->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, pctl);
	err = bflb_pinctrl_parse_dt(pctl);
	if (err)
		return err;

	pctl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctl->base))
		return PTR_ERR(pctl->base);

	pctl->map = devm_regmap_init_mmio(&pdev->dev, pctl->base, &regmap_config);
	if (IS_ERR(pctl->map)) {
		return dev_err_probe(&pdev->dev, PTR_ERR(pctl->map),
				"Failed to create regmap\n");
	}

	pctl->pinctrl_desc.name = dev_name(pctl->dev);
	pctl->pinctrl_desc.pins = bl808_pins;
	pctl->pinctrl_desc.npins = ARRAY_SIZE(bl808_pins);
	pctl->pinctrl_desc.pctlops = &bflb_pinctrl_ops;
	pctl->pinctrl_desc.pmxops = &bflb_pinmux_ops;
	pctl->pinctrl_desc.confops = &bflb_pinconf_ops;
	pctl->pctldev =	devm_pinctrl_register(&pdev->dev, &pctl->pinctrl_desc,
			pctl);
	if (IS_ERR(pctl->pctldev)) {
		return dev_err_probe(&pdev->dev, PTR_ERR(pctl->pctldev),
				"Failed to register pinctrl device.\n");
	}

	err = bflb_gpio_register(pctl);
	if (err)
		return err;

	dev_info(&pdev->dev,
		"Bouffalo Lab pinctrl+GPIO+interrupt controller registered\n");
	return 0;
}

static const struct of_device_id bflb_pinctrl_gpio_of_ids[] = {
	{ .compatible = "bflb,bl808-pinctrl", },
	{ }
};

MODULE_DEVICE_TABLE(of, bflb_pinctrl_gpio_of_ids);

static struct platform_driver bflb_pinctrl_gpio_driver = {
	.driver = {
		.name = "bflb-pinctrl-gpio",
		.of_match_table = bflb_pinctrl_gpio_of_ids,
		.suppress_bind_attrs = true,
	},
	.probe = bflb_pinctrl_gpio_probe,
};

module_platform_driver(bflb_pinctrl_gpio_driver);

MODULE_DESCRIPTION("Bouffalo pinctrl-gpio driver");
MODULE_AUTHOR("Bouffalo Lab");
MODULE_LICENSE("GPL v2");
