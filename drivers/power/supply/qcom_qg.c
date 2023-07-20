// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Bert Karwatzki <spasswolf (at) web.de >. */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <linux/iio/consumer.h>
#include <linux/nvmem-consumer.h>

/*
 * register definitions from downstream kernel
 * drivers/power/supply/qcom/qg-regs.h
 * */
#define PERPH_TYPE_REG				0x04
#define QG_TYPE					0x0D

#define QG_STATUS1_REG				0x08
#define QG_OK_BIT				BIT(7)
#define BATTERY_PRESENT_BIT			BIT(0)
#define ESR_MEAS_DONE_BIT			BIT(4)

#define QG_STATUS2_REG				0x09
#define GOOD_OCV_BIT				BIT(1)

#define QG_STATUS3_REG				0x0A
#define COUNT_FIFO_RT_MASK			GENMASK(3, 0)

#define QG_STATUS4_REG				0x0B
#define ESR_MEAS_IN_PROGRESS_BIT		BIT(4)

#define QG_INT_RT_STS_REG			0x10
#define FIFO_UPDATE_DONE_RT_STS_BIT		BIT(3)
#define VBAT_LOW_INT_RT_STS_BIT			BIT(1)
#define BATTERY_MISSING_INT_RT_STS_BIT		BIT(0)

#define QG_INT_LATCHED_STS_REG			0x18
#define FIFO_UPDATE_DONE_INT_LAT_STS_BIT	BIT(3)

#define QG_DATA_CTL1_REG			0x41
#define MASTER_HOLD_OR_CLR_BIT			BIT(0)

#define QG_DATA_CTL2_REG			0x42
#define BURST_AVG_HOLD_FOR_READ_BIT		BIT(0)

#define QG_MODE_CTL1_REG			0x43
#define PARALLEL_IBAT_SENSE_EN_BIT		BIT(7)

#define QG_VBAT_EMPTY_THRESHOLD_REG		0x4B
#define QG_VBAT_LOW_THRESHOLD_REG		0x4C

#define QG_S2_NORMAL_MEAS_CTL2_REG		0x51
#define FIFO_LENGTH_MASK			GENMASK(5, 3)
#define FIFO_LENGTH_SHIFT			3
#define NUM_OF_ACCUM_MASK			GENMASK(2, 0)

#define QG_S2_NORMAL_MEAS_CTL3_REG		0x52

#define QG_S3_SLEEP_OCV_MEAS_CTL4_REG		0x59
#define S3_SLEEP_OCV_TIMER_MASK			GENMASK(2, 0)

#define QG_S3_SLEEP_OCV_TREND_CTL2_REG		0x5C
#define TREND_TOL_MASK				GENMASK(5, 0)

#define QG_S3_SLEEP_OCV_IBAT_CTL1_REG		0x5D
#define SLEEP_IBAT_QUALIFIED_LENGTH_MASK	GENMASK(2, 0)

#define QG_S3_ENTRY_IBAT_THRESHOLD_REG		0x5E
#define QG_S3_EXIT_IBAT_THRESHOLD_REG		0x5F

#define QG_S5_OCV_VALIDATE_MEAS_CTL1_REG	0x60
#define ALLOW_S5_BIT				BIT(7)

#define QG_S7_PON_OCV_MEAS_CTL1_REG		0x64
#define ADC_CONV_DLY_MASK			GENMASK(3, 0)

#define QG_ESR_MEAS_TRIG_REG			0x68
#define HW_ESR_MEAS_START_BIT			BIT(0)

#define QG_S7_PON_OCV_V_DATA0_REG		0x70
#define QG_S7_PON_OCV_I_DATA0_REG		0x72
#define QG_S3_GOOD_OCV_V_DATA0_REG		0x74
#define QG_S3_GOOD_OCV_I_DATA0_REG		0x76

#define QG_PRE_ESR_V_DATA0_REG			0x78
#define QG_PRE_ESR_I_DATA0_REG			0x7A
#define QG_POST_ESR_V_DATA0_REG			0x7C
#define QG_POST_ESR_I_DATA0_REG			0x7E

#define QG_V_ACCUM_DATA0_RT_REG			0x88
#define QG_I_ACCUM_DATA0_RT_REG			0x8B
#define QG_ACCUM_CNT_RT_REG			0x8E

#define QG_V_FIFO0_DATA0_REG			0x90
#define QG_I_FIFO0_DATA0_REG			0xA0

#define QG_SOC_MONOTONIC_REG			0xBF

#define QG_LAST_ADC_V_DATA0_REG			0xC0
#define QG_LAST_ADC_I_DATA0_REG			0xC2

#define QG_LAST_BURST_AVG_I_DATA0_REG		0xC6

#define QG_LAST_S3_SLEEP_V_DATA0_REG		0xCC

/* SDAM offsets (should these go to the devicetree (as nvmem cells)?) */
#define QG_SDAM_VALID_OFFSET			0x46 /* 1-byte 0x46 */
#define QG_SDAM_SOC_OFFSET			0x47 /* 1-byte 0x47 */
#define QG_SDAM_TEMP_OFFSET			0x48 /* 2-byte 0x48-0x49 */
#define QG_SDAM_RBAT_OFFSET			0x4A /* 2-byte 0x4A-0x4B */
#define QG_SDAM_OCV_OFFSET			0x4C /* 4-byte 0x4C-0x4F */
#define QG_SDAM_IBAT_OFFSET			0x50 /* 4-byte 0x50-0x53 */
#define QG_SDAM_TIME_OFFSET			0x54 /* 4-byte 0x54-0x57 */
#define QG_SDAM_CYCLE_COUNT_OFFSET		0x58 /* 16-byte 0x58-0x67 */
#define QG_SDAM_LEARNED_CAPACITY_OFFSET		0x68 /* 2-byte 0x68-0x69 */
#define QG_SDAM_ESR_CHARGE_DELTA_OFFSET		0x6A /* 4-byte 0x6A-0x6D */
#define QG_SDAM_ESR_DISCHARGE_DELTA_OFFSET	0x6E /* 4-byte 0x6E-0x71 */
#define QG_SDAM_ESR_CHARGE_SF_OFFSET		0x72 /* 2-byte 0x72-0x73 */
#define QG_SDAM_ESR_DISCHARGE_SF_OFFSET		0x74 /* 2-byte 0x74-0x75 */
#define QG_SDAM_MAGIC_OFFSET			0x80 /* 4-byte 0x80-0x83 */
#define QG_SDAM_MAX_OFFSET			0xA4

/* Below offset is used by PBS */
#define QG_SDAM_SEQ_OFFSET			0xBB /* 1-byte 0xBB */
#define QG_SDAM_PON_OCV_OFFSET			0xBC /* 2-byte 0xBC-0xBD */
/* end of regs from qg-reg.h */

/* defs from qg-defs.h*/
#define is_between(left, right, value) \
		(((left) >= (right) && (left) >= (value) \
			&& (value) >= (right)) \
		|| ((left) <= (right) && (left) <= (value) \
			&& (value) <= (right)))

#define UDATA_READY_VOTER		"UDATA_READY_VOTER"
#define FIFO_DONE_VOTER			"FIFO_DONE_VOTER"
#define FIFO_RT_DONE_VOTER		"FIFO_RT_DONE_VOTER"
#define SUSPEND_DATA_VOTER		"SUSPEND_DATA_VOTER"
#define GOOD_OCV_VOTER			"GOOD_OCV_VOTER"
#define PROFILE_IRQ_DISABLE		"NO_PROFILE_IRQ_DISABLE"
#define QG_INIT_STATE_IRQ_DISABLE	"QG_INIT_STATE_IRQ_DISABLE"
#define TTF_AWAKE_VOTER			"TTF_AWAKE_VOTER"

#define V_RAW_TO_UV(V_RAW)		div_u64(194637ULL * (u64)V_RAW, 1000)
#define I_RAW_TO_UA(I_RAW)		div_s64(152588LL * (s64)I_RAW, 1000)
#define FIFO_V_RESET_VAL		0x8000
#define FIFO_I_RESET_VAL		0x8000

#define DEGC_SCALE			10
#define UV_TO_DECIUV(a)			(a / 100)
#define DECIUV_TO_UV(a)			(a * 100)

#define QG_MAX_ESR_COUNT		10
#define QG_MIN_ESR_COUNT		2

#define CAP(min, max, value)			\
		((min > value) ? min : ((value > max) ? max : value))

#define QG_SOC_FULL	10000
#define BATT_SOC_32BIT	GENMASK(31, 0)
/* end of defs from qg-defs.h*/

struct qcom_qg_chip;

struct qcom_qg_ops {
	int (*get_capacity)(struct qcom_qg_chip *chip, int *);
	int (*get_temperature)(struct qcom_qg_chip *chip, int *);
	int (*get_current)(struct qcom_qg_chip *chip, int *);
	int (*get_voltage)(struct qcom_qg_chip *chip, int *);
};

struct qcom_qg_data {
	const struct qcom_qg_ops *ops;
};

struct qcom_qg_chip {
	struct device *dev;
	unsigned int qg_base;
	struct regmap *regmap;
	struct iio_channel *bat_therm_chan;
	// We do not use bat_id (yet ?).
	struct iio_channel *bat_id_chan;
	/* The downstream driver uses nvmem to store SOC on shutdown
	 * to use as SOC on next start if shotdwon time was not to
	 * long. We do not (yet?) use it.*/
	struct nvmem_device *nvmem;
	const struct qcom_qg_ops *ops;

	struct power_supply *batt_psy;
	struct power_supply_battery_info *batt_info;
	/* No charger support, yet. */
	struct power_supply *chg_psy;
	struct mutex bus_lock;
	// esr: internal resistance in mOhm
	int esr;
	bool battery_missing;
	int status;
};

/* from qg-util.c */
static inline bool is_sticky_register(u32 addr)
{
	if ((addr & 0xFF) == QG_STATUS2_REG)
		return true;

	return false;
}

/* from qg-util.c */
static int qcom_qg_read(struct qcom_qg_chip *chip, u32 addr, u8 *val, int len)
{
	int rc, i;
	u32 dummy = 0;

	rc = regmap_bulk_read(chip->regmap, addr, val, len);
	if (rc < 0) {
		dev_err(chip->dev, "Failed regmap_read for address %04x rc=%d\n", addr, rc);
		return rc;
	}

	if (is_sticky_register(addr)) {
		/* write to the sticky register to clear it */
		rc = regmap_write(chip->regmap, addr, dummy);
		if (rc < 0) {
			dev_err(chip->dev, "Failed regmap_write for %04x rc=%d\n",
						addr, rc);
			return rc;
		}
	}

	return 0;
}

/* from qg-util.c */
static int qcom_qg_write(struct qcom_qg_chip *chip, u32 addr, u8 *val, int len)
{
	int rc, i;

	mutex_lock(&chip->bus_lock);

	if (len > 1)
		rc = regmap_bulk_write(chip->regmap, addr, val, len);
	else
		rc = regmap_write(chip->regmap, addr, *val);

	if (rc < 0)
		dev_err(chip->dev, "Failed regmap_write for address %04x rc=%d\n",
				addr, rc);

	mutex_unlock(&chip->bus_lock);
	return rc;
}

/* from qg-util.c */
static int qcom_qg_masked_write(struct qcom_qg_chip *chip, int addr, u32 mask, u32 val)
{
	int rc;

	mutex_lock(&chip->bus_lock);

	rc = regmap_update_bits(chip->regmap, addr, mask, val);
	if (rc < 0)
		dev_err(chip->dev , "Failed regmap_update_bits for address %04x rc=%d\n",
				addr, rc);

	mutex_unlock(&chip->bus_lock);
	return rc;
}

static int qcom_qg_get_current(struct qcom_qg_chip *chip, int *ibat_ua)
{
	int rc = 0, last_ibat = 0;

	if (chip->battery_missing) {
		*ibat_ua = 0;
		return 0;
	}

	/* hold data */
	rc = qcom_qg_masked_write(chip, chip->qg_base + QG_DATA_CTL2_REG,
				BURST_AVG_HOLD_FOR_READ_BIT,
				BURST_AVG_HOLD_FOR_READ_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to hold burst-avg data rc=%d\n", rc);
		goto release;
	}

	rc = qcom_qg_read(chip, chip->qg_base + QG_LAST_BURST_AVG_I_DATA0_REG,
				(u8 *)&last_ibat, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read LAST_BURST_AVG_I reg, rc=%d\n", rc);
		goto release;
	}

	last_ibat = sign_extend32(last_ibat, 15);
	/* If current is negative we're charging, I presume.*/
	*ibat_ua = I_RAW_TO_UA(last_ibat);

release:
	/* release */
	qcom_qg_masked_write(chip, chip->qg_base + QG_DATA_CTL2_REG,
				BURST_AVG_HOLD_FOR_READ_BIT, 0);
	return rc;
}

/* from qg-util.c */
static int qcom_qg_get_voltage(struct qcom_qg_chip *chip, int *vbat_uv)
{
	int rc = 0;
	u64 last_vbat = 0;

	if (chip->battery_missing) {
		*vbat_uv = 3700000;
		return 0;
	}

	rc = qcom_qg_read(chip, chip->qg_base + QG_LAST_ADC_V_DATA0_REG,
				(u8 *)&last_vbat, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read LAST_ADV_V reg, rc=%d\n", rc);
		return rc;
	}

	*vbat_uv = V_RAW_TO_UV(last_vbat);

	return rc;
}

static int qcom_qg_get_temperature(struct qcom_qg_chip *chip, int *temp)
{
	int ret;

	/* Is *temp given in millicelsius? */
	ret = iio_read_channel_processed(chip->bat_therm_chan, temp);
	if (ret < 0)
		dev_err(chip->dev, "error reading battery temp, ret = %d\n", ret);

	return ret;
}

static int qcom_qg_get_capacity(struct qcom_qg_chip *chip, int *soc)
{
	int temp, vbat, ocv, ibat, ret;
	bool charging;
	ret = qcom_qg_get_current(chip, &ibat);
	if (ret < 0) {
		dev_err(chip->dev, "failed to read ibat, ret = %d\n", ret);
		return ret;
	}
	charging = (ibat < 0) ? true : false;
	ret = qcom_qg_get_temperature(chip, &temp);
	if (ret < 0) {
		dev_err(chip->dev, "failed to read temperature, ret = %d\n", ret);
		return ret;
	}
	// convert from millicelsius to celsius
	temp /= 1000;
	ret = qcom_qg_get_voltage(chip, &vbat);
	if (ret < 0) {
		dev_err(chip->dev, "failed to read vbat, ret = %d\n", ret);
		return ret;
	}

	ocv = vbat + (ibat * chip->esr) / 1000;

	*soc = power_supply_batinfo_ocv2cap(chip->batt_info, ocv, temp);
	return 0;
}

static const struct qcom_qg_ops ops_qg = {
	.get_capacity = qcom_qg_get_capacity,
	.get_temperature = qcom_qg_get_temperature,
	.get_current = qcom_qg_get_current,
	.get_voltage = qcom_qg_get_voltage,
};

static const struct qcom_qg_data pmi632_data = {
	.ops = &ops_qg,
};

static const struct dev_pm_ops qcom_qg_pm_ops = {
	//.suspend_noirq	= qcom_qg_suspend_noirq,
	.suspend_noirq	= NULL,
	//.resume_noirq	= qcom_qg_resume_noirq,
	.resume_noirq	= NULL,
	//.suspend	= qcom_qg_suspend,
	.suspend	= NULL,
	//.resume		= qcom_qg_resume,
	.resume		= NULL,
};


static enum power_supply_property qcom_qg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
};

static int qcom_qg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct qcom_qg_chip *chip = power_supply_get_drvdata(psy);
	int ibat, ret = 0;

	dev_dbg(chip->dev, "Getting property: %d", psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = chip->ops->get_current(chip, &ibat);
		if (ibat > 0)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (ibat < 0)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = chip->ops->get_capacity(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = chip->ops->get_current(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = chip->ops->get_voltage(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->batt_info->voltage_min_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->batt_info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->batt_info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = chip->ops->get_temperature(chip, &val->intval);
		break;
	default:
		dev_err(chip->dev, "invalid property: %d\n", psp);
		return -EINVAL;
	}

	return ret;
}

static const struct power_supply_desc batt_psy_desc = {
	.name = "qcom-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = qcom_qg_props,
	.num_properties = ARRAY_SIZE(qcom_qg_props),
	.get_property = qcom_qg_get_property,
	//.external_power_changed	= qcom_fg_external_power_changed,
	.external_power_changed	= NULL,
};

static int qcom_qg_probe(struct platform_device *pdev)
{
	struct power_supply_config supply_config = {};
	struct qcom_qg_chip *chip;
	struct iio_channel *chan0, *chan1;
	const struct qcom_qg_data *data;
	const __be32 *prop_addr;
	int irq;
	u8 dma_status;
	bool error_present;
	int ret, val;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	data = of_device_get_match_data(&pdev->dev);
	chip->ops = data->ops;

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		dev_err(chip->dev, "Failed to locate the regmap\n");
		return -ENODEV;
	}

	/* Get base address */
	prop_addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
	if (!prop_addr) {
		dev_err(chip->dev, "Failed to read SOC base address from dt\n");
		return -EINVAL;
	}
	chip->qg_base = be32_to_cpu(*prop_addr);

	/* With this we can try to read the temperature so we can at least
	 * do something, how can we calcluate the actual temperature from
	 * this? */
	chan0 = devm_iio_channel_get(chip->dev, "bat_therm");
	if (IS_ERR(chan0)) {
		dev_err(chip->dev, "Failed to get bat_therm iio_channel\n");
	}
	chip->bat_therm_chan = chan0;

	/* We only need to read bat_id if we're going to use fancy profiles */
	chan1 = devm_iio_channel_get(chip->dev, "bat_id");
	if (IS_ERR(chan1)) {
		dev_err(chip->dev, "Failed to get bat_id iio_channel\n");
	}
	chip->bat_id_chan = chan1;

	mutex_init(&chip->bus_lock);

	supply_config.drv_data = chip;
	supply_config.of_node = pdev->dev.of_node;

	chip->batt_psy = devm_power_supply_register(chip->dev,
			&batt_psy_desc, &supply_config);
	if (IS_ERR(chip->batt_psy)) {
		dev_err(chip->dev, "Failed to register battery\n");
		return PTR_ERR(chip->batt_psy);
	}

	ret = power_supply_get_battery_info(chip->batt_psy, &chip->batt_info);
	if (ret) {
		dev_err(chip->dev, "Failed to get battery info: %d\n", ret);
		return ret;
	}

	// set esr from devicetree, esr in mOhm
	chip->esr = chip->batt_info->factory_internal_resistance_uohm / 1000;

	chip->nvmem = devm_nvmem_device_get(chip->dev, "pmi632_sdam_2");
	if (IS_ERR(chip->nvmem)) {
		dev_err(chip->dev, "Failed to get nvmem\n");
		return PTR_ERR(chip->nvmem);
	}

	return 0;
}

static int qcom_qg_remove(struct platform_device *pdev)
{
	/* coming soon ... */
	return 0;
}

static const struct of_device_id qg_match_id_table[] = {
	{ .compatible = "qcom,pmi632-qg", .data = &pmi632_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, qg_match_id_table);

static struct platform_driver qcom_qg_driver = {
	.probe = qcom_qg_probe,
	.remove = qcom_qg_remove,
	.driver = {
		.name = "qcom-qg",
		.of_match_table = qg_match_id_table,
		//.pm = &qcom_qg_pm_ops,
	},
};

module_platform_driver(qcom_qg_driver);

MODULE_AUTHOR("Caleb Connolly <caleb@connolly.tech>");
MODULE_AUTHOR("Joel Selvaraj <jo@jsfamily.in>");
MODULE_AUTHOR("Yassine Oudjana <y.oudjana@protonmail.com>");
MODULE_AUTHOR("Bert Karwatzki <spasswolf@web.de>");
MODULE_DESCRIPTION("Qualcomm PMIC Fuel Gauge Driver");
MODULE_LICENSE("GPL v2");
