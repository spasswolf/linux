// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Himax hx83112b touchscreens
 *
 * Copyright (C) 2022 Job Noorman <job@noorman.info>
 *
 * HX83100A support
 * Copyright (C) 2024 Felix Kaechele <felix@kaechele.ca>
 *
 * This code is based on "Himax Android Driver Sample Code for QCT platform":
 *
 * Copyright (C) 2017 Himax Corporation.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/regmap.h>

#include <drm/drm_panel.h>

#define HIMAX_MAX_POINTS		10

#define HIMAX_AHB_ADDR_BYTE_0			0x00
#define HIMAX_AHB_ADDR_RDATA_BYTE_0		0x08
#define HIMAX_AHB_ADDR_ACCESS_DIRECTION		0x0c
#define HIMAX_AHB_ADDR_INCR4			0x0d
#define HIMAX_AHB_ADDR_CONTI			0x13
#define HIMAX_AHB_ADDR_EVENT_STACK		0x30

#define HIMAX_AHB_CMD_ACCESS_DIRECTION_READ	0x00
#define HIMAX_AHB_CMD_INCR4			0x10
#define HIMAX_AHB_CMD_CONTI			0x31

#define HIMAX_REG_ADDR_ICID			0x900000d0

#define HX83100A_REG_FW_EVENT_STACK		0x90060000

#define HIMAX_INVALID_COORD		0xffff

struct himax_event_point {
	__be16 x;
	__be16 y;
} __packed;

struct himax_event {
	struct himax_event_point points[HIMAX_MAX_POINTS];
	u8 majors[HIMAX_MAX_POINTS];
	u8 pad0[2];
	u8 num_points;
	u8 pad1[2];
	u8 checksum_fix;
} __packed;

static_assert(sizeof(struct himax_event) == 56);

struct himax_ts_data;
struct himax_chip {
	u32 id;
	int (*check_id)(struct himax_ts_data *ts);
	int (*read_events)(struct himax_ts_data *ts, struct himax_event *event,
			   size_t length);
};

struct himax_ts_data {
	const struct himax_chip *chip;
	struct gpio_desc *gpiod_rst;
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct touchscreen_properties props;
	// TODO: Should these go somewhere else?
	wait_queue_head_t	wait;		/* For waiting the interrupt */
	struct mutex		reset_lock;
	struct drm_panel_follower panel_follower;
	struct work_struct	panel_follower_prepare_work;
	bool			is_panel_follower;
	bool			prepare_work_finished;
};

static const struct regmap_config himax_regmap_config = {
	.reg_bits = 8,
	.val_bits = 32,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int himax_bus_enable_burst(struct himax_ts_data *ts)
{
	int error;

	error = regmap_write(ts->regmap, HIMAX_AHB_ADDR_CONTI,
			     HIMAX_AHB_CMD_CONTI);
	if (error)
		return error;

	error = regmap_write(ts->regmap, HIMAX_AHB_ADDR_INCR4,
			     HIMAX_AHB_CMD_INCR4);
	if (error)
		return error;

	return 0;
}

static int himax_bus_read(struct himax_ts_data *ts, u32 address, void *dst,
			  size_t length)
{
	int error;

	if (length > 4) {
		error = himax_bus_enable_burst(ts);
		if (error)
			return error;
	}

	error = regmap_write(ts->regmap, HIMAX_AHB_ADDR_BYTE_0, address);
	if (error)
		return error;

	error = regmap_write(ts->regmap, HIMAX_AHB_ADDR_ACCESS_DIRECTION,
			     HIMAX_AHB_CMD_ACCESS_DIRECTION_READ);
	if (error)
		return error;

	if (length > 4)
		error = regmap_noinc_read(ts->regmap, HIMAX_AHB_ADDR_RDATA_BYTE_0,
					  dst, length);
	else
		error = regmap_read(ts->regmap, HIMAX_AHB_ADDR_RDATA_BYTE_0,
				    dst);
	if (error)
		return error;

	return 0;
}

static void himax_reset(struct himax_ts_data *ts)
{
	gpiod_set_value_cansleep(ts->gpiod_rst, 1);

	/* Delay copied from downstream driver */
	msleep(20);
	gpiod_set_value_cansleep(ts->gpiod_rst, 0);

	/*
	 * The downstream driver doesn't contain this delay but is seems safer
	 * to include it. The range is just a guess that seems to work well.
	 */
	usleep_range(1000, 1100);
}

static int himax_read_product_id(struct himax_ts_data *ts, u32 *product_id)
{
	int error;

	error = himax_bus_read(ts, HIMAX_REG_ADDR_ICID, product_id,
			       sizeof(*product_id));
	if (error)
		return error;

	*product_id >>= 8;
	return 0;
}

static int himax_check_product_id(struct himax_ts_data *ts)
{
	int error;
	u32 product_id;

	error = himax_read_product_id(ts, &product_id);
	if (error)
		return error;

	dev_dbg(&ts->client->dev, "Product id: %x\n", product_id);

	if (product_id == ts->chip->id)
		return 0;

	dev_err(&ts->client->dev, "Unknown product id: %x\n",
		product_id);
	return -EINVAL;
}

static int himax_input_register(struct himax_ts_data *ts)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input_dev->name = "Himax Touchscreen";

	input_set_capability(ts->input_dev, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(ts->input_dev, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 200, 0, 0);

	touchscreen_parse_properties(ts->input_dev, true, &ts->props);

	error = input_mt_init_slots(ts->input_dev, HIMAX_MAX_POINTS,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to initialize MT slots: %d\n", error);
		return error;
	}

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d\n", error);
		return error;
	}

	return 0;
}

static u8 himax_event_get_num_points(const struct himax_event *event)
{
	if (event->num_points == 0xff)
		return 0;
	else
		return event->num_points & 0x0f;
}

static bool himax_process_event_point(struct himax_ts_data *ts,
				      const struct himax_event *event,
				      int point_index)
{
	const struct himax_event_point *point = &event->points[point_index];
	u16 x = be16_to_cpu(point->x);
	u16 y = be16_to_cpu(point->y);
	u8 w = event->majors[point_index];

	if (x == HIMAX_INVALID_COORD || y == HIMAX_INVALID_COORD)
		return false;

	input_mt_slot(ts->input_dev, point_index);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	touchscreen_report_pos(ts->input_dev, &ts->props, x, y, true);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	return true;
}

static void himax_process_event(struct himax_ts_data *ts,
				const struct himax_event *event)
{
	int i;
	int num_points_left = himax_event_get_num_points(event);

	for (i = 0; i < HIMAX_MAX_POINTS && num_points_left > 0; i++) {
		if (himax_process_event_point(ts, event, i))
			num_points_left--;
	}

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

static bool himax_verify_checksum(struct himax_ts_data *ts,
				  const struct himax_event *event)
{
	u8 *data = (u8 *)event;
	int i;
	u16 checksum = 0;

	for (i = 0; i < sizeof(*event); i++)
		checksum += data[i];

	if ((checksum & 0x00ff) != 0) {
		dev_err(&ts->client->dev, "Wrong event checksum: %04x\n",
			checksum);
		return false;
	}

	return true;
}

static int himax_read_events(struct himax_ts_data *ts,
			     struct himax_event *event, size_t length)
{
	return regmap_raw_read(ts->regmap, HIMAX_AHB_ADDR_EVENT_STACK, event,
			       length);
}

static int hx83100a_read_events(struct himax_ts_data *ts,
				struct himax_event *event, size_t length)
{
	return himax_bus_read(ts, HX83100A_REG_FW_EVENT_STACK, event, length);
};

static int himax_handle_input(struct himax_ts_data *ts)
{
	int error;
	struct himax_event event;

	error = ts->chip->read_events(ts, &event, sizeof(event));
	if (error) {
		dev_err(&ts->client->dev, "Failed to read input event: %d\n",
			error);
		return error;
	}

	/*
	 * Only process the current event when it has a valid checksum but
	 * don't consider it a fatal error when it doesn't.
	 */
	if (himax_verify_checksum(ts, &event))
		himax_process_event(ts, &event);

	return 0;
}

static irqreturn_t himax_irq_handler(int irq, void *dev_id)
{
	int error;
	struct himax_ts_data *ts = dev_id;

	error = himax_handle_input(ts);
	if (error)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static int himax_suspend(struct himax_ts_data *ts)
{
	disable_irq(ts->client->irq);
	return 0;
}

static int himax_resume(struct himax_ts_data *ts)
{
	enable_irq(ts->client->irq);
	return 0;
}

static void himax_panel_prepare_work(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data,
					    panel_follower_prepare_work);
	int ret;


	// TODO: Is ther a function for the himax panel that is analogous
	// to i2c_hid_core_initial_power_up in i2c-hid-core.c?
	ret = himax_resume(ts);

	if (ret)
		dev_warn(&ts->client->dev, "Power on failed: %d\n", ret);
	else
		WRITE_ONCE(ts->prepare_work_finished, true);

	/*
	 * The work APIs provide a number of memory ordering guarantees
	 * including one that says that memory writes before schedule_work()
	 * are always visible to the work function, but they don't appear to
	 * guarantee that a write that happened in the work is visible after
	 * cancel_work_sync(). We'll add a write memory barrier here to match
	 * with i2c_hid_core_panel_unpreparing() to ensure that our write to
	 * prepare_work_finished is visible there.
	 */
	smp_wmb();
}

static int himax_panel_prepared(struct drm_panel_follower *follower)
{
	struct himax_ts_data *ts = container_of(follower, struct himax_ts_data, panel_follower);

	/*
	 * Powering on a touchscreen can be a slow process. Queue the work to
	 * the system workqueue so we don't block the panel's power up.
	 */
	WRITE_ONCE(ts->prepare_work_finished, false);
	schedule_work(&ts->panel_follower_prepare_work);

	return 0;
}

static int himax_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct himax_ts_data *ts = container_of(follower, struct himax_ts_data, panel_follower);

	cancel_work_sync(&ts->panel_follower_prepare_work);

	/* Match with ihid_core_panel_prepare_work() */
	smp_rmb();
	if (!READ_ONCE(ts->prepare_work_finished))
		return 0;

	return himax_suspend(ts);
}

static const struct drm_panel_follower_funcs himax_panel_follower_funcs = {
	.panel_prepared = himax_panel_prepared,
	.panel_unpreparing = himax_panel_unpreparing,
};

static int himax_probe(struct i2c_client *client)
{
	int error;
	struct device *dev = &client->dev;
	struct himax_ts_data *ts;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C check functionality failed\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	i2c_set_clientdata(client, ts);
	ts->client = client;
	ts->chip = i2c_get_match_data(client);

	init_waitqueue_head(&ts->wait);
	mutex_init(&ts->reset_lock);
	INIT_WORK(&ts->panel_follower_prepare_work, himax_panel_prepare_work);

	ts->regmap = devm_regmap_init_i2c(client, &himax_regmap_config);
	error = PTR_ERR_OR_ZERO(ts->regmap);
	if (error) {
		dev_err(dev, "Failed to initialize regmap: %d\n", error);
		return error;
	}

	ts->gpiod_rst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	error = PTR_ERR_OR_ZERO(ts->gpiod_rst);
	if (error) {
		dev_err(dev, "Failed to get reset GPIO: %d\n", error);
		return error;
	}

	himax_reset(ts);

	if (ts->chip->check_id) {
		error = himax_check_product_id(ts);
		if (error)
			return error;
	}

	error = himax_input_register(ts);
	if (error)
		return error;

	device_enable_async_suspend(&client->dev);

	error = devm_request_threaded_irq(dev, client->irq, NULL,
					  himax_irq_handler, IRQF_ONESHOT | IRQF_NO_AUTOEN,
					  client->name, ts);
	if (error)
		return error;

	ts->panel_follower.funcs = &himax_panel_follower_funcs;
	error = drm_panel_add_follower(&client->dev, &ts->panel_follower);
	if(!error) {
		printk(KERN_INFO "%s: panel is a follower\n", __func__);
		ts->is_panel_follower = true;
	}

	return 0;
}

static void himax_remove(struct i2c_client *arg_client)
{
	//struct himax_ts_data *ts = container_of(arg_client, struct himax_ts_data, client);
	struct himax_ts_data *ts = i2c_get_clientdata(arg_client);

	if (ts->is_panel_follower)
		drm_panel_remove_follower(&ts->panel_follower);
}

static int himax_pm_suspend(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	if (ts->is_panel_follower)
		return 0;

	return himax_suspend(ts);
}

static int himax_pm_resume(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	if (ts->is_panel_follower)
		return 0;

	return himax_resume(ts);
}

static DEFINE_SIMPLE_DEV_PM_OPS(himax_pm_ops, himax_pm_suspend, himax_pm_resume);

static const struct himax_chip hx83100a_chip = {
	.read_events = hx83100a_read_events,
};

static const struct himax_chip hx83112b_chip = {
	.id = 0x83112b,
	.check_id = himax_check_product_id,
	.read_events = himax_read_events,
};

static const struct i2c_device_id himax_ts_id[] = {
	{ "hx83100a", (kernel_ulong_t)&hx83100a_chip },
	{ "hx83112b", (kernel_ulong_t)&hx83112b_chip },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, himax_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id himax_of_match[] = {
	{ .compatible = "himax,hx83100a", .data = &hx83100a_chip },
	{ .compatible = "himax,hx83112b", .data = &hx83112b_chip },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, himax_of_match);
#endif

static struct i2c_driver himax_ts_driver = {
	.probe = himax_probe,
	.remove = himax_remove,
	.id_table = himax_ts_id,
	.driver = {
		.name = "Himax-hx83112b-TS",
		.of_match_table = of_match_ptr(himax_of_match),
		.pm = pm_sleep_ptr(&himax_pm_ops),
	},
};
module_i2c_driver(himax_ts_driver);

MODULE_AUTHOR("Job Noorman <job@noorman.info>");
MODULE_DESCRIPTION("Himax hx83112b touchscreen driver");
MODULE_LICENSE("GPL");
