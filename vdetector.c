/*
 * drivers/input/misc/vdetector.c - voltage detector device driver
 *
 * Copyright (c) 2019 Hsinko Yu <hsinkoyu@gmail.com>
 *
 * This file is released under the GPLv2
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define UEVENT_ENV_VOLTAGE_DETECTED           "VOLTAGE_DETECTED=YES"
#define UEVENT_ENV_VOLTAGE_DETECTED_CONTINUED "VOLTAGE_DETECTED=YES_CONTINUED"
#define UEVENT_ENV_VOLTAGE_NOT_DETECTED       "VOLTAGE_DETECTED=NO"
#define UEVENT_ENV_DETECTOR_PUSHED            "VOLTAGE_DETECTOR=PUSHED"
#define UEVENT_ENV_DETECTOR_PULLED            "VOLTAGE_DETECTOR=PULLED"

enum _vdetector_input_event {
	INPUT_EVENT_VOLTAGE_NOT_DETECTED,
	INPUT_EVENT_VOLTAGE_DETECTED,
	INPUT_EVENT_VOLTAGE_DETECTED_CONTINUED,

	INPUT_EVENT_DETECTOR_PULLED,
	INPUT_EVENT_DETECTOR_PUSHED,

	INPUT_EVENT_COUNT
};

struct _vdetector_input {
	unsigned int type;
	unsigned int code;
	int value;
} vdetector_input_events[INPUT_EVENT_COUNT] = {
	{EV_KEY, KEY_F10, 0},
	{EV_KEY, KEY_F10, 1},
	{EV_KEY, KEY_F10, 1},
	{EV_KEY, KEY_F11, 0},
	{EV_KEY, KEY_F11, 1},
};

#define PUSH_PUSH_STABILIZATION_TIME_MS       100

struct vdetector_driver {
	struct platform_device *pdev;

	unsigned int en_gpio;
	unsigned int enabled_level;

	unsigned int result_gpio;
	int result_irq;
	unsigned int result_irq_flags;

	/* square wave duty time in millisecond when voltage detected */
	unsigned int duty_high_ms;
	unsigned int duty_low_ms;
	unsigned int duty_timeout_ms;

	/* a high-resolution timer to determine the detected result */
	struct hrtimer result_timer;

	unsigned int push_gpio;
	int push_irq;
	unsigned int push_irq_flags;
	int pushed_level;

	struct workqueue_struct *workqueue;

	/* uevent handling */
	struct work_struct uevent_work;
	char *uevent_envp;

	/* push-push handling */
	struct delayed_work pp_work;

	/* as an input device */
	struct input_dev *idev;
};

static void vdetector_uevent_worker(struct work_struct *work) {
	struct vdetector_driver *drv_data = container_of(work,
		struct vdetector_driver, uevent_work);
	char *envp_ext[] = {drv_data->uevent_envp, NULL};

	if (kobject_uevent_env(&drv_data->pdev->dev.kobj, KOBJ_CHANGE, envp_ext))
		pr_err("failed sending uevent with env '%s'\n", envp_ext[0]);
	else
		pr_debug("notify userspace uevent '%s'\n", envp_ext[0]);
}

static void vdetector_input_report(struct vdetector_driver *drv_data,
	enum _vdetector_input_event event)
{
	/*
	 * Until key up event, the key state keeps down. No more key down is needed.
	 */
	if (event == INPUT_EVENT_VOLTAGE_DETECTED_CONTINUED)
		return;

	input_event(drv_data->idev, vdetector_input_events[event].type,
		vdetector_input_events[event].code,
		vdetector_input_events[event].value);
	input_sync(drv_data->idev);
}

static void vdetector_send_uevent(struct vdetector_driver *drv_data,
	char *envp)
{
	drv_data->uevent_envp = envp;
	queue_work(drv_data->workqueue, &drv_data->uevent_work);
}

static enum hrtimer_restart result_timer_expiry(struct hrtimer *timer)
{
	struct vdetector_driver *drv_data = container_of(timer,
		struct vdetector_driver, result_timer);

	vdetector_input_report(drv_data, INPUT_EVENT_VOLTAGE_NOT_DETECTED);
	vdetector_send_uevent(drv_data, UEVENT_ENV_VOLTAGE_NOT_DETECTED);

	return HRTIMER_NORESTART;
}

static irqreturn_t result_irq_handler(int irq, void *p)
{
	struct vdetector_driver *drv_data = (struct vdetector_driver *)p;
	bool detected_continues = false;
	unsigned int expiry_ms;

	expiry_ms = gpio_get_value(drv_data->result_gpio) ?
		drv_data->duty_high_ms : drv_data->duty_low_ms;
	expiry_ms += drv_data->duty_timeout_ms;

	if (hrtimer_active(&drv_data->result_timer)) {
		if (hrtimer_callback_running(&drv_data->result_timer)) {
			pr_info("voltage disappeared and wait for sending its uevent\n");
			/* wait for the handler to finish */
			hrtimer_cancel(&drv_data->result_timer);
		} else {
			/* detected continues */
			detected_continues = true;
		}
	}

	/* (re)start the timer */
	hrtimer_start(&drv_data->result_timer, ms_to_ktime(expiry_ms),
		HRTIMER_MODE_REL);

	vdetector_input_report(drv_data, detected_continues ?
		INPUT_EVENT_VOLTAGE_DETECTED_CONTINUED : INPUT_EVENT_VOLTAGE_DETECTED);

	/* bottom-half processing */
	vdetector_send_uevent(drv_data, detected_continues ?
		UEVENT_ENV_VOLTAGE_DETECTED_CONTINUED : UEVENT_ENV_VOLTAGE_DETECTED);

	return IRQ_HANDLED;
}

static void vdetector_pp_worker(struct work_struct *work) {
	struct vdetector_driver *drv_data = container_of(work,
		struct vdetector_driver, pp_work.work);
	bool pushed = gpio_get_value(drv_data->push_gpio) == drv_data->pushed_level ?
		true : false;

	vdetector_input_report(drv_data, pushed ?
		INPUT_EVENT_DETECTOR_PUSHED : INPUT_EVENT_DETECTOR_PULLED);
	vdetector_send_uevent(drv_data, pushed ?
		UEVENT_ENV_DETECTOR_PUSHED : UEVENT_ENV_DETECTOR_PULLED);
	gpio_set_value(drv_data->en_gpio, pushed ?
		drv_data->enabled_level : !drv_data->enabled_level);

	pr_info("turn %s voltage detector\n", pushed ? "on" : "off");
}

static irqreturn_t push_irq_handler(int irq, void *p)
{
	struct vdetector_driver *drv_data = (struct vdetector_driver *)p;
	unsigned long delay = msecs_to_jiffies(PUSH_PUSH_STABILIZATION_TIME_MS);

	queue_delayed_work(drv_data->workqueue, &drv_data->pp_work, delay);

	/*
	 * for push-push switch, we only care about the change on the inverse
	 * level to prevent un-necessary triggers.
	 */
	irq_set_irq_type(irq, gpio_get_value(drv_data->push_gpio) ?
		IRQ_TYPE_LEVEL_LOW : IRQ_TYPE_LEVEL_HIGH);

	return IRQ_HANDLED;
}

static ssize_t vdetector_result_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct vdetector_driver *drv_data = platform_get_drvdata(pdev);

	return sprintf(buf, "ELECTRICITY=%s\n",
		hrtimer_active(&drv_data->result_timer) ? "YES" : "NO");
}

static DEVICE_ATTR(result, 0444, vdetector_result_show, NULL);

static ssize_t vdetector_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct vdetector_driver *drv_data = platform_get_drvdata(pdev);

	return sprintf(buf, "%d (%s)\n", gpio_get_value(drv_data->en_gpio),
		gpio_get_value(drv_data->push_gpio) == drv_data->pushed_level ?
		"pushed" : "pulled");
}

static ssize_t vdetector_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct vdetector_driver *drv_data = platform_get_drvdata(pdev);
	int enable = -1;

	if (buf != NULL) {
		if (0 == strncmp(buf, "1", 1))
			enable = 1;
		else if (0 == strncmp(buf, "0", 1))
			enable = 0;
	}

	if (enable != -1) {
		if (enable != gpio_get_value(drv_data->en_gpio))
			gpio_set_value(drv_data->en_gpio, enable);
	} else
		pr_err("valid value [0,1]\n");

	return count;
}

static DEVICE_ATTR(enable, 0644, vdetector_enable_show, vdetector_enable_store);

static int vdetector_parse_dt(struct platform_device *pdev)
{
	struct vdetector_driver *drv_data = platform_get_drvdata(pdev);
	struct device_node *dtn = pdev->dev.of_node;
	int ret;

	ret = of_get_named_gpio_flags(dtn, "en-gpio", 0, &drv_data->enabled_level);
	if (!gpio_is_valid(ret)) {
		pr_err("failed reading property 'en-gpio', err=%d\n", ret);
		return ret;
	}
	drv_data->en_gpio = ret;

	ret = of_get_named_gpio_flags(dtn, "push-gpio", 0,
		&drv_data->push_irq_flags);
	if (!gpio_is_valid(ret)) {
		pr_err("failed reading property 'push-gpio', err=%d\n", ret);
		return ret;
	}
	drv_data->push_gpio = ret;

	/* default low when detector pushed */
	drv_data->pushed_level = 0;
	of_property_read_u32(dtn, "pushed-level", &drv_data->pushed_level);

	ret = of_get_named_gpio_flags(dtn, "result-gpio", 0,
		&drv_data->result_irq_flags);
	if (!gpio_is_valid(ret)) {
		pr_err("failed reading property 'result-gpio', err=%d\n", ret);
		return ret;
	}
	drv_data->result_gpio = ret;

	/* default square wave duty time */
	drv_data->duty_high_ms = 30;
	of_property_read_u32(dtn, "duty-high-ms", &drv_data->duty_high_ms);
	drv_data->duty_low_ms = 40;
	of_property_read_u32(dtn, "duty-low-ms", &drv_data->duty_low_ms);
	drv_data->duty_timeout_ms = 8;
	of_property_read_u32(dtn, "duty-timeout-ms", &drv_data->duty_timeout_ms);

	return 0;
}

enum _gpio_direction {
	GPIO_DIRECTION_INPUT,
	GPIO_DIRECTION_OUTPUT,
};

static int vdetector_gpio_setup(unsigned int gpio, const char *label,
	enum _gpio_direction direction, int value)
{
	int ret;

	ret = gpio_request(gpio, label);
	if (ret < 0) {
		pr_err("failed requesting gpio_%d as '%s', err=%d\n", gpio, label, ret);
		return ret;
	}

	if (direction == GPIO_DIRECTION_INPUT)
		ret = gpio_direction_input(gpio);
	else if (direction == GPIO_DIRECTION_OUTPUT)
		ret = gpio_direction_output(gpio, value);

	if (ret < 0) {
		pr_err("failed setting gpio_%d direction, err=%d\n", gpio, ret);
		gpio_free(gpio);
		return ret;
	}

	return 0;
}

static void vdetector_gpio_release(unsigned int gpio)
{
	gpio_free(gpio);
}

/**
 * vdetector_irq_setup - get the IRQ corresponding to @gpio and then request it
 * @gpio: an already requested gpio
 * @irq:
 */
static int vdetector_irq_setup(unsigned int gpio, int *irq,
	irq_handler_t handler, unsigned long flags, const char *name, void *dev)
{
	int ret;

	ret = gpio_to_irq(gpio);
	if (ret < 0) {
		pr_err("failed retrieving the irq corresponding to gpio_%d, err=%d\n",
			gpio, ret);
		return ret;
	}
	*irq = ret;

	ret = request_irq(*irq, handler, flags, name, dev);
	if (ret < 0) {
		pr_err("failed requesting irq(%d) as '%s', err=%d\n", *irq, name, ret);
		return ret;
	}

	return 0;
}

static void vdetector_irq_release(int *irq, void *dev_id)
{
	free_irq(*irq, dev_id);
}

static int vdetector_probe(struct platform_device *pdev)
{
	struct vdetector_driver *drv_data;
	int ret;
	struct input_dev *input;
	bool idev_registered = false;
	int i;

	drv_data = kzalloc(sizeof(struct vdetector_driver), GFP_KERNEL);
	if (!drv_data) {
		pr_err("failed memory allocation\n");
		ret = -ENOMEM;
		goto err_nomem;
	}

	input = input_allocate_device();
	if (input == NULL) {
		pr_err("failed input device allocation\n");
		ret = -ENOMEM;
		goto err_input_dev_allocation;
	}

	input->name = pdev->name;
	input->phys = "soc/soc:voltage_detector/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	for (i = 0; i < INPUT_EVENT_COUNT; i++) {
		input_set_capability(input, vdetector_input_events[i].type,
			vdetector_input_events[i].code);
	}

	ret = input_register_device(input);
	if (ret) {
		pr_err("failed input device registration\n");
		goto err_input_dev_registration;
	}

	/* to know each other */
	drv_data->pdev = pdev;
	drv_data->idev = input;
	platform_set_drvdata(pdev, drv_data);
	input_set_drvdata(input, drv_data);

	ret = vdetector_parse_dt(pdev);
	if (ret) {
		pr_err("failed parsing device tree node, err=%d\n", ret);
		goto err_dt;
	}

	/* enable pin configuration */
	ret = vdetector_gpio_setup(drv_data->en_gpio, "vdetector_en",
		GPIO_DIRECTION_OUTPUT, !drv_data->enabled_level);
	if (ret)
		goto err_en_gpio_setup;

	/* result pin configuration */
	ret = vdetector_gpio_setup(drv_data->result_gpio, "vdetector_result",
		GPIO_DIRECTION_INPUT, 0);
	if (ret)
		goto err_result_gpio_setup;

	ret = vdetector_irq_setup(drv_data->result_gpio, &drv_data->result_irq,
		result_irq_handler, drv_data->result_irq_flags, "vdetector_result_irq",
		drv_data);
	if (ret)
		goto err_result_irq_setup;

	/* push pin configuration */
	ret = vdetector_gpio_setup(drv_data->push_gpio, "vdetector_push",
		GPIO_DIRECTION_INPUT, 0);
	if (ret)
		goto err_push_gpio_setup;

	ret = vdetector_irq_setup(drv_data->push_gpio, &drv_data->push_irq,
		push_irq_handler, IRQ_TYPE_NONE, "vdetector_push_irq", drv_data);
	if (ret)
		goto err_push_irq_setup;

	/* a global workqueue */
	drv_data->workqueue = create_singlethread_workqueue("workqueue");
	if (drv_data->workqueue == NULL) {
		ret = -ENOMEM;
		goto err_workqueue;
	}

	/* uevent notification */
	INIT_WORK(&drv_data->uevent_work, vdetector_uevent_worker);

	/* a delayed work handles push-push stabilization */
	INIT_DELAYED_WORK(&drv_data->pp_work, vdetector_pp_worker);

	/* sysfs device attributes */
	ret = device_create_file(&pdev->dev, &dev_attr_enable);
	if (ret) {
		pr_err("failed creating 'enable' sysfs device attribute, err=%d\n", ret);
		goto err_en_gpio_sysfs;
	}
	ret = device_create_file(&pdev->dev, &dev_attr_result);
	if (ret) {
		pr_err("failed creating 'result' sysfs device attribute, err=%d\n", ret);
		goto err_result_sysfs;
	}

	/* detected result determinaion */
	hrtimer_init(&drv_data->result_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv_data->result_timer.function = result_timer_expiry;

	/* initial go */
	push_irq_handler(drv_data->push_irq, drv_data);

	pr_info("devpath=%s\n", kobject_get_path(&drv_data->pdev->dev.kobj, GFP_KERNEL));

	return 0;

err_result_sysfs:
	device_remove_file(&pdev->dev, &dev_attr_enable);
err_en_gpio_sysfs:
	destroy_workqueue(drv_data->workqueue);
err_workqueue:
	vdetector_irq_release(&drv_data->push_irq, drv_data);
err_push_irq_setup:
	vdetector_gpio_release(drv_data->push_gpio);
err_push_gpio_setup:
	vdetector_irq_release(&drv_data->result_irq, drv_data);
err_result_irq_setup:
	vdetector_gpio_release(drv_data->result_gpio);
err_result_gpio_setup:
	vdetector_gpio_release(drv_data->en_gpio);
err_en_gpio_setup:
err_dt:
	idev_registered = true;
err_input_dev_registration:
	if (idev_registered)
		input_unregister_device(input);
	else
		input_free_device(input);
err_input_dev_allocation:
	kfree(drv_data);
err_nomem:

	return ret;
}

static int vdetector_remove(struct platform_device *pdev)
{
	struct vdetector_driver *drv_data = platform_get_drvdata(pdev);

	hrtimer_cancel(&drv_data->result_timer);
	device_remove_file(&pdev->dev, &dev_attr_result);
	device_remove_file(&pdev->dev, &dev_attr_enable);

	destroy_workqueue(drv_data->workqueue);
	vdetector_irq_release(&drv_data->push_irq, drv_data);
	vdetector_gpio_release(drv_data->push_gpio);
	vdetector_irq_release(&drv_data->result_irq, drv_data);
	vdetector_gpio_release(drv_data->result_gpio);
	vdetector_gpio_release(drv_data->en_gpio);
	input_unregister_device(drv_data->idev);
	kfree(drv_data);

	return 0;
}

static const struct of_device_id vdetector_of_match[] = {
	{ .compatible = "voltage_detector", },
	{ }
};
MODULE_DEVICE_TABLE(of, vdetector_of_match);

#if defined(CONFIG_SUSPEND) && defined(CONFIG_PM_SLEEP)
static int vdetector_suspend(struct device *dev)
{
	pr_debug("suspend\n");
	return 0;
}

static int vdetector_resume(struct device *dev)
{
	pr_debug("resume\n");
	return 0;
}
#else
#define vdetector_suspend NULL
#define vdetector_resume NULL
#endif

static const struct dev_pm_ops vdetector_pm_ops =
{
	.suspend = vdetector_suspend,
	.resume = vdetector_resume,
};

static struct platform_driver vdetector_driver = {
	.driver = {
		.name = "vdetector",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(vdetector_of_match),
		.pm = &vdetector_pm_ops,
	},
	.probe = vdetector_probe,
	.remove = vdetector_remove,
};

static int __init vdetector_init(void)
{
	return platform_driver_register(&vdetector_driver);
}

static void __exit vdetector_exit(void)
{
	platform_driver_unregister(&vdetector_driver);
}

module_init(vdetector_init);
module_exit(vdetector_exit);

MODULE_AUTHOR("Hsinko Yu <hsinkoyu@gmail.com>");
MODULE_DESCRIPTION("Driver for voltage detector");
MODULE_LICENSE("GPL v2");
