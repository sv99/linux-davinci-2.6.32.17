/*
 *  DaVinci PWM driver
 *
 *  Copyright (C) 2013 Linear LLC
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/io.h>

#include <asm/div64.h>
#include <mach/hardware.h>

#include "pwm.h"

struct davinci_pwm {
	void __iomem *base;
	struct clk *clk;

	int irq;
	int intr_complete;
	wait_queue_head_t intr_wait;
};

static void pwm_write(struct davinci_pwm *pwm, u32 val, u8 reg)
{
	__raw_writel(val, pwm->base + reg);
}

static u32 pwm_read(struct davinci_pwm *pwm, u8 reg)
{
	return __raw_readl(pwm->base + reg);
}

static void davinci_pwm_config(struct davinci_pwm *pwm, int duty_ns, int period_ns)
{
	unsigned long period, duty;
	unsigned long long clk;
	unsigned long long tmp;
	u32 cfg;

	clk = clk_get_rate(pwm->clk);

	tmp = clk * period_ns;
	do_div(tmp, NSEC_PER_SEC);
	period = tmp;

	tmp = clk * duty_ns;
	do_div(tmp, NSEC_PER_SEC);
	duty = tmp;

	if (duty >= period)
		duty = period - 1;

	cfg = pwm_read(pwm, DAVINCI_PWM_REG_CFG);
	if (cfg & (1 << 17)) {
		/* Enable interrupt */
		pwm_write(pwm, cfg | (1 << 6), DAVINCI_PWM_REG_CFG);
		/* Wait for interrupt */
		wait_event_timeout(pwm->intr_wait,
		                   pwm->intr_complete,
		                   DAVINCI_PWM_TIMEOUT);

		if (pwm->intr_complete) {
			pwm_write(pwm, period, DAVINCI_PWM_REG_PER);
			pwm_write(pwm, duty, DAVINCI_PWM_REG_PH1D);
		}
	} else {
		pwm_write(pwm, period, DAVINCI_PWM_REG_PER);
		pwm_write(pwm, duty, DAVINCI_PWM_REG_PH1D);
	}
}

static void davinci_pwm_stop(struct davinci_pwm *pwm)
{
	u32 cfg;

	cfg = pwm_read(pwm, DAVINCI_PWM_REG_CFG);
	if (cfg & (1 << 17)) {
		if (cfg & 0x1)
			pwm_write(pwm, cfg & 0xFFFFFFFC, DAVINCI_PWM_REG_CFG);

		/* Enable interrupts */
		pwm_write(pwm, cfg | (1 << 6), DAVINCI_PWM_REG_CFG);
		/* Wait for interrupt */
		wait_event_timeout(pwm->intr_wait,
		                   pwm->intr_complete,
		                   DAVINCI_PWM_TIMEOUT);

		if (pwm->intr_complete) {
			cfg &= 0xFFFFFFFC;
			cfg |= 0x1;
			pwm_write(pwm, cfg, DAVINCI_PWM_REG_CFG);
		}
	}

	/* Disable clock */
	clk_disable(pwm->clk);
}

static void davinci_pwm_start(struct davinci_pwm *pwm)
{
	/* Enable clock */
	clk_enable(pwm->clk);

	/* Enable continuous mode, first-phase out high, inactive out low */
	pwm_write(pwm, 0x12, DAVINCI_PWM_REG_CFG);

	/* Start PWM */
	pwm_write(pwm, 0x1, DAVINCI_PWM_REG_START);
}

static ssize_t write_start(struct device *dev, struct device_attribute *attr,
					const char *buffer, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct davinci_pwm *pwm = platform_get_drvdata(pdev);

	davinci_pwm_start(pwm);

	return count;
}

static ssize_t write_stop(struct device *dev, struct device_attribute *attr,
					const char *buffer, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct davinci_pwm *pwm = platform_get_drvdata(pdev);

	davinci_pwm_stop(pwm);

	return count;
}


static ssize_t write_config(struct device *dev, struct device_attribute *attr,
					const char *buffer, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct davinci_pwm *pwm = platform_get_drvdata(pdev);
	int duty_ns;
	int period_ns;

	if (sscanf(buffer, "%d %d", &duty_ns, &period_ns) == 2)
		davinci_pwm_config(pwm, duty_ns, period_ns);

	return count;
}

DEVICE_ATTR(start, 0644, NULL, write_start);
DEVICE_ATTR(stop, 0644, NULL, write_stop);
DEVICE_ATTR(config, 0644, NULL, write_config);

static struct attribute *pwm_attrs[] = {
	&dev_attr_start.attr,
	&dev_attr_stop.attr,
	&dev_attr_config.attr,
	NULL
};

static struct attribute_group davinci_pwm_attr_group = {
	.attrs = pwm_attrs,
};

static irqreturn_t pwm_isr(int irq, void *dev_id)
{
	struct davinci_pwm *pwm = dev_id;
	u32 cfg;

	/* Disable PWM interrupts */
	cfg = pwm_read(pwm, DAVINCI_PWM_REG_CFG);
	cfg &= ~(1 << 6);
	pwm_write(pwm, cfg, DAVINCI_PWM_REG_CFG);

	pwm->intr_complete = 1;
	wake_up(&pwm->intr_wait);

	return IRQ_HANDLED;
}

static int __init davinci_pwm_probe(struct platform_device *pdev)
{
	struct davinci_pwm *pwm;
	struct resource *mem;
	struct clk *clk;
	char clkname[16];
	void __iomem *base;
	int irq;
	int ret;

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "irq resource missing\n");
		ret = -ENODEV;
		goto err;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "mem resource missing\n");
		ret = -ENOMEM;
		goto err;
	}

	snprintf(clkname, sizeof(clkname), "pwm%d", pdev->id);
	clk = clk_get(&pdev->dev, clkname);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock %s\n", clkname);
		ret = -ENODEV;
		goto err;
	}
	clk_enable(clk);

	if (!request_mem_region(mem->start, resource_size(mem), pdev->name)) {
		dev_err(&pdev->dev, "failed to request mem region\n");
		ret = -EBUSY;
		goto err_put;
	}

	base = ioremap_nocache(mem->start, resource_size(mem));
	if (!base) {
		dev_err(&pdev->dev, "failed to remap mem\n");
		ret = -ENOMEM;
		goto err_release;
	}

	pwm = kzalloc(sizeof(*pwm), GFP_KERNEL);
	if (!pwm) {
		dev_err(&pdev->dev, "unable to allocate pwm device\n");
		ret = -ENOMEM;
		goto err_iounmap;
	}

	memset(pwm, 0, sizeof(*pwm));
	pwm->base = base;
	pwm->irq = irq;
	pwm->clk = clk;
	init_waitqueue_head(&pwm->intr_wait);
	platform_set_drvdata(pdev, pwm);

	ret = request_irq(irq, pwm_isr, IRQF_SHARED, dev_name(&pdev->dev), pwm);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto err_kfree;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &davinci_pwm_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs entries\n");
		goto err_irqfree;
	}

	dev_info(&pdev->dev, "device registered (base=%p, irq=%d)\n", base, irq);

	return 0;

err_irqfree:
	free_irq(irq, pwm);
err_kfree:
	kfree(pwm);
err_iounmap:
	iounmap(base);
err_release:
	release_mem_region(mem->start, resource_size(mem));
err_put:
	clk_put(clk);
err:
	return ret;
}

static int __exit davinci_pwm_remove(struct platform_device *pdev)
{
	struct davinci_pwm *pwm = platform_get_drvdata(pdev);

	iounmap(pwm->base);
	clk_disable(pwm->clk);
	clk_put(pwm->clk);
	free_irq(pwm->irq, pwm);
	kfree(pwm);

	return 0;
}

static struct platform_driver davinci_pwm_driver = {
	.remove		= __exit_p(davinci_pwm_remove),
	.driver		= {
		.name	= "davinci_pwm",
	},
};
MODULE_ALIAS("platform:davinci_pwm");

static int __init davinci_pwm_init(void)
{
	return platform_driver_probe(&davinci_pwm_driver, davinci_pwm_probe);
}
module_init(davinci_pwm_init);

static void davinci_pwm_exit(void)
{
	platform_driver_unregister(&davinci_pwm_driver);
}
module_exit(davinci_pwm_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI DaVinci PWM");
MODULE_AUTHOR("Adam J. Porter <aporter@linearcorp.com>");
