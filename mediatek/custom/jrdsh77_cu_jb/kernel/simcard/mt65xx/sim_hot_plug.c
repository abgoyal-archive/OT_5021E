/* linux/drivers/simcard/sim_hot_plug.c
 *
 * ChangeLog
 *
 * Author Kong,Troy 2012-09-28: <yiliang.kong@jrdcom.com>
 * Copyright 2012 TCL. All Rights Reserved.
 *	-initial version.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/sched.h>

#ifdef MT6575
#include <mach/mt6575_gpio.h>
#include <mach/mt6575_typedefs.h>
#endif

#ifdef MT6577
#include <mach/mt6577_gpio.h>
#include <mach/mt6577_typedefs.h>
#endif

#include <cust_eint.h>
#include "cust_gpio_usage.h"

static int debug;
#define	DBG(format, arg...) do { if (debug) printk(KERN_INFO "[SIM/Hotplug] " format "\n"  , ## arg); } while (0)

extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En, kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void), kal_bool auto_umask);

struct sim_card {
	struct delayed_work work;
	struct device *dev;
	int id;
	int sim_onoff;
	int sim_irq;
};
static struct sim_card *sim0_global, *sim1_global;
static DEFINE_MUTEX(sim_card_swap_mutex);
static DECLARE_COMPLETION(sim_card_timeout_wait);
static int sim_card_installed;

static void sim_report_status(struct work_struct *work)
{
       struct sim_card *card = container_of(work, struct sim_card, work);
       int onoff;

       mutex_lock(&sim_card_swap_mutex);

       /* Report kernel object uevent */
       onoff = mt_get_gpio_in(card->sim_irq);

       if (onoff != card->sim_onoff)
       {
               kobject_uevent(&card->dev->kobj, onoff ? KOBJ_ADD : KOBJ_REMOVE);
               card->sim_onoff = onoff;
               printk("[SIM/Hotplug] SIM card%d status(%d) \n", card->id, onoff);
       }       

       mutex_unlock(&sim_card_swap_mutex);
}

void sim0_hot_swap_handler(void)
{
	complete(&sim_card_timeout_wait);
}

void sim1_hot_swap_handler(void)
{
	complete(&sim_card_timeout_wait);
}

static int sim_eint_config(int id)
{
	if (id == 0) { 
		// SIM CARD 1
		/* Configure gpio69 MAX for SIM card0 swap detection */
		mt_set_gpio_mode(GPIO_SIM_1_EINT_PIN, GPIO_SIM_1_EINT_PIN_M_EINT);
		mt_set_gpio_dir(GPIO_SIM_1_EINT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_SIM_1_EINT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_SIM_1_EINT_PIN, GPIO_PULL_UP);

		/* Configure eint3 for SIM card0 swap detection */
		mt65xx_eint_set_sens(CUST_EINT_SIM1_DET_NUM, CUST_EINT_SIM1_DET_SENSITIVE);
		mt65xx_eint_set_hw_debounce(CUST_EINT_SIM1_DET_NUM, CUST_EINT_SIM1_DET_DEBOUNCE_CN);
		mt65xx_eint_registration(CUST_EINT_SIM1_DET_NUM, CUST_EINT_SIM1_DET_POLARITY, CUST_EINT_SIM1_DET_DEBOUNCE_EN, sim0_hot_swap_handler, 1); 
		mt65xx_eint_unmask(CUST_EINT_SIM1_DET_NUM);
	}else if (id == 1) {	
		// SIM CARED 2	
		/* Configure gpio109 MAX for SIM card0 swap detection */
		mt_set_gpio_mode(GPIO_SIM_2_EINT_PIN, GPIO_SIM_2_EINT_PIN_M_EINT);
		mt_set_gpio_dir(GPIO_SIM_2_EINT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_SIM_2_EINT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_SIM_2_EINT_PIN, GPIO_PULL_DOWN);

		/* Configure eint3 for SIM card0 swap detection */
		mt65xx_eint_set_sens(CUST_EINT_SIM2_DET_NUM, CUST_EINT_SIM2_DET_SENSITIVE);
		mt65xx_eint_set_hw_debounce(CUST_EINT_SIM2_DET_NUM, CUST_EINT_SIM2_DET_DEBOUNCE_CN);
		mt65xx_eint_registration(CUST_EINT_SIM2_DET_NUM, CUST_EINT_SIM2_DET_POLARITY, CUST_EINT_SIM2_DET_DEBOUNCE_EN, sim1_hot_swap_handler, 1); 
		mt65xx_eint_unmask(CUST_EINT_SIM2_DET_NUM);
	}else{
		printk("[SIM/Hotplug] Invaild SIM card%d \n", id);
		return -1;
	} 

	return 0;
}

static ssize_t sim_hot_plug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{	
	struct sim_card *card = (struct sim_card*)dev_get_drvdata(dev);
	int onoff;
	
	onoff = mt_get_gpio_in(card->sim_irq);
	card->sim_onoff = onoff;
	printk("[SIM/Hotplug] SIM card%d status(%d) \n", card->id, onoff);

	return sprintf(buf, "%d \n", card->sim_onoff);
}

static struct device_attribute sim_attr = {
	.attr = {.name = "state", .mode = 0644},
	.show = sim_hot_plug_show,
};

static int sim_card_thread(void *arg)
{
	int onoff;

	daemonize("sim_card");
	while (sim_card_installed)
	{
		/* Wait for an Pen down interrupt */
		wait_for_completion_timeout(&sim_card_timeout_wait, HZ);

		mutex_lock(&sim_card_swap_mutex);

		/* Report SIM0 kernel object uevent */
		onoff = mt_get_gpio_in(sim0_global->sim_irq);

		if (onoff != sim0_global->sim_onoff)
		{
			kobject_uevent(&sim0_global->dev->kobj, onoff ? KOBJ_ADD : KOBJ_REMOVE);
			sim0_global->sim_onoff = onoff;
			printk("[SIM/Hotplug] SIM card0 status(%d) \n",onoff);
		}	

		/* Report SIM1 kernel object uevent */
		onoff = mt_get_gpio_in(sim1_global->sim_irq);

		if (onoff != sim1_global->sim_onoff)
		{
			kobject_uevent(&sim1_global->dev->kobj, onoff ? KOBJ_ADD : KOBJ_REMOVE);
			sim1_global->sim_onoff = onoff;
			printk("[SIM/Hotplug] SIM card1 status(%d) \n",onoff);
		}
	
		mutex_unlock(&sim_card_swap_mutex);
	}
}

static int sim_swap_probe(struct platform_device *pdev)
{
	static int retval;
	struct sim_card *card = NULL;
	
	/* Allocation SIM card device region */
	card = kzalloc(sizeof(struct sim_card), GFP_KERNEL);
	if(!card){
		retval = -ENOMEM;
		goto err;
	}

	/* Initialize SIM card platform data */
	switch(pdev->id)
	{
		case 0:
			/* Set SIM card0 driver data */
			card->id = 0;
			card->sim_irq = GPIO_SIM_1_EINT_PIN;
			card->dev = &pdev->dev;
			card->sim_onoff = mt_get_gpio_in(card->sim_irq);
			INIT_DELAYED_WORK(&card->work, sim_report_status);
			sim0_global = card;
			break;
		case 1:
			/* Set SIM card1 driver data */
			card->id = 1;
			card->sim_irq = GPIO_SIM_2_EINT_PIN;
			card->dev = &pdev->dev;
			card->sim_onoff = mt_get_gpio_in(card->sim_irq);
			INIT_DELAYED_WORK(&card->work, sim_report_status);
			sim1_global = card;
			break;
		default:
			printk("[SIM/Hotplug] Invaild SIM card%d \n", pdev->id);
			retval = -EPERM;
			goto err;
	}

	/* Install SIM card kernel thread */
	if(!sim_card_installed)
	{
        	kernel_thread(sim_card_thread, NULL, CLONE_VM | CLONE_FS);
		sim_card_installed = 1;
	}
	platform_set_drvdata(pdev,card);

	/* Configure eint pin of sim1 and sim2 */
	if(sim_eint_config(card->id) != 0) {
		retval = -EIO;
		goto err;
	}

	/* Create hot plug sys attribute file */
	retval = device_create_file(&pdev->dev, &sim_attr);
	if(retval){
		printk(KERN_INFO, "[SIM/Hotplug] Failed to create SIM card%d attribute file!  \n", pdev->id);
		retval = -EBUSY;
		goto err;
	}

	printk(KERN_INFO, "[SIM/Hotplug] SIM card%d installed! \n", pdev->id);
	return 0;
err:
	kfree(card);
	printk(KERN_INFO, "[SIM/Hotplug] SIM card%d failed to install:%d \n", pdev->id, retval);	
	
	return retval;
}

static int sim_swap_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &sim_attr);
	return 0;
}

static struct platform_driver sim_mt65xx = {
	.driver     = {
		.owner  = THIS_MODULE,
		.name   = "sim-mt65xx",
	},
	.probe      = sim_swap_probe,
	.remove     = sim_swap_remove,
};

static int __init sim_swap_init(void)
{
	return platform_driver_register(&sim_mt65xx);
}

static void __exit sim_swap_exit(void)
{
	platform_driver_unregister(&sim_mt65xx);
}

module_init(sim_swap_init);
module_exit(sim_swap_exit);
module_param(debug, int, 0644);

MODULE_AUTHOR("Troy.Kong");
MODULE_DESCRIPTION("SIM card hot swap driver");
MODULE_LICENSE("GPL");
MODULE_PARM_DESC(debug, "Enable debug log");
