/*
 *  dp3t switch driver.
 *
 * Copyright (C) 2011 LG Electronics, Inc.
 * Author: Seungho Park <seungho1.park@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/slab.h>
#include <linux/muic/muic.h>
#include <linux/switch_dp3t.h>
#include <linux/muic/muic_client.h>
#include <asm/gpio.h>
#include <lge/board.h> /* temporary board header file for gpio num */


TYPE_DP3T_MODE dp3t_mode = DP3T_NC;


int dp3t_switch_ctrl_ops(struct muic_client_device* mcdev)
{
	struct dp3t_switch *dp3t;
	unsigned long mode = mcdev->mode; 
	pr_info("dp3t: mcdev->name , %s\n", mcdev->name);
	pr_info("dp3t: mcdev->mode , %d\n", mcdev->mode);
	dp3t = dev_get_drvdata(&mcdev->dev);
	pr_info("dp3t: ctrl_ifx_vbus_gpio = %d\n", dp3t->ctrl_ifx_vbus_gpio);
	pr_info("dp3t: ctrl_gpio1 = %d\n", dp3t->ctrl_gpio1);
	pr_info("dp3t: ctrl_gpio2 = %d\n", dp3t->ctrl_gpio2);
	
	if (mode == MUIC_AP_UART) {
		gpio_set_value(dp3t->ctrl_ifx_vbus_gpio, 0);
		gpio_set_value(dp3t->ctrl_gpio1, 1);
		gpio_set_value(dp3t->ctrl_gpio2, 0);
		pr_info("dp3t: dp3t_switch_ctrl, AP UART is connected to MUIC UART\n");
	} else if (mode == MUIC_CP_UART) {
		gpio_set_value(dp3t->ctrl_ifx_vbus_gpio, 0);
		gpio_set_value(dp3t->ctrl_gpio1, 0);
		gpio_set_value(dp3t->ctrl_gpio2, 1);
		pr_info("dp3t: dp3t_switch_ctrl, CP UART is connected to MUIC UART\n");
	} else if (mode == MUIC_CP_USB) {
		gpio_set_value(dp3t->ctrl_ifx_vbus_gpio, 1);
		gpio_set_value(dp3t->ctrl_gpio1, 1);
		gpio_set_value(dp3t->ctrl_gpio2, 1);
		pr_info("dp3t: dp3t_switch_ctrl, CP USB is connected to MUIC UART\n");
	} else if (mode == MUIC_NONE) {
		gpio_set_value(dp3t->ctrl_ifx_vbus_gpio, 0);
		gpio_set_value(dp3t->ctrl_gpio1, 0);
		gpio_set_value(dp3t->ctrl_gpio2, 0);
		pr_info("dp3t: dp3t_switch_ctrl, None is connected to MUIC UART\n");
	} else {
		/* Just keep the current path */
	}
	
	dp3t_mode = mode;

	return 0;
}
EXPORT_SYMBOL(dp3t_switch_ctrl_ops);
int dp3t_on_none(struct muic_client_device *mcdev)
{
	struct dp3t_switch *dp3t;
	
	dp3t = dev_get_drvdata(&mcdev->dev);

	pr_info("dp3t: dp3t_switch_ctrl, None is connected to MUIC UART\n");

	dp3t_mode = MUIC_NONE;
	return 0;
}

int dp3t_on_ap_uart(struct muic_client_device *mcdev)
{
	struct dp3t_switch *dp3t;
	
	dp3t = dev_get_drvdata(&mcdev->dev);
	
	gpio_set_value(dp3t->ctrl_ifx_vbus_gpio, 0);
	gpio_set_value(dp3t->ctrl_gpio1, 1);
	gpio_set_value(dp3t->ctrl_gpio2, 0);
	pr_info("dp3t: dp3t_switch_ctrl, AP UART is connected to MUIC UART\n");

	dp3t_mode = MUIC_AP_UART;
	return 0;
}

int dp3t_on_cp_uart(struct muic_client_device *mcdev)
{
	struct dp3t_switch *dp3t;
	
	dp3t = dev_get_drvdata(&mcdev->dev);
	
	gpio_set_value(dp3t->ctrl_ifx_vbus_gpio, 0);
	gpio_set_value(dp3t->ctrl_gpio1, 0);
	gpio_set_value(dp3t->ctrl_gpio2, 1);
	pr_info("dp3t: dp3t_switch_ctrl, CP UART is connected to MUIC UART\n");

	dp3t_mode = MUIC_CP_UART;
	return 0;
}

int dp3t_on_cp_usb(struct muic_client_device *mcdev)
{
	struct dp3t_switch *dp3t;
	
	dp3t = dev_get_drvdata(&mcdev->dev);
	
	gpio_set_value(dp3t->ctrl_ifx_vbus_gpio, 1);
	gpio_set_value(dp3t->ctrl_gpio1, 1);
	gpio_set_value(dp3t->ctrl_gpio2, 1);
	pr_info("dp3t: dp3t_switch_ctrl, CP USB is connected to MUIC UART\n");

	dp3t_mode = MUIC_CP_USB;
	return 0;
}

void dp3t_switch_ctrl(TYPE_DP3T_MODE mode)
{
	pr_info("dp3t: dp3t_switch_ctrl()\n");

	if (mode == DP3T_AP_UART) {
		gpio_set_value(GPIO_IFX_USB_VBUS_EN, 0);
		gpio_set_value(GPIO_DP3T_IN_1, 1);
		gpio_set_value(GPIO_DP3T_IN_2, 0);
		pr_info("dp3t: dp3t_switch_ctrl, AP UART is connected to MUIC UART\n");
	} else if (mode == DP3T_CP_UART) {
		gpio_set_value(GPIO_IFX_USB_VBUS_EN, 0);
		gpio_set_value(GPIO_DP3T_IN_1, 0);
		gpio_set_value(GPIO_DP3T_IN_2, 1);
		pr_info("dp3t: dp3t_switch_ctrl, CP UART is connected to MUIC UART\n");
	} else if (mode == DP3T_CP_USB) {
		gpio_set_value(GPIO_IFX_USB_VBUS_EN, 1);
		gpio_set_value(GPIO_DP3T_IN_1, 1);
		gpio_set_value(GPIO_DP3T_IN_2, 1);
		pr_info("dp3t: dp3t_switch_ctrl, CP USB is connected to MUIC UART\n");
	} else if (mode == DP3T_NC) {
		gpio_set_value(GPIO_IFX_USB_VBUS_EN, 0);
		gpio_set_value(GPIO_DP3T_IN_1, 0);
		gpio_set_value(GPIO_DP3T_IN_2, 0);
		pr_info("dp3t: dp3t_switch_ctrl, None is connected to MUIC UART\n");
	} else {
		/* Just keep the current path */
	}
	
	dp3t_mode = mode;

}
EXPORT_SYMBOL(dp3t_switch_ctrl);

static struct muic_client_ops dp3t_ops = {
	.notifier_priority = MUIC_CLIENT_NOTI_DP3T,
	.on_none = dp3t_on_none,
	.on_ap_uart = dp3t_on_ap_uart,
	.on_ap_usb = dp3t_on_ap_uart,
	.on_cp_uart = dp3t_on_cp_uart,
	.on_cp_usb = dp3t_on_cp_usb,
};

static int dp3t_switch_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct dp3t_switch* dp3t = NULL;
	struct dp3t_switch_platform_data *pdata = pdev->dev.platform_data;
	
	dp3t = kzalloc(sizeof(struct dp3t_switch), GFP_KERNEL);

	if(dp3t == NULL) {
		dev_err(&pdev->dev, "%s: failed to alloc new_node\n", __func__);
		return -ENOMEM;
	}

	dp3t->pdev = pdev;
	dp3t->ctrl_gpio1 = pdata->ctrl_gpio1;
	dp3t->ctrl_gpio2 = pdata->ctrl_gpio2;
	dp3t->ctrl_ifx_vbus_gpio = pdata->ctrl_ifx_vbus_gpio;

	ret = muic_client_dev_register(pdev->name, (void*)dp3t, &dp3t_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: failed to muic_client_dev_register\n", __func__);
		goto err_4;
	}

	/*
	 * Initializes gpio_11 (UART_SW1) and gpio_12 (UART_SW2).
	 * Checks if other driver already occupied them.
	 */
	ret = gpio_request(dp3t->ctrl_gpio1, "DP3T switch control 1 GPIO");
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: GPIO %d is already used!\\n",
							__func__, dp3t->ctrl_gpio1);
		goto err_4;
	}

	ret = gpio_direction_output(dp3t->ctrl_gpio1, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: GPIO %d direction initialization failed!\n",
							__func__, dp3t->ctrl_gpio1);
		goto err_3;
	}
	
	ret = gpio_request(dp3t->ctrl_gpio2, "DP3T switch control 2 GPIO");
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: GPIO %d is already used!\\n",
							__func__, dp3t->ctrl_gpio2);
		goto err_3;
	}

	ret = gpio_direction_output(dp3t->ctrl_gpio2, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: GPIO %d direction initialization failed!\n",
							__func__, dp3t->ctrl_gpio2);
		goto err_2;
	}

	ret = gpio_request(dp3t->ctrl_ifx_vbus_gpio, "DP3T switch ifx vbus GPIO");
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: GPIO %d is already used!\\n", 
							__func__, dp3t->ctrl_ifx_vbus_gpio);
		goto err_2;
	}

	ret = gpio_direction_output(dp3t->ctrl_ifx_vbus_gpio, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: GPIO %d direction initialization failed!\n",
						__func__, dp3t->ctrl_ifx_vbus_gpio);
		goto err_1;
	}

	platform_set_drvdata(pdev, dp3t);

	return 0;

err_1:
	gpio_free(pdata->ctrl_ifx_vbus_gpio);
err_2:
	gpio_free(pdata->ctrl_gpio2);
err_3:	
	gpio_free(pdata->ctrl_gpio1);
err_4:
	kfree(dp3t);

	return ret;	
}

static int __devexit dp3t_switch_remove(struct platform_device *pdev)
{
	struct dp3t_switch_platform_data *pdata = pdev->dev.platform_data;
	struct dp3t_switch *dp3t= platform_get_drvdata(pdev);
	
	gpio_free(pdata->ctrl_ifx_vbus_gpio);
	gpio_free(pdata->ctrl_gpio1);
	gpio_free(pdata->ctrl_gpio2);

	kfree(dp3t);

	return 0;
}
	
static struct platform_driver dp3t_switch_driver = {
	.probe		= dp3t_switch_probe,
	.remove		= __devexit_p(dp3t_switch_remove),
	.driver		= {
		.name	= "switch-dp3t",
		.owner	= THIS_MODULE,
	},
};

static int __init dp3t_switch_init(void)
{
	return platform_driver_register(&dp3t_switch_driver);
}

static void __exit dp3t_switch_exit(void)
{
	platform_driver_unregister(&dp3t_switch_driver);
}

module_init(dp3t_switch_init);
module_exit(dp3t_switch_exit);

MODULE_AUTHOR("Seungho Park <seungho1.park@lge.com>");
MODULE_DESCRIPTION("DP3T Switch driver");
MODULE_LICENSE("GPL");
