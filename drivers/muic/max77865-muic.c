/*
 * max77865-muic.c - MUIC driver for the Maxim 77865
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <linux/mfd/max77865.h>
#include <linux/mfd/max77865-private.h>
#include <linux/muic/muic.h>
#include <linux/muic/max77865-muic-hv-typedef.h>
#include <linux/muic/max77865-muic.h>

#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#if defined(CONFIG_VBUS_NOTIFIER)
#include <linux/vbus_notifier.h>
#endif /* CONFIG_VBUS_NOTIFIER */

#if defined(CONFIG_SEC_SYSFS)
#include <linux/sec_sysfs.h>
#endif

struct max77865_muic_data *g_muic_data;

#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
extern bool max77865_muic_check_is_enable_afc(struct max77865_muic_data *muic_data,
		muic_attached_dev_t new_dev);
extern muic_attached_dev_t max77865_hv_muic_check_id_err(struct max77865_muic_data *muic_data,
		muic_attached_dev_t new_dev);
extern void max77865_muic_afc_reset(struct max77865_muic_data *muic_data);
extern void max77865_muic_clear_hv_control(struct max77865_muic_data *muic_data);
extern void max77865_muic_afc_hv_set(struct max77865_muic_data *muic_data, int voltage);
extern void max77865_muic_qc_hv_set(struct max77865_muic_data *muic_data, int voltage);
extern void max77865_muic_prepare_afc_charger(struct max77865_muic_data *muic_data);
extern void max77865_muic_handle_vdnmon(struct max77865_muic_data *muic_data);
extern void max77865_muic_handle_detect_dev_afc(struct max77865_muic_data *muic_data);
extern void max77865_muic_handle_detect_dev_mpnack(struct max77865_muic_data *muic_data);
extern void max77865_muic_handle_vbadc(struct max77865_muic_data *muic_data);
extern void max77865_hv_muic_init_detect(struct max77865_muic_data *muic_data);
extern void max77865_hv_muic_remove(struct max77865_muic_data *muic_data);
#endif /* CONFIG_HV_MUIC_MAX77865_AFC */

/* INT_BC / INT_FC / INT_GP bitmasks */
#define INT_BC_CHGTYP_MASK	(1 << 0)
#define INT_BC_DCDTMR_MASK	(1 << 1)
#define INT_BC_PRCHGTYP_MASK	(1 << 2)
#define INT_BC_VBVOLT_MASK	(1 << 3)
#define INT_BC_DNVDATREF_MASK	(1 << 5)
#define INT_FC_MPNACK_MASK	(1 << 3)
#define INT_FC_MRXRDY_MASK	(1 << 7)
#define INT_GP_VBADC_MASK	(1 << 2)
#define INT_GP_UIDGND_MASK	(1 << 5)
#define INT_GP_UID_MASK		(1 << 6)

struct max77865_muic_vps_data {
	u8			uid;
	vbvolt_t		vbvolt;
	chgtyp_t		chgtyp;
	const char		*vps_name;
	muic_attached_dev_t	attached_dev;
	max77865_reg_ctrl1_t	control1;
};

static const struct max77865_muic_vps_data muic_vps_table[] = {
	{
		.uid		= UID_JIG_USB_ON,
		.vbvolt		= VB_DONTCARE,
		.chgtyp		= CHGTYP_DONTCARE,
		.vps_name	= "Jig USB On",
		.attached_dev	= ATTACHED_DEV_JIG_USB_ON_MUIC,
		.control1	= CTRL1_USB,
	},
	{
		.uid		= UID_JIG_UART_OFF,
		.vbvolt		= VB_LOW,
		.chgtyp		= CHGTYP_DONTCARE,
		.vps_name	= "Jig UART Off",
		.attached_dev	= ATTACHED_DEV_JIG_UART_OFF_MUIC,
		.control1	= CTRL1_UART,
	},
	{
		.uid		= UID_JIG_UART_OFF,
		.vbvolt		= VB_HIGH,
		.chgtyp		= CHGTYP_DONTCARE,
		.vps_name	= "Jig UART Off + VB",
		.attached_dev	= ATTACHED_DEV_JIG_UART_OFF_VB_MUIC,
		.control1	= CTRL1_UART,
	},
	{
		.uid		= UID_OPEN,
		.vbvolt		= VB_HIGH,
		.chgtyp		= CHGTYP_USB,
		.vps_name	= "USB",
		.attached_dev	= ATTACHED_DEV_USB_MUIC,
		.control1	= CTRL1_USB,
	},
	{
		.uid		= UID_OPEN,
		.vbvolt		= VB_HIGH,
		.chgtyp		= CHGTYP_CDP,
		.vps_name	= "CDP",
		.attached_dev	= ATTACHED_DEV_CDP_MUIC,
		.control1	= CTRL1_USB,
	},
	{
		.uid		= UID_OPEN,
		.vbvolt		= VB_HIGH,
		.chgtyp		= CHGTYP_DEDICATED_CHARGER,
		.vps_name	= "TA",
		.attached_dev	= ATTACHED_DEV_TA_MUIC,
		.control1	= CTRL1_OPEN,
	},
};

static int max77865_muic_read_reg(struct i2c_client *i2c, u8 reg, u8 *value)
{
	int ret = max77865_read_reg(i2c, reg, value);

	if (ret < 0)
		pr_err("%s:%s err read reg(0x%02x) ret(%d)\n",
				MUIC_DEV_NAME, __func__, reg, ret);
	return ret;
}

static int max77865_muic_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	int ret = max77865_write_reg(i2c, reg, value);

	if (ret < 0)
		pr_err("%s:%s err write reg(0x%02x)=0x%02x ret(%d)\n",
				MUIC_DEV_NAME, __func__, reg, value, ret);
	return ret;
}

static void max77865_muic_handle_vbus(struct max77865_muic_data *muic_data)
{
#if defined(CONFIG_VBUS_NOTIFIER)
	vbus_status_t status = (muic_data->bcstatus1 & BC_STATUS1_VBVOLT_MASK) ?
			STATUS_VBUS_HIGH : STATUS_VBUS_LOW;

	pr_info("%s:%s <%d>\n", MUIC_DEV_NAME, __func__, status);
	vbus_notifier_handle(status);
#endif
}

static int muic_lookup_vps_table(muic_attached_dev_t new_dev,
		const struct max77865_muic_vps_data **vps)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(muic_vps_table); i++) {
		if (muic_vps_table[i].attached_dev == new_dev) {
			*vps = &muic_vps_table[i];
			return 0;
		}
	}
	return -EINVAL;
}

static int max77865_switch_path(struct max77865_muic_data *muic_data,
		max77865_reg_ctrl1_t reg_val)
{
	u8 val = 0;
	int ret;

	ret = max77865_muic_read_reg(muic_data->i2c, MAX77865_MUIC_REG_CONTROL1, &val);
	if (ret < 0 || val == reg_val)
		return ret;

	return max77865_muic_write_reg(muic_data->i2c, MAX77865_MUIC_REG_CONTROL1, reg_val);
}

static void com_to_open(struct max77865_muic_data *muic_data)
{
	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);
	max77865_switch_path(muic_data, CTRL1_OPEN);
}

static int com_to_usb_ap(struct max77865_muic_data *muic_data)
{
	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);
	return max77865_switch_path(muic_data, CTRL1_USB);
}

static int com_to_usb_cp(struct max77865_muic_data *muic_data)
{
	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);
	return max77865_switch_path(muic_data, CTRL1_USB_CP);
}

static void com_to_uart_ap(struct max77865_muic_data *muic_data)
{
	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);
	max77865_switch_path(muic_data, CTRL1_UART);
}

static void com_to_uart_cp(struct max77865_muic_data *muic_data)
{
	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);
	max77865_switch_path(muic_data, CTRL1_UART_CP);
}

static int switch_to_ap_uart(struct max77865_muic_data *muic_data,
		muic_attached_dev_t new_dev)
{
	switch (new_dev) {
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
		com_to_uart_ap(muic_data);
		break;
	default:
		break;
	}
	return 0;
}

static int switch_to_cp_uart(struct max77865_muic_data *muic_data,
		muic_attached_dev_t new_dev)
{
	switch (new_dev) {
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
		com_to_uart_cp(muic_data);
		break;
	default:
		break;
	}
	return 0;
}

static void max77865_muic_enable_chgdet(struct max77865_muic_data *muic_data)
{
	max77865_update_reg(muic_data->i2c, MAX77865_MUIC_REG_CONTROL1_BC,
			BC_CONTROL1_CHGDETEN_MASK, BC_CONTROL1_CHGDETEN_MASK);
}

static void max77865_muic_disable_chgdet(struct max77865_muic_data *muic_data)
{
	max77865_update_reg(muic_data->i2c, MAX77865_MUIC_REG_CONTROL1_BC,
			0, BC_CONTROL1_CHGDETEN_MASK);
}

static u8 max77865_muic_get_uid_value(struct max77865_muic_data *muic_data)
{
	u8 gp_status = 0;

	max77865_muic_read_reg(muic_data->i2c, MAX77865_MUIC_REG_STATUS_GP, &gp_status);
	return gp_status & GP_STATUS_UID_MASK;
}

static u8 max77865_muic_get_vbadc_value(struct max77865_muic_data *muic_data)
{
	u8 gp_status = 0;

	max77865_muic_read_reg(muic_data->i2c, MAX77865_MUIC_REG_STATUS_GP, &gp_status);
	return (gp_status & GP_STATUS_VBADC_MASK) >> GP_STATUS_VBADC_SHIFT;
}

/* Sysfs attributes */
static ssize_t max77865_muic_show_uart_sel(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77865_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *pdata = muic_data->pdata;
	const char *mode = "UNKNOWN\n";

	switch (pdata->uart_path) {
	case MUIC_PATH_UART_AP:
		mode = "AP\n";
		break;
	case MUIC_PATH_UART_CP:
		mode = "CP\n";
		break;
	default:
		break;
	}

	return snprintf(buf, PAGE_SIZE, "%s", mode);
}

static ssize_t max77865_muic_set_uart_sel(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max77865_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *pdata = muic_data->pdata;

	mutex_lock(&muic_data->muic_mutex);
	if (!strncasecmp(buf, "AP", 2)) {
		pdata->uart_path = MUIC_PATH_UART_AP;
		switch_to_ap_uart(muic_data, muic_data->attached_dev);
	} else if (!strncasecmp(buf, "CP", 2)) {
		pdata->uart_path = MUIC_PATH_UART_CP;
		switch_to_cp_uart(muic_data, muic_data->attached_dev);
	} else {
		pr_warn("%s:%s invalid value\n", MUIC_DEV_NAME, __func__);
	}
	mutex_unlock(&muic_data->muic_mutex);

	return count;
}

static ssize_t max77865_muic_show_usb_sel(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77865_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *pdata = muic_data->pdata;
	const char *mode = "UNKNOWN\n";

	switch (pdata->usb_path) {
	case MUIC_PATH_USB_AP:
		mode = "PDA\n";
		break;
	case MUIC_PATH_USB_CP:
		mode = "MODEM\n";
		break;
	default:
		break;
	}

	return snprintf(buf, PAGE_SIZE, "%s", mode);
}

static ssize_t max77865_muic_set_usb_sel(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max77865_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *pdata = muic_data->pdata;

	mutex_lock(&muic_data->muic_mutex);
	if (!strncasecmp(buf, "PDA", 3))
		pdata->usb_path = MUIC_PATH_USB_AP;
	else if (!strncasecmp(buf, "MODEM", 5))
		pdata->usb_path = MUIC_PATH_USB_CP;
	mutex_unlock(&muic_data->muic_mutex);

	return count;
}

static ssize_t max77865_muic_show_adc(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77865_muic_data *muic_data = dev_get_drvdata(dev);
	u8 uid = max77865_muic_get_uid_value(muic_data);

	return snprintf(buf, PAGE_SIZE, "%x\n", uid);
}

static ssize_t max77865_muic_show_usb_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77865_muic_data *muic_data = dev_get_drvdata(dev);

	switch (muic_data->attached_dev) {
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
		return snprintf(buf, PAGE_SIZE, "USB_STATE_CONFIGURED\n");
	default:
		return snprintf(buf, PAGE_SIZE, "USB_STATE_NOTCONFIGURED\n");
	}
}

static ssize_t max77865_muic_show_attached_dev(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77865_muic_data *muic_data = dev_get_drvdata(dev);
	const struct max77865_muic_vps_data *vps;

	if (muic_lookup_vps_table(muic_data->attached_dev, &vps) == 0)
		return snprintf(buf, PAGE_SIZE, "%s\n", vps->vps_name);

	return snprintf(buf, PAGE_SIZE, "No VPS\n");
}

#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
static ssize_t max77865_muic_show_afc_disable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77865_muic_data *muic_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", muic_data->pdata->afc_disable);
}

static ssize_t max77865_muic_set_afc_disable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max77865_muic_data *muic_data = dev_get_drvdata(dev);

	mutex_lock(&muic_data->muic_mutex);
	if (!strncasecmp(buf, "1", 1)) {
		muic_data->pdata->afc_disable = true;
		max77865_muic_afc_reset(muic_data);
	} else if (!strncasecmp(buf, "0", 1)) {
		muic_data->pdata->afc_disable = false;
	}
	mutex_unlock(&muic_data->muic_mutex);

	return count;
}
#endif /* CONFIG_HV_MUIC_MAX77865_AFC */

static ssize_t max77865_muic_show_vbus_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77865_muic_data *muic_data = dev_get_drvdata(dev);
	u8 vbadc = max77865_muic_get_vbadc_value(muic_data);

	return snprintf(buf, PAGE_SIZE, "%d\n", vbadc);
}

static DEVICE_ATTR(uart_sel, 0664, max77865_muic_show_uart_sel,
		max77865_muic_set_uart_sel);
static DEVICE_ATTR(usb_sel, 0664, max77865_muic_show_usb_sel,
		max77865_muic_set_usb_sel);
static DEVICE_ATTR(adc, 0444, max77865_muic_show_adc, NULL);
static DEVICE_ATTR(usb_state, 0444, max77865_muic_show_usb_state, NULL);
static DEVICE_ATTR(attached_dev, 0444, max77865_muic_show_attached_dev, NULL);
#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
static DEVICE_ATTR(afc_disable, 0664, max77865_muic_show_afc_disable,
		max77865_muic_set_afc_disable);
#endif
static DEVICE_ATTR(vbus_value, 0444, max77865_muic_show_vbus_value, NULL);

static struct attribute *max77865_muic_attributes[] = {
	&dev_attr_uart_sel.attr,
	&dev_attr_usb_sel.attr,
	&dev_attr_adc.attr,
	&dev_attr_usb_state.attr,
	&dev_attr_attached_dev.attr,
#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
	&dev_attr_afc_disable.attr,
#endif
	&dev_attr_vbus_value.attr,
	NULL
};

static const struct attribute_group max77865_muic_group = {
	.attrs = max77865_muic_attributes,
};

void max77865_muic_read_register(struct i2c_client *i2c)
{
	u8 bc1 = 0, bc2 = 0, gp = 0, ctrl1 = 0, hv1 = 0, hv2 = 0;

	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_STATUS1_BC, &bc1);
	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_STATUS2_BC, &bc2);
	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_STATUS_GP, &gp);
	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_CONTROL1, &ctrl1);
	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_HVCONTROL1, &hv1);
	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_HVCONTROL2, &hv2);

	pr_info("%s:%s BC1:0x%02x BC2:0x%02x GP:0x%02x CTRL1:0x%02x HV1:0x%02x HV2:0x%02x\n",
			MUIC_DEV_NAME, __func__, bc1, bc2, gp, ctrl1, hv1, hv2);
}

static int max77865_muic_attach_uart_path(struct max77865_muic_data *muic_data,
		muic_attached_dev_t new_dev)
{
	if (muic_data->pdata->uart_path == MUIC_PATH_UART_AP)
		return switch_to_ap_uart(muic_data, new_dev);
	else if (muic_data->pdata->uart_path == MUIC_PATH_UART_CP)
		return switch_to_cp_uart(muic_data, new_dev);
	return -EINVAL;
}

static int max77865_muic_attach_usb_path(struct max77865_muic_data *muic_data,
		muic_attached_dev_t new_dev)
{
	if (muic_data->pdata->usb_path == MUIC_PATH_USB_AP)
		return com_to_usb_ap(muic_data);
	else if (muic_data->pdata->usb_path == MUIC_PATH_USB_CP)
		return com_to_usb_cp(muic_data);
	return -EINVAL;
}

#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
void max77865_muic_disable_afc_protocol(struct max77865_muic_data *muic_data)
{
	max77865_muic_clear_hv_control(muic_data);
}
#endif

static int max77865_muic_handle_detach(struct max77865_muic_data *muic_data)
{
	muic_attached_dev_t prev_dev = muic_data->attached_dev;

	pr_info("%s:%s prev_dev(%d)\n", MUIC_DEV_NAME, __func__, prev_dev);

	switch (muic_data->attached_dev) {
	case ATTACHED_DEV_TA_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_TA_MUIC:
	case ATTACHED_DEV_AFC_CHARGER_5V_MUIC:
	case ATTACHED_DEV_AFC_CHARGER_9V_MUIC:
	case ATTACHED_DEV_QC_CHARGER_5V_MUIC:
	case ATTACHED_DEV_QC_CHARGER_9V_MUIC:
		if ((muic_data->bcstatus1 & BC_STATUS1_VBVOLT_MASK) > 0) {
			/* W/A for chgtype 0 irq when CC pin is only detached */
			pr_info("%s:%s Vbus is high, keep the current state(%d)\n",
					MUIC_DEV_NAME, __func__, muic_data->attached_dev);
			return 0;
		}
		break;
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_OTG_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_FG_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_VB_MUIC:
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
	case ATTACHED_DEV_NONE_MUIC:
		com_to_open(muic_data);
		break;
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_OTG_MUIC:
	case ATTACHED_DEV_TIMEOUT_OPEN_MUIC:
		if (muic_data->ccic_info_data.ccic_evt_attached == MUIC_CCIC_NOTI_DETACH)
			com_to_open(muic_data);
		break;
	case ATTACHED_DEV_UNOFFICIAL_ID_MUIC:
		goto out_without_noti;
	default:
		break;
	}

#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
	max77865_muic_disable_afc_protocol(muic_data);
#endif /* CONFIG_HV_MUIC_MAX77865_AFC */
#if defined(CONFIG_MUIC_NOTIFIER)
	if (prev_dev != ATTACHED_DEV_NONE_MUIC)
		muic_notifier_detach_attached_dev(prev_dev);
#endif /* CONFIG_MUIC_NOTIFIER */

out_without_noti:
	muic_data->attached_dev = ATTACHED_DEV_NONE_MUIC;

	return 0;
}

static int max77865_muic_handle_attach(struct max77865_muic_data *muic_data,
		muic_attached_dev_t new_dev)
{
	pr_info("%s:%s attached_dev(%d) -> new_dev(%d)\n",
			MUIC_DEV_NAME, __func__, muic_data->attached_dev, new_dev);

	switch (new_dev) {
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
	case ATTACHED_DEV_TIMEOUT_OPEN_MUIC:
		max77865_muic_attach_usb_path(muic_data, new_dev);
		break;
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
		max77865_muic_attach_uart_path(muic_data, new_dev);
		break;
	case ATTACHED_DEV_TA_MUIC:
	default:
		com_to_open(muic_data);
		break;
	}

#if defined(CONFIG_MUIC_NOTIFIER)
	muic_notifier_attach_attached_dev(new_dev);
#endif /* CONFIG_MUIC_NOTIFIER */

	muic_data->attached_dev = new_dev;

#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
	if (max77865_muic_check_is_enable_afc(muic_data, new_dev))
		max77865_muic_prepare_afc_charger(muic_data);
#endif /* CONFIG_HV_MUIC_MAX77865_AFC */

	return 0;
}

static void max77865_muic_detect_dev(struct max77865_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 bc1 = 0, bc2 = 0, gp = 0;
	u8 uid, chgtyp, prchgtyp;
	bool vbvolt;
	muic_attached_dev_t new_dev = ATTACHED_DEV_NONE_MUIC;
	int i;

	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_STATUS1_BC, &bc1);
	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_STATUS2_BC, &bc2);
	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_STATUS_GP, &gp);

	muic_data->bcstatus1 = bc1;
	muic_data->bcstatus2 = bc2;
	muic_data->gpstatus3 = gp;

	vbvolt = !!(bc1 & BC_STATUS1_VBVOLT_MASK);
	chgtyp = (bc1 & BC_STATUS1_CHGTYP_MASK) >> BC_STATUS1_CHGTYP_SHIFT;
	prchgtyp = (bc1 & BC_STATUS1_PRCHGTYP_MASK) >> BC_STATUS1_PRCHGTYP_SHIFT;
	uid = gp & GP_STATUS_UID_MASK;

	for (i = 0; i < ARRAY_SIZE(muic_vps_table); i++) {
		const struct max77865_muic_vps_data *vps = &muic_vps_table[i];

		if (vps->uid != UID_DONTCARE && vps->uid != uid)
			continue;
		if (vps->vbvolt != VB_DONTCARE && vps->vbvolt != (vbvolt ? VB_HIGH : VB_LOW))
			continue;
		if (vps->chgtyp != CHGTYP_DONTCARE && vps->chgtyp != chgtyp)
			continue;

		new_dev = vps->attached_dev;
		break;
	}

	if (new_dev == ATTACHED_DEV_NONE_MUIC && vbvolt) {
		if (prchgtyp >= PRCHGTYP_SAMSUNG_2A && prchgtyp <= PRCHGTYP_3A_DCP)
			new_dev = ATTACHED_DEV_TA_MUIC;
		else
			new_dev = ATTACHED_DEV_TIMEOUT_OPEN_MUIC;
	}

#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
	new_dev = max77865_hv_muic_check_id_err(muic_data, new_dev);
#endif

	max77865_muic_handle_vbus(muic_data);

	if (new_dev == muic_data->attached_dev)
		return;

	if (new_dev == ATTACHED_DEV_NONE_MUIC)
		max77865_muic_handle_detach(muic_data);
	else
		max77865_muic_handle_attach(muic_data, new_dev);
}

#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
static int max77865_muic_afc_set_voltage(int voltage)
{
	struct max77865_muic_data *muic_data = g_muic_data;

	if (!muic_data || !muic_data->is_afc_muic_ready)
		return -ENODEV;

	mutex_lock(&muic_data->muic_mutex);
	if (muic_data->attached_dev == ATTACHED_DEV_QC_CHARGER_5V_MUIC ||
	    muic_data->attached_dev == ATTACHED_DEV_QC_CHARGER_9V_MUIC)
		max77865_muic_qc_hv_set(muic_data, voltage);
	else
		max77865_muic_afc_hv_set(muic_data, voltage);
	mutex_unlock(&muic_data->muic_mutex);

	return 0;
}

static int max77865_muic_hv_charger_init(void)
{
	struct max77865_muic_data *muic_data = g_muic_data;

	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);

	if (!muic_data) {
		pr_err("%s:%s muic_data is NULL\n", MUIC_DEV_NAME, __func__);
		return -ENODEV;
	}

	mutex_lock(&muic_data->muic_mutex);
	muic_data->is_charger_ready = true;
	if (muic_data->attached_dev == ATTACHED_DEV_TA_MUIC)
		max77865_muic_prepare_afc_charger(muic_data);
	mutex_unlock(&muic_data->muic_mutex);

	return 0;
}
#endif /* CONFIG_HV_MUIC_MAX77865_AFC */

static irqreturn_t max77865_muic_irq_thread(int irq, void *data)
{
	struct max77865_muic_data *muic_data = data;
	struct i2c_client *i2c = muic_data->i2c;
	u8 int_main = 0, int_bc = 0, int_fc = 0, int_gp = 0;

	mutex_lock(&muic_data->muic_mutex);

	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_INT_MAIN, &int_main);
	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_INT_BC, &int_bc);
	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_INT_FC, &int_fc);
	max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_INT_GP, &int_gp);

	if ((int_bc & (INT_BC_CHGTYP_MASK | INT_BC_PRCHGTYP_MASK |
		       INT_BC_VBVOLT_MASK | INT_BC_DCDTMR_MASK)) ||
	    (int_gp & (INT_GP_UID_MASK | INT_GP_UIDGND_MASK)))
		max77865_muic_detect_dev(muic_data);

#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
	if (int_bc & INT_BC_DNVDATREF_MASK)
		max77865_muic_handle_vdnmon(muic_data);

	if (int_fc & INT_FC_MRXRDY_MASK)
		max77865_muic_handle_detect_dev_afc(muic_data);

	if (int_fc & INT_FC_MPNACK_MASK)
		max77865_muic_handle_detect_dev_mpnack(muic_data);

	if (int_gp & INT_GP_VBADC_MASK)
		max77865_muic_handle_vbadc(muic_data);
#endif /* CONFIG_HV_MUIC_MAX77865_AFC */

	mutex_unlock(&muic_data->muic_mutex);

	return IRQ_HANDLED;
}

static int max77865_muic_init_regs(struct max77865_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 dev_id = 0;
	int ret;

	ret = max77865_muic_read_reg(i2c, MAX77865_MUIC_REG_DEVICEID, &dev_id);
	if (ret < 0)
		return ret;

	pr_info("%s:%s DEVICEID=0x%02x\n", MUIC_DEV_NAME, __func__, dev_id);

	max77865_muic_write_reg(i2c, MAX77865_MUIC_REG_CCDET, CCDET_DISABLE);
#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
	max77865_muic_clear_hv_control(muic_data);
#endif
	max77865_muic_enable_chgdet(muic_data);

	max77865_muic_write_reg(i2c, MAX77865_MUIC_REG_INTMASK_MAIN, 0x00);
	max77865_muic_write_reg(i2c, MAX77865_MUIC_REG_INTMASK_BC, 0x00);
	max77865_muic_write_reg(i2c, MAX77865_MUIC_REG_INTMASK_GP, 0x00);

	return 0;
}

static void muic_register_usb_notifier(struct max77865_muic_data *pmuic)
{
	int ret = 0;

	pr_info("%s\n", __func__);


	ret = usb_external_notify_register(&pmuic->usb_nb,
		muic_handle_usb_notification, EXTERNAL_NOTIFY_DEV_MUIC);
	if (ret < 0) {
		pr_info("%s: USB Noti. is not ready.\n", __func__);
		return;
	}

	pr_info("%s: done.\n", __func__);
}

static void muic_unregister_usb_notifier(struct max77865_muic_data *pmuic)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = usb_external_notify_unregister(&pmuic->usb_nb);
	if (ret < 0) {
		pr_info("%s: USB Noti. unregister error.\n", __func__);
		return;
	}

	pr_info("%s: done.\n", __func__);
}
#else
static void muic_register_usb_notifier(struct max77865_muic_data *pmuic) {}
static void muic_unregister_usb_notifier(struct max77865_muic_data *pmuic) {}
#endif

static int max77865_muic_probe(struct platform_device *pdev)
{
	struct max77865_dev *max77865 = dev_get_drvdata(pdev->dev.parent);
	struct max77865_platform_data *mfd_pdata = dev_get_platdata(max77865->dev);
	struct max77865_muic_data *muic_data;
	int ret;

	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);

	muic_data = devm_kzalloc(&pdev->dev, sizeof(*muic_data), GFP_KERNEL);
	if (!muic_data)
		return -ENOMEM;

	if (!mfd_pdata) {
		pr_err("%s: failed to get mfd platform data\n", __func__);
		return -ENOMEM;
	}

	muic_data->dev = &pdev->dev;
	muic_data->i2c = max77865->muic;
	muic_data->mfd_pdata = mfd_pdata;
	muic_data->pdata = &muic_pdata;
	muic_data->attached_dev = ATTACHED_DEV_NONE_MUIC;

	mutex_init(&muic_data->muic_mutex);
	mutex_init(&muic_data->reset_mutex);

	platform_set_drvdata(pdev, muic_data);

#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
	if (get_afc_mode() == CH_MODE_AFC_DISABLE_VAL) {
		pr_info("%s:%s afc_disable: AFC disabled\n", MUIC_DEV_NAME, __func__);
		muic_data->pdata->afc_disable = true;
	} else {
		pr_info("%s:%s afc_disable: AFC enabled\n", MUIC_DEV_NAME, __func__);
		muic_data->pdata->afc_disable = false;
	}

	/* set MUIC afc voltage switching function */
	muic_data->pdata->muic_afc_set_voltage_cb = max77865_muic_afc_set_voltage;

	/* set MUIC check charger init function */
	muic_data->pdata->muic_hv_charger_init_cb = max77865_muic_hv_charger_init;
	muic_data->is_charger_ready = false;

	max77865_hv_muic_init_detect(muic_data);
#endif /* CONFIG_HV_MUIC_MAX77865_AFC */

	/* register usb external notifier */
	muic_register_usb_notifier(muic_data);

	/* create sysfs group */
	ret = sysfs_create_group(&switch_device->kobj, &max77865_muic_group);
	if (ret)
		pr_err("%s:%s failed to create sysfs group(%d)\n",
				MUIC_DEV_NAME, __func__, ret);
	dev_set_drvdata(switch_device, muic_data);

	ret = max77865_muic_init_regs(muic_data);
	if (ret < 0)
		return ret;

	{
		struct device_node *np = pdev->dev.parent ?
				pdev->dev.parent->of_node : pdev->dev.of_node;
		int irq_gpio = -1;
		int irq = 0;

		if (np) {
			irq_gpio = of_get_named_gpio(np, "muic-universal,irq-gpio", 0);
			if (!gpio_is_valid(irq_gpio))
				irq_gpio = of_get_named_gpio(np, "muic,irq-gpio", 0);
		}

		if (gpio_is_valid(irq_gpio)) {
			irq = gpio_to_irq(irq_gpio);
			pr_info("%s:%s irq_gpio(%d) irq(%d)\n",
					MUIC_DEV_NAME, __func__, irq_gpio, irq);
		} else if (max77865->irq > 0) {
			irq = max77865->irq;
		}

		if (irq > 0) {
			ret = devm_request_threaded_irq(&pdev->dev, irq,
					NULL, max77865_muic_irq_thread,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
					MUIC_DEV_NAME, muic_data);
			if (ret < 0) {
				pr_err("%s:%s failed to request irq(%d) ret(%d)\n",
						MUIC_DEV_NAME, __func__, irq, ret);
				return ret;
			}
			enable_irq_wake(irq);
		}
	}

	mutex_lock(&muic_data->muic_mutex);
	muic_data->is_muic_ready = true;
	max77865_muic_detect_dev(muic_data);
	mutex_unlock(&muic_data->muic_mutex);

	return 0;
}

static int max77865_muic_remove(struct platform_device *pdev)
{
	struct max77865_muic_data *muic_data = platform_get_drvdata(pdev);

	if (switch_device)
		sysfs_remove_group(&switch_device->kobj, &max77865_muic_group);

	max77865_muic_disable_chgdet(muic_data);
	muic_unregister_usb_notifier(muic_data);
	mutex_destroy(&muic_data->muic_mutex);
	mutex_destroy(&muic_data->reset_mutex);

	return 0;
}

static void max77865_muic_shutdown(struct platform_device *pdev)
{
	struct max77865_muic_data *muic_data = platform_get_drvdata(pdev);

	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);

	if (switch_device)
		sysfs_remove_group(&switch_device->kobj, &max77865_muic_group);

	if (!muic_data) {
		pr_err("%s no drvdata\n", __func__);
		goto out;
	}

	com_to_open(muic_data);

#if defined(CONFIG_HV_MUIC_MAX77865_AFC)
	max77865_muic_clear_hv_control(muic_data);

	/* clear MUIC afc voltage switching function */
	muic_data->pdata->muic_afc_set_voltage_cb = NULL;
	/* clear MUIC check charger init function */
	muic_data->pdata->muic_hv_charger_init_cb = NULL;
	max77865_hv_muic_remove(muic_data);
#endif
	muic_unregister_usb_notifier(muic_data);

out:
	pr_info("%s -\n", __func__);
}

static struct platform_driver max77865_muic_driver = {
	.driver = {
		.name = "max77865-muic",
		.owner = THIS_MODULE,
	},
	.probe = max77865_muic_probe,
	.remove = max77865_muic_remove,
	.shutdown = max77865_muic_shutdown,
};

static int __init max77865_muic_init(void)
{
	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);
	return platform_driver_register(&max77865_muic_driver);
}
module_init(max77865_muic_init);

static void __exit max77865_muic_exit(void)
{
	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);
	platform_driver_unregister(&max77865_muic_driver);
}
module_exit(max77865_muic_exit);

MODULE_DESCRIPTION("Maxim MAX77865 MUIC driver");
MODULE_LICENSE("GPL");