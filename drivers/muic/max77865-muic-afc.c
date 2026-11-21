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
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/of.h>

#include <linux/mfd/max77865.h>
#include <linux/mfd/max77865-private.h>
#include <linux/muic/muic.h>
#include <linux/muic/max77865-muic-hv-typedef.h>
#include <linux/muic/max77865-muic.h>

#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#if defined(CONFIG_USB_HW_PARAM)
#include <linux/usb_notify.h>
#endif

#if defined(CONFIG_SEC_ABC)
#include <linux/sti/abc_common.h>
#endif

#define RETRY_COUNT			3
#define HVRXBYTE_MAX			16
#define MPING_MISS_WA_TIME		2000

/* MAX77865 HV CONTROL1 register */
#define HVCONTROL1_VBUSADCEN_SHIFT	0
#define HVCONTROL1_VBUSADCEN_MASK	(0x1 << HVCONTROL1_VBUSADCEN_SHIFT)

/* MAX77865 HV CONTROL2 register */
#define HVCONTROL2_FCRXEN_SHIFT		7
#define HVCONTROL2_MPINGEN_SHIFT	6
#define HVCONTROL2_MTXBUSRES_SHIFT	5
#define HVCONTROL2_MTXEN_SHIFT		4
#define HVCONTROL2_MPING_SHIFT		3
#define HVCONTROL2_DP06EN_SHIFT		1
#define HVCONTROL2_HVDIGEN_SHIFT	0
#define HVCONTROL2_FCRXEN_MASK		(0x1 << HVCONTROL2_FCRXEN_SHIFT)
#define HVCONTROL2_MPINGEN_MASK		(0x1 << HVCONTROL2_MPINGEN_SHIFT)
#define HVCONTROL2_MTXBUSRES_MASK	(0x1 << HVCONTROL2_MTXBUSRES_SHIFT)
#define HVCONTROL2_MTXEN_MASK		(0x1 << HVCONTROL2_MTXEN_SHIFT)
#define HVCONTROL2_MPING_MASK		(0x1 << HVCONTROL2_MPING_SHIFT)
#define HVCONTROL2_DP06EN_MASK		(0x1 << HVCONTROL2_DP06EN_SHIFT)
#define HVCONTROL2_HVDIGEN_MASK		(0x1 << HVCONTROL2_HVDIGEN_SHIFT)

/* MAX77865 FC INTMASK values */
#define INTMASK_FC_RESET		0xFF
#define INTMASK_FC_INIT			0x03

/* MAX77865 VBADC levels */
#define MAX77865_VBADC_4V_5V		0x1
#define MAX77865_VBADC_5V_6V		0x2
#define MAX77865_VBADC_6V_7V		0x3
#define MAX77865_VBADC_7V_8V		0x4
#define MAX77865_VBADC_8V_9V		0x5
#define MAX77865_VBADC_9V_10V		0x6
#define MAX77865_VBADC_10V_11V		0x7
#define MAX77865_VBADC_11V_12V		0x8
#define MAX77865_VBADC_12V_13V		0x9

/* MAX77865 QC2.0 DPDN drive presets (BC_CONTROL2) */
#define MAX77865_QC_DPDN_5V		0x14 /* DPDNMan=1, DP=0.6V, DN=GND */
#define MAX77865_QC_DPDN_9V		0x1D /* DPDNMan=1, DP=3.3V, DN=0.6V */
#define MAX77865_QC_DPDN_12V		0x15 /* DPDNMan=1, DP=0.6V, DN=0.6V */

static struct max77865_muic_data *g_hv_muic_data;

bool max77865_muic_check_is_enable_afc(struct max77865_muic_data *muic_data,
		muic_attached_dev_t new_dev)
{
	bool ret = false;

	if (new_dev == ATTACHED_DEV_TA_MUIC) {
		if (muic_data->pdata->afc_disable) {
			pr_info("%s:%s AFC Disable(%d) by USER!, skip AFC\n",
				MUIC_DEV_NAME, __func__, muic_data->pdata->afc_disable);
		} else if (!muic_data->is_charger_ready) {
			pr_info("%s:%s Charger is not ready(%d), skip AFC\n",
				MUIC_DEV_NAME, __func__, muic_data->is_charger_ready);
		} else {
			ret = true;
		}
	}

	return ret;
}

muic_attached_dev_t max77865_hv_muic_check_id_err(struct max77865_muic_data *muic_data,
		muic_attached_dev_t new_dev)
{
	muic_attached_dev_t after_new_dev = new_dev;

	if (!muic_data->is_afc_muic_ready)
		return after_new_dev;

	switch (new_dev) {
	case ATTACHED_DEV_TA_MUIC:
		if (muic_data->attached_dev == ATTACHED_DEV_AFC_CHARGER_5V_MUIC ||
		    muic_data->attached_dev == ATTACHED_DEV_AFC_CHARGER_9V_MUIC ||
		    muic_data->attached_dev == ATTACHED_DEV_QC_CHARGER_5V_MUIC ||
		    muic_data->attached_dev == ATTACHED_DEV_QC_CHARGER_9V_MUIC)
			after_new_dev = muic_data->attached_dev;
		break;
	default:
		break;
	}

	return after_new_dev;
}

void max77865_muic_afc_reset(struct max77865_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;

	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);

	/* DPDNMan enable, DP GND, DM Open */
	max77865_write_reg(i2c, MAX77865_MUIC_REG_CONTROL2_BC, 0x13);
	msleep(60);
	max77865_write_reg(i2c, MAX77865_MUIC_REG_CONTROL2_BC, 0x00);
}

static void max77865_muic_afc_hv_tx_byte_set(struct max77865_muic_data *muic_data,
		u8 tx_byte)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 hvctrl2;

	pr_info("%s:%s tx_byte(0x%02x)\n", MUIC_DEV_NAME, __func__, tx_byte);

	muic_data->tx_data = tx_byte;
	max77865_write_reg(i2c, MAX77865_MUIC_REG_HVTXBYTE, tx_byte);

	max77865_update_reg(i2c, MAX77865_MUIC_REG_HVCONTROL1,
			HVCONTROL1_VBUSADCEN_MASK, HVCONTROL1_VBUSADCEN_MASK);
	max77865_write_reg(i2c, MAX77865_MUIC_REG_INTMASK_FC, INTMASK_FC_INIT);

	hvctrl2 = HVCONTROL2_FCRXEN_MASK | HVCONTROL2_MPINGEN_MASK |
		  HVCONTROL2_MTXEN_MASK | HVCONTROL2_MPING_MASK |
		  HVCONTROL2_DP06EN_MASK | HVCONTROL2_HVDIGEN_MASK;
	muic_data->hvcontrol2 = hvctrl2;
	max77865_write_reg(i2c, MAX77865_MUIC_REG_HVCONTROL2, hvctrl2);

	schedule_delayed_work(&muic_data->hv_muic_mping_miss_wa,
			msecs_to_jiffies(MPING_MISS_WA_TIME));
}

void max77865_muic_clear_hv_control(struct max77865_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;

	muic_data->hvcontrol1 = 0x00;
	muic_data->hvcontrol2 = 0x00;
	muic_data->afc_count = 0;
	muic_data->is_afc_handshaking = false;
	muic_data->is_afc_muic_prepare = false;
	muic_data->is_mrxrdy = false;
	muic_data->qc_hv = 0;

	max77865_write_reg(i2c, MAX77865_MUIC_REG_HVCONTROL1, 0x00);
	max77865_write_reg(i2c, MAX77865_MUIC_REG_HVCONTROL2, 0x00);
	max77865_write_reg(i2c, MAX77865_MUIC_REG_CONTROL2_BC, 0x00);
	max77865_write_reg(i2c, MAX77865_MUIC_REG_INTMASK_FC, INTMASK_FC_RESET);
}

void max77865_muic_afc_hv_set(struct max77865_muic_data *muic_data, int voltage)
{
	u8 tx_byte;

	switch (voltage) {
	case 5:
		tx_byte = 0x08;
		break;
	case 9:
		tx_byte = 0x46;
		break;
	default:
		pr_info("%s:%s invalid value(%d), return\n", MUIC_DEV_NAME,
				__func__, voltage);
		return;
	}

	pr_info("%s:%s voltage(%d)\n", MUIC_DEV_NAME, __func__, voltage);

	muic_data->is_afc_muic_prepare = true;
	muic_data->is_afc_handshaking = true;
	max77865_muic_afc_hv_tx_byte_set(muic_data, tx_byte);
}

void max77865_muic_qc_hv_set(struct max77865_muic_data *muic_data, int voltage)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 dpdndrv;

	switch (voltage) {
	case 5:
		dpdndrv = MAX77865_QC_DPDN_5V;
		break;
	case 9:
		dpdndrv = MAX77865_QC_DPDN_9V;
		break;
	default:
		pr_info("%s:%s invalid value(%d), return\n", MUIC_DEV_NAME,
				__func__, voltage);
		return;
	}

	pr_info("%s:%s voltage(%d)\n", MUIC_DEV_NAME, __func__, voltage);

	muic_data->qc_hv = voltage;
	muic_data->is_qc_vb_settle = false;

	max77865_update_reg(i2c, MAX77865_MUIC_REG_HVCONTROL1,
			HVCONTROL1_VBUSADCEN_MASK, HVCONTROL1_VBUSADCEN_MASK);
	max77865_write_reg(i2c, MAX77865_MUIC_REG_CONTROL2_BC, dpdndrv);

	schedule_delayed_work(&muic_data->hv_muic_qc_vb_work,
			msecs_to_jiffies(300));
}

void max77865_muic_prepare_afc_charger(struct max77865_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;

	pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);

	muic_data->is_afc_muic_prepare = true;
	muic_data->afc_count = 0;

	max77865_update_reg(i2c, MAX77865_MUIC_REG_HVCONTROL1,
			HVCONTROL1_VBUSADCEN_MASK, HVCONTROL1_VBUSADCEN_MASK);
	max77865_update_reg(i2c, MAX77865_MUIC_REG_CONTROL2_BC,
			BC_CONTROL2_DNMONEN_MASK, BC_CONTROL2_DNMONEN_MASK);

#if defined(CONFIG_MUIC_NOTIFIER)
	muic_notifier_attach_attached_dev(ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC);
#endif
}

static u8 max77865_muic_get_vbadc(struct max77865_muic_data *muic_data)
{
	u8 gp_status = 0;

	max77865_read_reg(muic_data->i2c, MAX77865_MUIC_REG_STATUS_GP, &gp_status);
	muic_data->gpstatus3 = gp_status;

	return (gp_status & GP_STATUS_VBADC_MASK) >> GP_STATUS_VBADC_SHIFT;
}

void max77865_muic_handle_detect_dev_mpnack(struct max77865_muic_data *muic_data)
{
	cancel_delayed_work(&muic_data->hv_muic_mping_miss_wa);

	muic_data->afc_count++;
	pr_info("%s:%s MPing NACK(%d)\n", MUIC_DEV_NAME, __func__, muic_data->afc_count);

	if (muic_data->afc_count < RETRY_COUNT) {
		max77865_muic_afc_hv_tx_byte_set(muic_data, muic_data->tx_data);
	} else {
		muic_data->is_afc_handshaking = false;
		max77865_muic_qc_hv_set(muic_data, muic_data->qc_hv ? muic_data->qc_hv : 9);
	}
}

void max77865_muic_handle_detect_dev_afc(struct max77865_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 rx_data[HVRXBYTE_MAX] = { 0 };
	int vbadc;
	muic_attached_dev_t new_afc_dev = muic_data->attached_dev;
#if defined(CONFIG_USB_HW_PARAM)
	struct otg_notify *o_notify = get_otg_notify();
	bool afc_err = false;
#endif
	int i;

	cancel_delayed_work(&muic_data->hv_muic_mping_miss_wa);
	muic_data->is_mrxrdy = true;

	for (i = 0; i < HVRXBYTE_MAX; i++) {
		max77865_read_reg(i2c, MAX77865_MUIC_REG_HVRXBYTE1 + i, &rx_data[i]);
		if (rx_data[i] == 0x00)
			break;
	}

	vbadc = max77865_muic_get_vbadc(muic_data);

	pr_info("%s:%s AFC Success, vbadc(0x%x) rxbyte:0x%x %x %x %x\n",
			MUIC_DEV_NAME, __func__, vbadc,
			rx_data[0], rx_data[1], rx_data[2], rx_data[3]);

	muic_data->afc_count = 0;
	muic_data->is_afc_handshaking = false;

	if (vbadc >= MAX77865_VBADC_4V_5V && vbadc <= MAX77865_VBADC_6V_7V)
		new_afc_dev = ATTACHED_DEV_AFC_CHARGER_5V_MUIC;
	else if (vbadc >= MAX77865_VBADC_7V_8V && vbadc <= MAX77865_VBADC_10V_11V)
		new_afc_dev = ATTACHED_DEV_AFC_CHARGER_9V_MUIC;
#if defined(CONFIG_USB_HW_PARAM)
	else
		afc_err = true;
#endif

	if (new_afc_dev != muic_data->attached_dev) {
#if defined(CONFIG_MUIC_NOTIFIER)
		muic_notifier_attach_attached_dev(new_afc_dev);
#endif /* CONFIG_MUIC_NOTIFIER */
		muic_data->attached_dev = new_afc_dev;
	}

#if defined(CONFIG_USB_HW_PARAM)
	if (o_notify && afc_err)
		inc_hw_param(o_notify, USB_MUIC_AFC_ERROR_COUNT);
#endif
}

void max77865_muic_handle_detect_dev_qc(struct max77865_muic_data *muic_data)
{
	int vbadc = max77865_muic_get_vbadc(muic_data);
	muic_attached_dev_t new_afc_dev = muic_data->attached_dev;
#if defined(CONFIG_USB_HW_PARAM)
	struct otg_notify *o_notify = get_otg_notify();
	bool afc_err = false;
#endif

	pr_info("%s:%s QC2.0 check, vbadc(0x%x)\n", MUIC_DEV_NAME, __func__, vbadc);

	muic_data->is_qc_vb_settle = true;

	if (vbadc >= MAX77865_VBADC_4V_5V && vbadc <= MAX77865_VBADC_6V_7V)
		new_afc_dev = ATTACHED_DEV_QC_CHARGER_5V_MUIC;
	else if (vbadc >= MAX77865_VBADC_7V_8V && vbadc <= MAX77865_VBADC_10V_11V)
		new_afc_dev = ATTACHED_DEV_QC_CHARGER_9V_MUIC;
#if defined(CONFIG_USB_HW_PARAM)
	else
		afc_err = true;
#endif

	if (new_afc_dev != muic_data->attached_dev) {
#if defined(CONFIG_MUIC_NOTIFIER)
		muic_notifier_attach_attached_dev(new_afc_dev);
#endif /* CONFIG_MUIC_NOTIFIER */
		muic_data->attached_dev = new_afc_dev;
	}

#if defined(CONFIG_USB_HW_PARAM)
	if (o_notify && afc_err)
		inc_hw_param(o_notify, USB_MUIC_AFC_ERROR_COUNT);
#endif
}

void max77865_muic_handle_vdnmon(struct max77865_muic_data *muic_data)
{
	u8 bc_status2 = 0;

	max77865_read_reg(muic_data->i2c, MAX77865_MUIC_REG_STATUS2_BC, &bc_status2);
	muic_data->bcstatus2 = bc_status2;

	pr_info("%s:%s bcstatus2(0x%02x)\n", MUIC_DEV_NAME, __func__, bc_status2);

	if (!(bc_status2 & BC_STATUS2_DNVDATREF_MASK) && muic_data->is_afc_muic_prepare) {
		muic_data->is_afc_handshaking = true;
		max77865_muic_afc_hv_tx_byte_set(muic_data,
				muic_data->tx_data ? muic_data->tx_data : 0x46);
	}
}

void max77865_muic_handle_vbadc(struct max77865_muic_data *muic_data)
{
	if (muic_data->is_afc_handshaking)
		max77865_muic_handle_detect_dev_afc(muic_data);
	else if (muic_data->qc_hv)
		max77865_muic_handle_detect_dev_qc(muic_data);
}

void max77865_hv_muic_qc_vb_work(struct work_struct *work)
{
	struct max77865_muic_data *muic_data = container_of(work,
			struct max77865_muic_data, hv_muic_qc_vb_work.work);

	mutex_lock(&muic_data->muic_mutex);
	if (!muic_data->is_qc_vb_settle)
		max77865_muic_handle_detect_dev_qc(muic_data);
	mutex_unlock(&muic_data->muic_mutex);
}

void max77865_hv_muic_mping_miss_wa(struct work_struct *work)
{
	struct max77865_muic_data *muic_data = container_of(work,
			struct max77865_muic_data, hv_muic_mping_miss_wa.work);

	mutex_lock(&muic_data->muic_mutex);
	if (muic_data->is_afc_handshaking && !muic_data->is_mrxrdy) {
		pr_info("%s:%s MPing miss timeout\n", MUIC_DEV_NAME, __func__);
		max77865_muic_handle_detect_dev_mpnack(muic_data);
	}
	mutex_unlock(&muic_data->muic_mutex);
}

#if defined(CONFIG_OF)
static int of_max77865_hv_muic_dt(struct max77865_muic_data *muic_data)
{
	struct device_node *np = muic_data->dev->parent ?
			muic_data->dev->parent->of_node : muic_data->dev->of_node;
	int ret;

	/* Defaults if properties are absent */
	muic_data->tx_data = 0x46;
	muic_data->qc_hv = 9;

	if (!np)
		return -EINVAL;

	ret = of_property_read_u8(np, "muic,afcmode-tx", &muic_data->tx_data);
	if (ret)
		pr_info("%s:%s muic,afcmode-tx not set, using 0x%02x\n",
				MUIC_DEV_NAME, __func__, muic_data->tx_data);

	ret = of_property_read_u8(np, "muic,qc-hv", &muic_data->qc_hv);
	if (ret)
		pr_info("%s:%s muic,qc-hv not set, using %dV\n",
				MUIC_DEV_NAME, __func__, muic_data->qc_hv);

	pr_info("%s:%s afcmode-tx=0x%02x qc-hv=%dV\n",
			MUIC_DEV_NAME, __func__, muic_data->tx_data, muic_data->qc_hv);

	return 0;
}
#endif /* CONFIG_OF */

void max77865_hv_muic_init_detect(struct max77865_muic_data *muic_data)
{
	g_hv_muic_data = muic_data;
	muic_data->is_afc_muic_ready = true;

#if defined(CONFIG_OF)
	of_max77865_hv_muic_dt(muic_data);
#endif

	INIT_DELAYED_WORK(&muic_data->hv_muic_qc_vb_work,
			max77865_hv_muic_qc_vb_work);
	INIT_DELAYED_WORK(&muic_data->hv_muic_mping_miss_wa,
			max77865_hv_muic_mping_miss_wa);
}

void max77865_hv_muic_remove(struct max77865_muic_data *muic_data)
{
	muic_data->is_afc_muic_ready = false;
	cancel_delayed_work_sync(&muic_data->hv_muic_qc_vb_work);
	cancel_delayed_work_sync(&muic_data->hv_muic_mping_miss_wa);
	max77865_muic_clear_hv_control(muic_data);
	g_hv_muic_data = NULL;
}