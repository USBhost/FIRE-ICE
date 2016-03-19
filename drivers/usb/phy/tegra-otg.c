/*
 * drivers/usb/otg/tegra-otg.c
 *
 * OTG transceiver driver for Tegra UTMI phy
 *
 * Copyright (C) 2010-2014 NVIDIA CORPORATION. All rights reserved.
 * Copyright (C) 2010 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/pm_runtime.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/hcd.h>
#include <linux/tegra_pm_domains.h>

#include <mach/tegra_usb_pad_ctrl.h>

#define USB_PHY_WAKEUP		0x408
#define  USB_ID_INT_EN		(1 << 0)
#define  USB_ID_INT_STATUS	(1 << 1)
#define  USB_ID_STATUS		(1 << 2)
#define  USB_ID_PIN_WAKEUP_EN	(1 << 6)
#define  USB_VBUS_WAKEUP_EN	(1 << 30)
#define  USB_VBUS_INT_EN	(1 << 8)
#define  USB_VBUS_INT_STATUS	(1 << 9)
#define  USB_VBUS_STATUS	(1 << 10)
#define  USB_INT_EN		(USB_VBUS_INT_EN | USB_ID_INT_EN | \
						USB_VBUS_WAKEUP_EN | USB_ID_PIN_WAKEUP_EN)
#define USB_VBUS_INT_STS_MASK	(0x7 << 8)
#define USB_ID_INT_STS_MASK	(0x7 << 0)

#ifdef OTG_DEBUG
#define DBG(stuff...)	pr_info("tegra-otg: " stuff)
#else
#define DBG(stuff...)	do {} while (0)
#endif

#define YCABLE_CHARGING_CURRENT_UA 1200000u

struct tegra_otg {
	struct platform_device *pdev;
	struct tegra_usb_otg_data *pdata;
	struct usb_phy phy;
	unsigned long int_status;
	unsigned long int_mask;
	spinlock_t lock;
	struct mutex irq_work_mutex;
	void __iomem *regs;
	struct clk *clk;
	int irq;
	struct work_struct work;
	struct regulator *vbus_reg;
	struct regulator *vbus_bat_reg;
	unsigned int intr_reg_data;
	bool support_y_cable;
	bool y_cable_conn;
	bool turn_off_vbus_on_lp0;
	bool clk_enabled;
	bool interrupt_mode;
	bool suspended;
	bool support_pmu_vbus;
	bool support_usb_id;
	bool support_pmu_id;
	bool support_gpio_id;
	int id_det_gpio;
	struct extcon_dev *id_extcon_dev;
	struct extcon_dev *vbus_extcon_dev;
	struct extcon_dev *edev;
};

static struct tegra_otg *tegra_clone;
static struct notifier_block otg_vbus_nb;
static struct notifier_block otg_id_nb;
struct extcon_specific_cable_nb *extcondev;

enum tegra_connect_type {
	CONNECT_TYPE_Y_CABLE
};

static char *const tegra_otg_extcon_cable[] = {
	[CONNECT_TYPE_Y_CABLE] = "Y-cable",
	NULL,
};

static void tegra_otg_set_extcon_state(struct tegra_otg *tegra)
{
	const char **cables;
	struct extcon_dev *edev;

	if (tegra->edev == NULL || tegra->edev->supported_cable == NULL)
		return;

	edev = tegra->edev;
	cables = tegra->edev->supported_cable;

	if (tegra->y_cable_conn)
		extcon_set_cable_state(edev, cables[CONNECT_TYPE_Y_CABLE],
				true);
	else
		extcon_set_cable_state(edev,
				cables[CONNECT_TYPE_Y_CABLE], false);
}

static int otg_notifications(struct notifier_block *nb,
				   unsigned long event, void *unused)
{
	struct tegra_otg *tegra = tegra_clone;
	unsigned long flags;
	DBG("%s(%d) Begin\n", __func__, __LINE__);

	spin_lock_irqsave(&tegra->lock, flags);

	if (tegra->support_pmu_vbus) {
		if (extcon_get_cable_state(tegra->vbus_extcon_dev, "USB"))
			tegra->int_status |= USB_VBUS_STATUS ;
		else
			tegra->int_status &= ~USB_VBUS_STATUS;
	}

	if (tegra->support_pmu_id) {
		if (extcon_get_cable_state(tegra->id_extcon_dev, "USB-Host")) {
			tegra->int_status &= ~USB_ID_STATUS;
			tegra->int_status |= USB_ID_INT_EN;
		 } else
			tegra->int_status |= USB_ID_STATUS;
	}

	spin_unlock_irqrestore(&tegra->lock, flags);
	DBG("%s(%d) tegra->int_status = 0x%lx\n", __func__,
				__LINE__, tegra->int_status);

	if (!tegra->suspended)
		schedule_work(&tegra->work);

	DBG("%s(%d) End\n", __func__, __LINE__);
	return NOTIFY_DONE;
}

void check_host_cable_connection(struct tegra_otg *tegra)
{
	unsigned long flags;
	bool id_present;
	DBG("%s(%d) Begin\n", __func__, __LINE__);

	id_present = (gpio_get_value_cansleep(tegra->id_det_gpio) == 0);

	spin_lock_irqsave(&tegra->lock, flags);
	if (id_present) {
		DBG("%s(%d) id connect\n", __func__, __LINE__);
		tegra->int_status &= ~USB_ID_STATUS;
		tegra->int_status |= USB_ID_INT_EN;
	} else {
		DBG("%s(%d) id disconnect\n", __func__, __LINE__);
		tegra->int_status |= USB_ID_STATUS;
	}
	spin_unlock_irqrestore(&tegra->lock, flags);
	DBG("%s(%d) tegra->int_status = 0x%lx\n", __func__,
				__LINE__, tegra->int_status);

	mutex_lock(&tegra->irq_work_mutex);
	if (!tegra->suspended)
		schedule_work(&tegra->work);
	mutex_unlock(&tegra->irq_work_mutex);
}

static irqreturn_t tegra_otg_id_detect_gpio_thr(int irq, void *data)
{
	struct tegra_otg *tegra = data;
	check_host_cable_connection(tegra);
	return IRQ_HANDLED;
}

static inline unsigned long otg_readl(struct tegra_otg *tegra,
				      unsigned int offset)
{
	return readl(tegra->regs + offset);
}

static inline void otg_writel(struct tegra_otg *tegra, unsigned long val,
			      unsigned int offset)
{
	writel(val, tegra->regs + offset);
}

static const char *tegra_state_name(enum usb_otg_state state)
{
	switch (state) {
		case OTG_STATE_A_HOST:
			return "HOST";
		case OTG_STATE_B_PERIPHERAL:
			return "PERIPHERAL";
		case OTG_STATE_A_SUSPEND:
			return "SUSPEND";
		case OTG_STATE_UNDEFINED:
			return "UNDEFINED";
		default:
			return "INVALID";
	}
}

static unsigned long enable_interrupt(struct tegra_otg *tegra, bool en)
{
	unsigned long val;

	pm_runtime_get_sync(tegra->phy.dev);
	clk_prepare_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	if (en) {
		/* Enable ID interrupt if detection is through USB controller */
		if (tegra->support_usb_id) {
			val |= USB_ID_INT_EN | USB_ID_PIN_WAKEUP_EN;
			tegra->int_mask |= USB_ID_INT_STS_MASK;
		}

		/* Enable vbus interrupt if cable is not detected through PMU */
		if (!tegra->support_pmu_vbus) {
			val |= USB_VBUS_INT_EN | USB_VBUS_WAKEUP_EN;
			tegra->int_mask |= USB_VBUS_INT_STS_MASK;
		}
	}
	else
		val &= ~USB_INT_EN;

	otg_writel(tegra, val, USB_PHY_WAKEUP);
	/* Add delay to make sure register is updated */
	udelay(1);
	clk_disable_unprepare(tegra->clk);
	pm_runtime_mark_last_busy(tegra->phy.dev);
	pm_runtime_put_autosuspend(tegra->phy.dev);

	DBG("%s(%d) interrupt mask = 0x%lx\n", __func__, __LINE__,
							tegra->int_mask);
	return val;
}

static void tegra_otg_set_current(struct regulator *vbus_bat_reg, int max_uA)
{
	if (vbus_bat_reg == NULL)
		return ;

	regulator_set_current_limit(vbus_bat_reg, 0, max_uA);
}

static void tegra_otg_vbus_enable(struct regulator *vbus_reg, int on)
{
	static int vbus_enable = 1;

	if (vbus_reg == NULL)
		return ;

	if (on && vbus_enable && !regulator_enable(vbus_reg))
		vbus_enable = 0;
	else if (!on && !vbus_enable && !regulator_disable(vbus_reg))
		vbus_enable = 1;
}

static void tegra_start_host(struct tegra_otg *tegra)
{
	struct tegra_usb_otg_data *pdata = tegra->pdata;
	struct platform_device *pdev, *ehci_device = pdata->ehci_device;
	int val;

	DBG("%s(%d) Begin\n", __func__, __LINE__);

	if (tegra->pdev)
		return;

	if (pdata->is_xhci) {
		tegra_xhci_release_otg_port(false);
		return;
	}

	/* prepare device structure for registering host*/
	pdev = platform_device_alloc(ehci_device->name, ehci_device->id);
	if (!pdev)
		return ;

	val = platform_device_add_resources(pdev, ehci_device->resource,
			ehci_device->num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask = ehci_device->dev.dma_mask;
	pdev->dev.coherent_dma_mask = ehci_device->dev.coherent_dma_mask;

	val = platform_device_add_data(pdev, pdata->ehci_pdata,
			sizeof(struct tegra_usb_platform_data));
	if (val)
		goto error;

	val = platform_device_add(pdev);
	if (val) {
		pr_err("%s: platform_device_add failed\n", __func__);
		goto error;
	}

	tegra->pdev = pdev;

	DBG("%s(%d) End\n", __func__, __LINE__);
	return;

error:
	BUG_ON("failed to add the host controller device\n");
	platform_device_del(pdev);
	tegra->pdev = NULL;
}

static void tegra_stop_host(struct tegra_otg *tegra)
{
	struct platform_device *pdev = tegra->pdev;

	DBG("%s(%d) Begin\n", __func__, __LINE__);

	if (tegra->pdata->is_xhci) {
		tegra_xhci_release_otg_port(true);
		return;
	}
	if (pdev) {
		/* unregister host from otg */
		platform_device_unregister(pdev);
		tegra->pdev = NULL;
	}

	DBG("%s(%d) End\n", __func__, __LINE__);
}

static void tegra_otg_notify_event(struct tegra_otg *tegra, int event)
{
	tegra->phy.last_event = event;
	atomic_notifier_call_chain(&tegra->phy.notifier, event, tegra->phy.otg->gadget);
}

static int tegra_otg_start_host(struct tegra_otg *tegra, int on)
{
	if (on) {
		if (tegra->support_y_cable &&
				(tegra->int_status & USB_VBUS_STATUS)) {
			DBG("%s(%d) set current %dmA\n", __func__, __LINE__,
					YCABLE_CHARGING_CURRENT_UA/1000);
			tegra_otg_set_current(tegra->vbus_bat_reg,
					YCABLE_CHARGING_CURRENT_UA);
			tegra->y_cable_conn = true;
			tegra_otg_set_extcon_state(tegra);
		} else {
			tegra_otg_vbus_enable(tegra->vbus_reg, 1);
		}
		tegra_start_host(tegra);
		tegra_otg_notify_event(tegra, USB_EVENT_ID);
	} else {
		tegra_stop_host(tegra);
		tegra_otg_notify_event(tegra, USB_EVENT_NONE);
		tegra_otg_vbus_enable(tegra->vbus_reg, 0);
		if (tegra->support_y_cable && tegra->y_cable_conn) {
			tegra_otg_set_current(tegra->vbus_bat_reg, 0);
			tegra->y_cable_conn = false;
			tegra_otg_set_extcon_state(tegra);
		}
	}
	return 0;
}

static int tegra_otg_start_gadget(struct tegra_otg *tegra, int on)
{
	struct usb_otg *otg = tegra->phy.otg;

	if (on)
		usb_gadget_vbus_connect(otg->gadget);
	else
		usb_gadget_vbus_disconnect(otg->gadget);

	return 0;
}

static void tegra_change_otg_state(struct tegra_otg *tegra,
				enum usb_otg_state to)
{
	struct usb_otg *otg = tegra->phy.otg;
	enum usb_otg_state from = otg->phy->state;

	if (!tegra->interrupt_mode){
		DBG("OTG: Vbus detection is disabled");
		return;
	}

	DBG("%s(%d) requested otg state %s-->%s\n", __func__,
		__LINE__, tegra_state_name(from), tegra_state_name(to));

	if (to != OTG_STATE_UNDEFINED && from != to) {
		otg->phy->state = to;
		pr_info("otg state changed: %s --> %s\n", tegra_state_name(from), tegra_state_name(to));

		if (from == OTG_STATE_A_SUSPEND) {
			if (to == OTG_STATE_B_PERIPHERAL && otg->gadget)
				tegra_otg_start_gadget(tegra, 1);
			else if (to == OTG_STATE_A_HOST)
				tegra_otg_start_host(tegra, 1);
		} else if (from == OTG_STATE_A_HOST && to == OTG_STATE_A_SUSPEND) {
			tegra_otg_start_host(tegra, 0);
		} else if (from == OTG_STATE_B_PERIPHERAL && otg->gadget && to == OTG_STATE_A_SUSPEND) {
			tegra_otg_start_gadget(tegra, 0);
		}
	}

	if (tegra->support_y_cable && tegra->y_cable_conn &&
			from == to && from == OTG_STATE_A_HOST) {
		if (!(tegra->int_status & USB_VBUS_STATUS)) {
			DBG("%s(%d) Charger disconnect\n", __func__, __LINE__);
			tegra_otg_set_current(tegra->vbus_bat_reg, 0);
			tegra_otg_vbus_enable(tegra->vbus_reg, 1);
			tegra->y_cable_conn = false;
			tegra_otg_set_extcon_state(tegra);
		}
	}
}

static void irq_work(struct work_struct *work)
{
	struct tegra_otg *tegra =
		container_of(work, struct tegra_otg, work);
	struct usb_otg *otg = tegra->phy.otg;
	enum usb_otg_state from;
	enum usb_otg_state to = OTG_STATE_UNDEFINED;
	unsigned long flags;
	unsigned long status;

	/* Adding delay for proper detection of y-cable */
	msleep(150);

	mutex_lock(&tegra->irq_work_mutex);

	spin_lock_irqsave(&tegra->lock, flags);
	from = otg->phy->state;
	status = tegra->int_status;

	/* Debug prints */
	DBG("%s(%d) status = 0x%lx\n", __func__, __LINE__, status);
	if ((status & USB_ID_INT_STATUS) &&
			(status & USB_VBUS_INT_STATUS))
		DBG("%s(%d) got vbus & id interrupt\n", __func__, __LINE__);
	else {
		if (status & USB_ID_INT_STATUS)
			DBG("%s(%d) got id interrupt\n", __func__, __LINE__);
		if (status & USB_VBUS_INT_STATUS)
			DBG("%s(%d) got vbus interrupt\n", __func__, __LINE__);
	}

	if (!(status & USB_ID_STATUS) && (status & USB_ID_INT_EN))
		to = OTG_STATE_A_HOST;
	else if (status & USB_VBUS_STATUS && from != OTG_STATE_A_HOST)
		to = OTG_STATE_B_PERIPHERAL;
	else
		to = OTG_STATE_A_SUSPEND;

	spin_unlock_irqrestore(&tegra->lock, flags);
	tegra_change_otg_state(tegra, to);
	mutex_unlock(&tegra->irq_work_mutex);
}

static irqreturn_t tegra_otg_irq(int irq, void *data)
{
	struct tegra_otg *tegra = data;
	unsigned long flags;
	unsigned long val;
	struct usb_hcd *hcd = bus_to_hcd(tegra->phy.otg->host);
	enum usb_otg_state state = tegra->phy.state;

	spin_lock_irqsave(&tegra->lock, flags);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	DBG("%s(%d) interrupt val = 0x%lx\n", __func__, __LINE__, val);

	if (val & (USB_VBUS_INT_EN | USB_ID_INT_EN)) {
		DBG("%s(%d) PHY_WAKEUP = 0x%lx\n", __func__, __LINE__, val);
		otg_writel(tegra, val, USB_PHY_WAKEUP);
		if ((val & USB_ID_INT_STATUS) || (val & USB_VBUS_INT_STATUS)) {
			tegra->int_status &= ~tegra->int_mask;
			tegra->int_status |= val & tegra->int_mask;
			schedule_work(&tegra->work);
		}
	}

	/* Re-acquire wakelock to service the device connected */
	/* Abort the suspend */

	if (state == OTG_STATE_A_HOST) {
		if (hcd && hcd->state == HC_STATE_SUSPENDED)
			tegra_otg_notify_event(tegra, USB_EVENT_ID);
	}
	spin_unlock_irqrestore(&tegra->lock, flags);

	return IRQ_HANDLED;
}


static int tegra_otg_set_peripheral(struct usb_otg *otg,
				struct usb_gadget *gadget)
{
	struct tegra_otg *tegra;
	unsigned long val;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	tegra = (struct tegra_otg *)container_of(otg->phy, struct tegra_otg, phy);
	otg->gadget = gadget;

	val = enable_interrupt(tegra, true);
	tegra->suspended = false;

	if (((val & USB_ID_STATUS) || tegra->support_pmu_id) &&
		 (val & USB_VBUS_STATUS) && !tegra->support_pmu_vbus)
		val |= USB_VBUS_INT_STATUS;
	else if (!(val & USB_ID_STATUS)) {
		if (tegra->support_usb_id)
			val |= USB_ID_INT_STATUS;
		else
			val &= ~USB_ID_INT_STATUS;
	}
	else
		val &= ~(USB_ID_INT_STATUS | USB_VBUS_INT_STATUS);

	if ((val & USB_ID_INT_STATUS) || (val & USB_VBUS_INT_STATUS)) {
		tegra->int_status = val;
		schedule_work(&tegra->work);
	}

	if (tegra->support_pmu_vbus || tegra->support_pmu_id)
		otg_notifications(NULL, 0, NULL);

	if (tegra->support_gpio_id && gpio_is_valid(tegra->id_det_gpio))
		check_host_cable_connection(tegra);

	DBG("%s(%d) END\n", __func__, __LINE__);
	return 0;
}

static int tegra_otg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct tegra_otg *tegra = container_of(otg->phy, struct tegra_otg, phy);
	unsigned long val;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	otg->host = host;

	pm_runtime_get_sync(tegra->phy.dev);
	clk_prepare_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	val &= ~(USB_VBUS_INT_STATUS | USB_ID_INT_STATUS);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	if (tegra->support_usb_id) {
		val = otg_readl(tegra, USB_PHY_WAKEUP);
		val |= (USB_ID_INT_EN | USB_ID_PIN_WAKEUP_EN);
		otg_writel(tegra, val, USB_PHY_WAKEUP);
	}
	clk_disable_unprepare(tegra->clk);
	pm_runtime_mark_last_busy(tegra->phy.dev);
	pm_runtime_put_autosuspend(tegra->phy.dev);

	DBG("%s(%d) END\n", __func__, __LINE__);
	return 0;
}

static ssize_t show_host_en(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg *tegra = platform_get_drvdata(pdev);
	struct usb_otg *otg = tegra->phy.otg;

	*buf = ((!tegra->interrupt_mode) &&
			(otg->phy->state == OTG_STATE_A_HOST)) ? '1' : '0';

	strcat(buf, "\n");
	return strlen(buf);
}

static ssize_t store_host_en(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg *tegra = platform_get_drvdata(pdev);
	int host;

	if (sscanf(buf, "%d", &host) != 1 || host < 0 || host > 1)
		return -EINVAL;

	if (host) {
		enable_interrupt(tegra, false);
		tegra->interrupt_mode = true;
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);
		tegra_change_otg_state(tegra, OTG_STATE_A_HOST);
		tegra->interrupt_mode = false;
	} else {
		tegra->interrupt_mode = true;
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);
		enable_interrupt(tegra, true);
	}

	return count;
}

static DEVICE_ATTR(enable_host, 0644, show_host_en, store_host_en);

static ssize_t show_device_en(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg *tegra = platform_get_drvdata(pdev);
	struct usb_otg *otg = tegra->phy.otg;

	*buf = ((!tegra->interrupt_mode) &&
			otg->phy->state == OTG_STATE_B_PERIPHERAL) ? '1' : '0';

	strcat(buf, "\n");
	return strlen(buf);
}

static ssize_t store_device_en(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg *tegra = platform_get_drvdata(pdev);
	unsigned long device;

	if (sscanf(buf, "%lu", &device) != 1 || device < 0 || device > 1)
		return -EINVAL;

	if (device) {
		enable_interrupt(tegra, false);
		tegra->interrupt_mode = true;
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);
		tegra_change_otg_state(tegra, OTG_STATE_B_PERIPHERAL);
		tegra->interrupt_mode = false;
	} else {
		tegra->interrupt_mode = true;
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);
		enable_interrupt(tegra, true);
	}

	return count;
}

static DEVICE_ATTR(enable_device, 0644, show_device_en, store_device_en);

static int tegra_otg_set_power(struct usb_phy *phy, unsigned mA)
{
	return 0;
}

static int tegra_otg_set_suspend(struct usb_phy *phy, int suspend)
{
	return 0;
}

void tegra_otg_set_id_detection_type(struct tegra_otg *tegra)
{
	switch (tegra->pdata->ehci_pdata->id_det_type) {
	case TEGRA_USB_ID:
		tegra->support_usb_id = true;
		break;
	case TEGRA_USB_PMU_ID:
		tegra->support_pmu_id = true;
		break;
	case TEGRA_USB_GPIO_ID:
		tegra->support_gpio_id = true;
		break;
	case TEGRA_USB_VIRTUAL_ID:
		tegra->support_usb_id = false;
		break;
	default:
		pr_info("otg usb id detection type is unknown\n");
		break;
	}
}

static int tegra_otg_conf(struct platform_device *pdev)
{
	struct tegra_usb_otg_data *pdata = dev_get_platdata(&pdev->dev);
	struct tegra_otg *tegra;
	int err;

	if (!pdata) {
		dev_err(&pdev->dev, "unable to get platform data\n");
		return -ENODEV;
	}

	tegra = devm_kzalloc(&pdev->dev, sizeof(struct tegra_otg), GFP_KERNEL);
	if (!tegra) {
		dev_err(&pdev->dev, "unable to allocate tegra_otg\n");
		return -ENOMEM;
	}

	tegra->phy.otg = devm_kzalloc(&pdev->dev, sizeof(struct usb_otg), GFP_KERNEL);
	if (!tegra->phy.otg) {
		dev_err(&pdev->dev, "unable to allocate otg\n");
		return -ENOMEM;
	}

	tegra->vbus_reg = regulator_get(&pdev->dev, "usb_vbus");
	if (IS_ERR_OR_NULL(tegra->vbus_reg)) {
		pr_err("failed to get regulator usb_vbus: %ld\n",
			PTR_ERR(tegra->vbus_reg));
		tegra->vbus_reg = NULL;
	}

	spin_lock_init(&tegra->lock);
	mutex_init(&tegra->irq_work_mutex);

	INIT_WORK(&tegra->work, irq_work);

	platform_set_drvdata(pdev, tegra);
	tegra_clone = tegra;

	tegra->interrupt_mode = true;
	tegra->suspended = true;
	tegra->turn_off_vbus_on_lp0 =
			pdata->ehci_pdata->u_data.host.turn_off_vbus_on_lp0;
	tegra->support_y_cable = true;
	tegra->y_cable_conn = false;
	tegra->support_pmu_vbus = pdata->ehci_pdata->support_pmu_vbus;
	tegra->id_det_gpio = pdata->id_det_gpio ? pdata->id_det_gpio : -1;
	tegra->pdata = pdata;
	tegra_otg_set_id_detection_type(tegra);

	tegra->phy.dev = &pdev->dev;
	tegra->phy.label = "tegra-otg";
	tegra->phy.set_suspend = tegra_otg_set_suspend;
	tegra->phy.set_power = tegra_otg_set_power;
	tegra->phy.state = OTG_STATE_A_SUSPEND;
	tegra->phy.otg->phy = &tegra->phy;
	tegra->phy.otg->set_host = tegra_otg_set_host;
	tegra->phy.otg->set_peripheral = tegra_otg_set_peripheral;

	if (tegra->support_pmu_vbus) {
		if (!pdata->ehci_pdata->vbus_extcon_dev_name) {
			dev_err(&pdev->dev, "Missing vbus_extcon_dev_name!\n");
			err = -EINVAL;
			goto err_vbus_extcon;
		}
		tegra->vbus_extcon_dev = extcon_get_extcon_dev(pdata->
					ehci_pdata->vbus_extcon_dev_name);
		if (!tegra->vbus_extcon_dev) {
			dev_err(&pdev->dev, "Cannot get the %s extcon dev\n",
				pdata->ehci_pdata->vbus_extcon_dev_name);
			err = -ENODEV;
			goto err_vbus_extcon;
		}
		otg_vbus_nb.notifier_call = otg_notifications;
		extcon_register_notifier(tegra->vbus_extcon_dev, &otg_vbus_nb);
	}

	if (tegra->support_pmu_id) {
		if (!pdata->ehci_pdata->id_extcon_dev_name) {
			dev_err(&pdev->dev, "Missing id_extcon_dev_name!\n");
			err = -EINVAL;
			goto err_id_extcon;
		}
		tegra->id_extcon_dev = extcon_get_extcon_dev(pdata->
					ehci_pdata->id_extcon_dev_name);
		if (!tegra->id_extcon_dev) {
			dev_err(&pdev->dev, "Cannot get the %s extcon dev\n",
					pdata->ehci_pdata->id_extcon_dev_name);
			err = -ENODEV;
			goto err_id_extcon;
		}
		otg_id_nb.notifier_call = otg_notifications;
		extcon_register_notifier(tegra->id_extcon_dev, &otg_id_nb);
	}

	ATOMIC_INIT_NOTIFIER_HEAD(&tegra->phy.notifier);

	err = usb_add_phy(&tegra->phy, USB_PHY_TYPE_USB2);
	if (err) {
		dev_err(&pdev->dev, "usb_set_transceiver failed\n");
		goto err_set_trans;
	}

	return 0;
err_set_trans:
	if (tegra->support_pmu_id)
		extcon_unregister_notifier(tegra->id_extcon_dev,
						&otg_id_nb);
err_id_extcon:
	if (tegra->support_pmu_vbus)
		extcon_unregister_notifier(tegra->vbus_extcon_dev,
						&otg_vbus_nb);
err_vbus_extcon:
	if (tegra->vbus_reg)
		regulator_put(tegra->vbus_reg);
	return err;
}

static int tegra_otg_start(struct platform_device *pdev)
{
	struct usb_phy *otg_trans = usb_get_phy(USB_PHY_TYPE_USB2);
	struct tegra_otg *tegra;
	struct resource *res;
	int err;

	tegra = container_of(otg_trans, struct tegra_otg, phy);
	tegra->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(tegra->clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		err = PTR_ERR(tegra->clk);
		goto err_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get I/O memory\n");
		err = -ENXIO;
		goto err_io;
	}

	tegra->regs = ioremap(res->start, resource_size(res));
	if (!tegra->regs) {
		err = -ENOMEM;
		goto err_io;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		err = -ENXIO;
		goto err_irq;
	}

	tegra->irq = res->start;
	err = devm_request_threaded_irq(&pdev->dev, tegra->irq, tegra_otg_irq,
				   NULL,
				   IRQF_SHARED | IRQF_TRIGGER_HIGH,
				   "tegra-otg", tegra);
	if (err) {
		dev_err(&pdev->dev, "Failed to register IRQ\n");
		goto err_irq;
	}

	if (tegra->support_gpio_id && gpio_is_valid(tegra->id_det_gpio)) {
		err = gpio_request(tegra->id_det_gpio, "id_det_gpio");
		if (err) {
			dev_err(&pdev->dev,
				"failed to allocate id_det_gpio\n");
			goto err_id_gpio_req;
		}
		gpio_direction_input(tegra->id_det_gpio);

		err = request_threaded_irq(gpio_to_irq(tegra->id_det_gpio), NULL
			, tegra_otg_id_detect_gpio_thr, IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_TRIGGER_RISING
			, "tegra-otg", tegra);
		if (err) {
			dev_err(&pdev->dev, "request irq error\n");
			goto err_id_gpio_irq;
		}
	}

	if (tegra->support_y_cable) {
		tegra->vbus_bat_reg = regulator_get(&pdev->dev, "usb_bat_chg");
		if (IS_ERR_OR_NULL(tegra->vbus_bat_reg)) {
			pr_err("failed to get regulator usb_bat_chg: %ld\n",
					PTR_ERR(tegra->vbus_bat_reg));
			tegra->vbus_bat_reg = NULL;
		}

		tegra->edev = kzalloc(sizeof(struct extcon_dev), GFP_KERNEL);
		if (!tegra->edev) {
			dev_err(&pdev->dev, "failed to allocate memory for extcon\n");
			err = -ENOMEM;
			goto err_id_free_irq;
		}
		tegra->edev->name = "tegra-otg";
		tegra->edev->supported_cable =
				(const char **) tegra_otg_extcon_cable;
		err = extcon_dev_register(tegra->edev, &pdev->dev);
		if (err) {
			dev_err(&pdev->dev, "failed to register extcon device\n");
			kfree(tegra->edev);
			tegra->edev = NULL;
		}
	}

	return 0;
err_id_free_irq:
	if (gpio_is_valid(tegra->id_det_gpio))
		free_irq(gpio_to_irq(tegra->id_det_gpio), tegra);
err_id_gpio_irq:
	if (gpio_is_valid(tegra->id_det_gpio))
		gpio_free(tegra->id_det_gpio);
err_id_gpio_req:
err_irq:
	iounmap(tegra->regs);
err_io:
	clk_put(tegra->clk);
err_clk:
	if (tegra->vbus_reg)
		regulator_put(tegra->vbus_reg);
	if (tegra->support_y_cable && tegra->vbus_bat_reg)
		regulator_put(tegra->vbus_bat_reg);

	return err;
}

static int tegra_otg_probe(struct platform_device *pdev)
{
	struct tegra_otg *tegra;
	int err = 0;

	err = tegra_otg_conf(pdev);
	if (err) {
		dev_err(&pdev->dev, "otg configuration failed\n");
		goto err;
	}

	err = tegra_otg_start(pdev);
	if (err) {
		dev_err(&pdev->dev, "otg start failed\n");
		goto err;
	}

	tegra = tegra_clone;
	if (!tegra->support_usb_id && !tegra->support_pmu_id
					&& !tegra->support_gpio_id) {
		err = device_create_file(&pdev->dev, &dev_attr_enable_host);
		if (err) {
			dev_warn(&pdev->dev,
				"Can't register host-sysfs attribute\n");
			goto err;
		}
		err = device_create_file(&pdev->dev, &dev_attr_enable_device);
		if (err) {
			dev_warn(&pdev->dev,
				"Can't register device-sysfs attribute\n");
			device_remove_file(&pdev->dev, &dev_attr_enable_host);
			goto err;
		}
	}

	tegra_pd_add_device(tegra->phy.dev);
	pm_runtime_use_autosuspend(tegra->phy.dev);
	pm_runtime_set_autosuspend_delay(tegra->phy.dev, 100);
	pm_runtime_enable(tegra->phy.dev);

	dev_info(&pdev->dev, "otg transceiver registered\n");
err:
	return err;
}

static int __exit tegra_otg_remove(struct platform_device *pdev)
{
	struct tegra_otg *tegra = platform_get_drvdata(pdev);

	if (tegra->vbus_reg)
		regulator_put(tegra->vbus_reg);

	if (tegra->support_y_cable && tegra->vbus_bat_reg)
		regulator_put(tegra->vbus_bat_reg);

	if (tegra->support_gpio_id && gpio_is_valid(tegra->id_det_gpio)) {
		free_irq(gpio_to_irq(tegra->id_det_gpio), tegra);
		gpio_free(tegra->id_det_gpio);
	} else if (tegra->support_pmu_id) {
		extcon_unregister_notifier(tegra->id_extcon_dev, &otg_id_nb);
	}

	if (tegra->support_pmu_vbus)
		extcon_unregister_notifier(tegra->vbus_extcon_dev,
							&otg_vbus_nb);

	if (tegra->edev != NULL) {
		extcon_dev_unregister(tegra->edev);
		kfree(tegra->edev);
	}

	pm_runtime_disable(tegra->phy.dev);
	usb_remove_phy(&tegra->phy);
	iounmap(tegra->regs);
	clk_put(tegra->clk);
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&tegra->irq_work_mutex);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_otg_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg *tegra = platform_get_drvdata(pdev);
	enum usb_otg_state from = tegra->phy.state;
	unsigned int val;
	int err = 0;

	mutex_lock(&tegra->irq_work_mutex);
	DBG("%s(%d) BEGIN state : %s\n", __func__, __LINE__,
				tegra_state_name(tegra->phy.state));

	pm_runtime_get_sync(dev);
	clk_prepare_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	val &= ~(USB_ID_INT_EN | USB_VBUS_INT_EN);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	clk_disable_unprepare(tegra->clk);
	pm_runtime_put_sync(dev);

	if (from == OTG_STATE_A_HOST && tegra->turn_off_vbus_on_lp0)
		tegra_otg_vbus_enable(tegra->vbus_reg, 0);

	if (tegra->irq) {
		err = enable_irq_wake(tegra->irq);
		if (err < 0) {
			dev_err(&pdev->dev,
			"Couldn't enable USB otg mode wakeup,"
			"irq=%d, error=%d\n", tegra->irq, err);
			goto fail;
		}
	}

	if (!(tegra->id_det_gpio == -1)) {
		err = enable_irq_wake(
				gpio_to_irq(tegra->id_det_gpio));
		if (err < 0) {
			dev_err(&pdev->dev,
			"Couldn't enable USB otg mode gpio wakeup, irq=%d,"
			"error=%d\n", gpio_to_irq(tegra->id_det_gpio), err);
			goto fail;
		}
	}
fail:
	tegra->suspended = true;
	DBG("%s(%d) END\n", __func__, __LINE__);
	mutex_unlock(&tegra->irq_work_mutex);
	return err;
}

static void tegra_otg_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg *tegra = platform_get_drvdata(pdev);
	int val;
	unsigned long flags;
	int err = 0;

	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	mutex_lock(&tegra->irq_work_mutex);
	if (!tegra->suspended) {
		mutex_unlock(&tegra->irq_work_mutex);
		return ;
	}
	if (tegra->irq) {
		err = disable_irq_wake(tegra->irq);
		if (err < 0)
			dev_err(&pdev->dev,
			"Couldn't disable USB otg mode wakeup,"
			"irq=%d, error=%d\n", tegra->irq, err);
	}

	if (!(tegra->id_det_gpio == -1)) {
		err = disable_irq_wake(gpio_to_irq(tegra->id_det_gpio));
		if (err < 0)
			dev_err(&pdev->dev,
			"Couldn't disable USB otg mode gpio wakeup, irq=%d,"
			"error=%d\n", gpio_to_irq(tegra->id_det_gpio), err);
	}
	/* Detect cable status after LP0 for all detection types */

	if (tegra->support_usb_id || !tegra->support_pmu_vbus) {
		/* Clear pending interrupts  */
		pm_runtime_get_sync(dev);
		clk_prepare_enable(tegra->clk);
		val = otg_readl(tegra, USB_PHY_WAKEUP);
		otg_writel(tegra, val, USB_PHY_WAKEUP);
		DBG("%s(%d) PHY WAKEUP : 0x%x\n", __func__, __LINE__, val);
		clk_disable_unprepare(tegra->clk);
		pm_runtime_mark_last_busy(dev);
		pm_runtime_put_autosuspend(dev);

		spin_lock_irqsave(&tegra->lock, flags);
		if (tegra->support_usb_id)
			val |= USB_ID_INT_EN | USB_ID_PIN_WAKEUP_EN;
		if (!tegra->support_pmu_vbus)
			val |= USB_VBUS_INT_EN | USB_VBUS_WAKEUP_EN;
		tegra->int_status = val;
		spin_unlock_irqrestore(&tegra->lock, flags);
	}

	if (tegra->support_pmu_vbus || tegra->support_pmu_id)
		otg_notifications(NULL, 0, NULL);

	if (tegra->support_gpio_id && gpio_is_valid(tegra->id_det_gpio)) {
		mutex_unlock(&tegra->irq_work_mutex);
		check_host_cable_connection(tegra);
		mutex_lock(&tegra->irq_work_mutex);
	}

	if (tegra->turn_off_vbus_on_lp0 &&
		!(tegra->int_status & USB_ID_STATUS)) {
		/* Handle Y-cable */
		if (tegra->support_y_cable &&
				(tegra->int_status & USB_VBUS_STATUS)) {
			tegra_otg_set_current(tegra->vbus_bat_reg,
				YCABLE_CHARGING_CURRENT_UA);
			tegra->y_cable_conn = true;
			tegra_otg_set_extcon_state(tegra);
		} else {
			tegra_otg_vbus_enable(tegra->vbus_reg, 1);
		}
	}

	/* Call work to set appropriate state */
	schedule_work(&tegra->work);

	enable_interrupt(tegra, true);

	tegra->suspended = false;

	DBG("%s(%d) END\n", __func__, __LINE__);
	mutex_unlock(&tegra->irq_work_mutex);
}

static const struct dev_pm_ops tegra_otg_pm_ops = {
	.complete = tegra_otg_resume,
	.suspend = tegra_otg_suspend,
};
#endif

static struct platform_driver tegra_otg_driver = {
	.driver = {
		.name  = "tegra-otg",
#ifdef CONFIG_PM
		.pm    = &tegra_otg_pm_ops,
#endif
	},
	.remove  = __exit_p(tegra_otg_remove),
	.probe   = tegra_otg_probe,
};

static int __init tegra_otg_init(void)
{
	return platform_driver_register(&tegra_otg_driver);
}
fs_initcall(tegra_otg_init);

static void __exit tegra_otg_exit(void)
{
	platform_driver_unregister(&tegra_otg_driver);
}
module_exit(tegra_otg_exit);
