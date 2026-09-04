// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Panel state for the charging drivers.
 *
 * Charging behaviour depends on whether the user is looking at the screen: a
 * device charging in a pocket can be driven harder than one in someone's hand
 * with the display at full brightness.  This watches the panel and passes
 * what it sees to the rest of the stack, and answers the current state for
 * drivers that would rather ask than listen.
 *
 * The panel is found through the "panel" phandles on this node.  Only one of
 * them is the panel actually fitted, and it is not registered with DRM until
 * the display driver has probed, so the search is retried until it appears.
 */

#define pr_fmt(fmt) "mca_panel: " fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#include <linux/workqueue.h>

#include <drm/drm_panel.h>

#include <mca/common/mca_event.h>
#include <mca/common/mca_panel.h>

/* The display driver may probe well after this one. */
#define MCA_PANEL_RETRY_MS	5000
#define MCA_PANEL_RETRY_MAX	3

struct mca_panel {
	struct device		*dev;
	struct drm_panel	*panel;
	void			*cookie;
	struct delayed_work	register_work;
	unsigned int		retry_count;
	bool			screen_on;
	bool			hbm_on;
};

static struct mca_panel *mca_panel;

int mca_panel_get_screen_state(void)
{
	return mca_panel && mca_panel->screen_on;
}
EXPORT_SYMBOL_GPL(mca_panel_get_screen_state);

int mca_panel_get_hbm_state(void)
{
	return mca_panel && mca_panel->hbm_on;
}
EXPORT_SYMBOL_GPL(mca_panel_get_hbm_state);

static void mca_panel_event_notifier_callback(enum panel_event_notifier_tag tag,
					      struct panel_event_notification *notification,
					      void *pvt_data)
{
	struct mca_panel *mp = pvt_data;
	bool screen_on = mp->screen_on;
	bool hbm_on = mp->hbm_on;

	if (!notification) {
		pr_err("Invalid panel notification\n");
		return;
	}

	pr_debug("panel event received, type: %d\n",
		 notification->notif_type);

	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_UNBLANK:
		screen_on = true;
		break;
	case DRM_PANEL_EVENT_BLANK:
	case DRM_PANEL_EVENT_BLANK_LP:
		screen_on = false;
		break;
	case DRM_PANEL_EVENT_HBM_ON:
		hbm_on = true;
		break;
	case DRM_PANEL_EVENT_HBM_OFF:
		hbm_on = false;
		break;
	default:
		pr_debug("Ignore panel event: %d\n", notification->notif_type);
		return;
	}

	if (screen_on == mp->screen_on && hbm_on == mp->hbm_on)
		return;

	mp->screen_on = screen_on;
	mp->hbm_on = hbm_on;

	mca_event_block_notify(MCA_EVENT_TYPE_PANEL,
			       notification->notif_type, NULL);
}

/* Return the panel among this node's phandles that DRM knows about. */
static struct drm_panel *mca_panel_find(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *node;
	struct drm_panel *panel;
	int count, i;

	node = of_find_node_by_name(np, "charge-screen");
	if (!node)
		node = np;

	count = of_count_phandle_with_args(node, "panel", NULL);
	if (count <= 0) {
		pr_err("ERROR: Cannot find node with panel!\n");
		return ERR_PTR(-ENODEV);
	}

	for (i = 0; i < count; i++) {
		struct device_node *pnode;

		pnode = of_parse_phandle(node, "panel", i);
		if (!pnode)
			continue;

		panel = of_drm_find_panel(pnode);
		of_node_put(pnode);

		if (!IS_ERR(panel))
			return panel;
	}

	return ERR_PTR(-EPROBE_DEFER);
}

static void mca_panel_register_panel_notifier_work(struct work_struct *work)
{
	struct mca_panel *mp = container_of(to_delayed_work(work),
					    struct mca_panel, register_work);
	struct drm_panel *panel;
	void *cookie;

	panel = mca_panel_find(mp->dev);
	if (IS_ERR(panel)) {
		pr_err("Failed to find active panel, rc=%d\n",
		       (int)PTR_ERR(panel));
		goto retry;
	}
	mp->panel = panel;

	cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_PRIMARY,
					       PANEL_EVENT_NOTIFIER_CLIENT_BATTERY_CHARGER,
					       mp->panel,
					       mca_panel_event_notifier_callback,
					       mp);
	if (IS_ERR_OR_NULL(cookie)) {
		pr_err("Failed to register panel event notifier, rc=%d\n",
		       (int)PTR_ERR(cookie));
		goto retry;
	}

	mp->cookie = cookie;
	pr_debug("register panel notifier successful\n");
	return;

retry:
	/*
	 * Both failures share the one budget.  The panel driver may simply not
	 * have probed yet, and either step can be the one that is early, so
	 * a failure to register is worth another go just as much as a panel
	 * that is not there to find.
	 */
	if (++mp->retry_count > MCA_PANEL_RETRY_MAX)
		return;

	pr_debug("retry register panel notifier, retry_count = %d\n",
		 mp->retry_count);
	queue_delayed_work(system_wq, &mp->register_work,
			   msecs_to_jiffies(MCA_PANEL_RETRY_MS));
}

static int mca_panel_probe(struct platform_device *pdev)
{
	struct mca_panel *mp;

	mp = devm_kzalloc(&pdev->dev, sizeof(*mp), GFP_KERNEL);
	if (!mp) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	mp->dev = &pdev->dev;
	/* Assume the screen is on until the panel says otherwise. */
	mp->screen_on = true;
	INIT_DELAYED_WORK(&mp->register_work,
			  mca_panel_register_panel_notifier_work);

	platform_set_drvdata(pdev, mp);
	mca_panel = mp;

	/*
	 * Give DRM time to register the panel before the first look: at probe
	 * it is not there yet, so an immediate attempt only spends one of the
	 * retries on a failure that is expected.
	 */
	queue_delayed_work(system_wq, &mp->register_work,
			   msecs_to_jiffies(MCA_PANEL_RETRY_MS));

	pr_debug("probe OK\n");

	return 0;
}

static int mca_panel_remove(struct platform_device *pdev)
{
	struct mca_panel *mp = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&mp->register_work);

	if (mp->cookie)
		panel_event_notifier_unregister(mp->cookie);

	mca_panel = NULL;

	return 0;
}

static void mca_panel_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id mca_panel_match[] = {
	{ .compatible = "mca,mca_panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, mca_panel_match);

static struct platform_driver mca_panel_driver = {
	.driver = {
		.name = "mca_panel",
		.of_match_table = mca_panel_match,
	},
	.probe = mca_panel_probe,
	.remove = mca_panel_remove,
	.shutdown = mca_panel_shutdown,
};
module_platform_driver(mca_panel_driver);

MODULE_DESCRIPTION("mca get panel event");
MODULE_LICENSE("GPL");
