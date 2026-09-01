/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _BATTERY_CHARGER_H
#define _BATTERY_CHARGER_H

#include <linux/notifier.h>

enum battery_charger_prop {
	BATTERY_RESISTANCE,
	FLASH_ACTIVE,
	BATTERY_CHARGER_PROP_MAX,
};

enum bc_hboost_event {
	VMAX_CLAMP,
};

/*
 * On a board whose charging the ADSP owns, the QTI client above is not built
 * and the MCA proxy answers for it instead -- so the prototypes have to be
 * visible for either provider, not just the first.
 */
#if IS_ENABLED(CONFIG_QTI_BATTERY_CHARGER) || IS_ENABLED(CONFIG_XM_POWER_SUPPLY)
int qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val);
int qti_battery_charger_set_prop(const char *name,
				enum battery_charger_prop prop_id, int val);
int register_hboost_event_notifier(struct notifier_block *nb);
int unregister_hboost_event_notifier(struct notifier_block *nb);
#else
static inline int
qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val)
{
	return -EINVAL;
}

static inline int
qti_battery_charger_set_prop(const char *name,
				enum battery_charger_prop prop_id, int val)
{
	return -EINVAL;
}

static inline int register_hboost_event_notifier(struct notifier_block *nb)
{
	return -EOPNOTSUPP;
}

static inline int unregister_hboost_event_notifier(struct notifier_block *nb)
{
	return -EOPNOTSUPP;
}
#endif

#endif
