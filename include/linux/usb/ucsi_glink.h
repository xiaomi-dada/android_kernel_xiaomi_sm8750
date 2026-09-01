/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __UCSI_GLINK_H__
#define __UCSI_GLINK_H__

#include <linux/errno.h>
#include <linux/usb/typec.h>

struct ucsi_glink_constat_info {
	enum typec_accessory acc;
	/*
	 * The connector's power operation mode, as UCSI reports it, for the
	 * one case a consumer needs to tell apart: a UFP that negotiated PD
	 * and talks USB.  Zero for everything else.
	 */
	int pwr_opmode;
};

/*
 * The only value pwr_opmode ever carries, mirroring the UCSI power operation
 * mode of the same number.  ucsi_qti_glink.c checks the two agree.
 */
#define UCSI_GLINK_PWR_OPMODE_PD	3

struct notifier_block;

#if IS_ENABLED(CONFIG_UCSI_QTI_GLINK)

int register_ucsi_glink_notifier(struct notifier_block *nb);
int unregister_ucsi_glink_notifier(struct notifier_block *nb);

#else

static inline int register_ucsi_glink_notifier(struct notifier_block *nb)
{
	return -ENODEV;
}

static inline int unregister_ucsi_glink_notifier(struct notifier_block *nb)
{
	return -ENODEV;
}

#endif
#endif
