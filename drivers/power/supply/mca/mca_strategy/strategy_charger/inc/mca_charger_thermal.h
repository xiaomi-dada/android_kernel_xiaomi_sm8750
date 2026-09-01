/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Charging within a thermal budget.
 *
 * The thermal framework decides how warm the phone may get; charging is one
 * of the things it turns down to stay there.  Each charging mode -- five volts
 * through the buck charger, twenty volts through a charge pump, a wireless pad
 * that authenticated as eighty watts -- gets its own cooling device, because
 * the same thermal level means a different current in each.
 *
 * A test rig needs the limits out of the way, so they can be removed, and the
 * fact that they were is remembered for the log.
 */

#ifndef __MCA_CHARGER_THERMAL_H
#define __MCA_CHARGER_THERMAL_H

#include <linux/types.h>

int mca_get_wls_charger_thermal_remove(bool *wls_thermal_remove);
int mca_set_wls_charger_thermal_remove(bool wls_thermal_remove);

#endif /* __MCA_CHARGER_THERMAL_H */
