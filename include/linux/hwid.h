/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Board identification.
 *
 * The Xiaomi drivers ask which board they are running on so they can adjust
 * for hardware revision and sales region.  The bootloader passes both on the
 * command line, which is where the values here come from.
 */

#ifndef __LINUX_HWID_H
#define __LINUX_HWID_H

#include <linux/types.h>

/*
 * Project identifiers, counting the boards this SoC is used in from one.
 * Xiaomi's hwid driver names them: haotian, dada, miro, xuanyuan, bixi,
 * nirvana, onyx, luming, piano, yupei, annibale.  This board is dada, the
 * second, and describes itself as O3 in its device tree, in
 * xiaomi,vendor_names and in its panel nodes; the charging drivers single out
 * 1 as O2 and 3 as O9, which is haotian and miro.  Nothing here runs on
 * either.
 */
enum hardware_project {
	HARDWARE_PROJECT_UNKNOWN = 0,
	HARDWARE_PROJECT_O2 = 1,
	HARDWARE_PROJECT_O3 = 2,
	HARDWARE_PROJECT_O9 = 3,
};

/*
 * Sales regions a board can report.  China is zero rather than an "unknown"
 * value, so a board whose bootloader passes no hwid value at all is treated
 * as a Chinese one; that is what the vendor drivers assume, and the region
 * only ever relaxes or tightens a charging limit.
 */
enum hardware_country {
	CountryCN = 0,
	CountryGlobal,
	CountryIndia,
	CountryJapan,
};

uint32_t get_hw_version_platform(void);
uint32_t get_hw_country_version(void);
uint32_t get_hw_version_major(void);
uint32_t get_hw_version_minor(void);
uint32_t get_hw_version_build(void);
uint32_t get_hw_id_value(void);
uint32_t get_hw_project_adc(void);
uint32_t get_hw_build_adc(void);
const char *product_name_get(void);

#endif /* __LINUX_HWID_H */
