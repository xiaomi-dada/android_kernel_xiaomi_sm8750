// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Board identification.
 *
 * The bootloader describes the board to this driver in module parameters it
 * appends to the kernel command line -- hwid.project=, hwid.hwid_value=,
 * hwid.project_adc= and hwid.build_adc= -- which is how Xiaomi's own hwid
 * driver takes them, down to the parameter names and their descriptions.
 *
 * hwid_value packs the hardware revision and the sales region into one word:
 *
 *	bits  0..15	minor revision
 *	bits 16..31	major revision, of which
 *	bits 16..19	is the build, and
 *	bits 20..31	is the region
 *
 * so the major revision carries the region in its upper nibbles.  That is
 * what the vendor driver's accessors compute, and the charging stack compares
 * the region against the values in enum hardware_country.
 */

#define pr_fmt(fmt) "hwid: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>

#include <soc/qcom/socinfo.h>

#include <linux/hwid.h>

/* socinfo IDs are per SoC; this tree builds for one. */
#define SOC_ID_SM8750		618

static uint project;
module_param(project, uint, 0444);
MODULE_PARM_DESC(project, "xiaomi project serial num predefine");

static uint hwid_value;
module_param(hwid_value, uint, 0444);
MODULE_PARM_DESC(hwid_value, "xiaomi hwid value correspondingly different build");

static uint project_adc;
module_param(project_adc, uint, 0444);
MODULE_PARM_DESC(project_adc, "xiaomi adc value of project resistance");

static uint build_adc;
module_param(build_adc, uint, 0444);
MODULE_PARM_DESC(build_adc, "xiaomi adc value of build resistance");

/*
 * Boards this SoC is used in, in the order the project number counts them
 * from one.  dada is the second.
 */
static const char * const sm8750_projects[] = {
	"haotian", "dada", "miro", "xuanyuan", "bixi", "nirvana",
	"onyx", "luming", "piano", "yupei", "annibale",
};

uint32_t get_hw_version_platform(void)
{
	return project;
}
EXPORT_SYMBOL_GPL(get_hw_version_platform);

uint32_t get_hw_id_value(void)
{
	return hwid_value;
}
EXPORT_SYMBOL_GPL(get_hw_id_value);

uint32_t get_hw_country_version(void)
{
	return hwid_value >> 20;
}
EXPORT_SYMBOL_GPL(get_hw_country_version);

uint32_t get_hw_version_major(void)
{
	return (hwid_value >> 16) & 0xffff;
}
EXPORT_SYMBOL_GPL(get_hw_version_major);

uint32_t get_hw_version_minor(void)
{
	return hwid_value & 0xffff;
}
EXPORT_SYMBOL_GPL(get_hw_version_minor);

uint32_t get_hw_version_build(void)
{
	return (hwid_value >> 16) & 0xf;
}
EXPORT_SYMBOL_GPL(get_hw_version_build);

uint32_t get_hw_project_adc(void)
{
	return project_adc;
}
EXPORT_SYMBOL_GPL(get_hw_project_adc);

uint32_t get_hw_build_adc(void)
{
	return build_adc;
}
EXPORT_SYMBOL_GPL(get_hw_build_adc);

const char *product_name_get(void)
{
	if (socinfo_get_id() != SOC_ID_SM8750 || !project ||
	    project > ARRAY_SIZE(sm8750_projects))
		return "unknown";

	return sm8750_projects[project - 1];
}
EXPORT_SYMBOL_GPL(product_name_get);

/*
 * Xiaomi's driver also publishes the four values under /sys/hwid, and then
 * calls kobject_del() on the directory it just filled, on the success path as
 * well as the failure one, so the directory is gone before anything can read
 * it.  Nothing reads it; it is not carried here.
 */
static int __init hwid_init(void)
{
	pr_info("project %u (%s) hwid 0x%x region %u revision %u.%u\n",
		project, product_name_get(), hwid_value,
		get_hw_country_version(), get_hw_version_major(),
		get_hw_version_minor());

	return 0;
}
module_init(hwid_init);

MODULE_DESCRIPTION("Xiaomi board identification");
MODULE_LICENSE("GPL");
