// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Where the charging stack shows itself to userspace.  See
 * include/mca/common/mca_sysfs.h.
 */

#define pr_fmt(fmt) "[mca_sysfs]%s:%d " fmt, __func__, __LINE__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <mca/common/mca_sysfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>

/* The debugfs directory the whole stack writes under. */
#define MCA_DEBUGFS_ROOT	"charger_debug"

/* Longest write a debugfs file will take. */
#define MCA_DEBUGFS_WRITE_MAX	256

/*
 * The devices the charging stack shows itself through.  They are created up
 * front rather than by whichever driver happens to want one first, because
 * the attributes are spread across a dozen modules and every one of them
 * expects to find its device already there.
 */
static const char * const g_mca_sysfs_dev_name_list[] = {
	MCA_SYSFS_DEV_CHARGER,
	MCA_SYSFS_DEV_FUELGAUGE,
	MCA_SYSFS_DEV_TYPEC,
	MCA_SYSFS_DEV_BATTERY,
	MCA_SYSFS_DEV_HW_MONITOR,
};

/**
 * struct mca_sysfs_dev_node - one device in a class
 * @node: links it into its class's device list
 * @name: what the device is called
 * @dev:  the device itself
 */
struct mca_sysfs_dev_node {
	struct list_head	node;
	const char		*name;
	struct device		*dev;
};

/**
 * struct mca_sysfs_class_node - one class
 * @node:            links it into the list of classes
 * @name:            what the class is called
 * @mca_class:       the class itself
 * @dev_list_header: the devices created under it
 */
struct mca_sysfs_class_node {
	struct list_head	node;
	const char		*name;
	struct class		*mca_class;
	struct list_head	dev_list_header;
};

static LIST_HEAD(mca_sysfs_class_list);
static DEFINE_MUTEX(mca_sysfs_lock);

static struct dentry *mca_debugfs_root;

/*
 * Modules load in whatever order the dependency graph allows, so whichever
 * one asks for a class first creates it and the rest find it.
 */
static noinline struct mca_sysfs_class_node *
mca_sysfs_get_or_create_class(const char *cls_name)
{
	struct mca_sysfs_class_node *cls;

	mutex_lock(&mca_sysfs_lock);

	list_for_each_entry(cls, &mca_sysfs_class_list, node) {
		if (!strcmp(cls->name, cls_name))
			goto out;
	}

	cls = kzalloc(sizeof(*cls), GFP_KERNEL);
	if (!cls)
		goto out;

	cls->name = cls_name;
	cls->mca_class = class_create(cls_name);
	if (IS_ERR(cls->mca_class)) {
		pr_err("%s dir create fail\n", cls_name);
		kfree(cls);
		cls = NULL;
		goto out;
	}

	INIT_LIST_HEAD(&cls->dev_list_header);
	list_add(&cls->node, &mca_sysfs_class_list);

out:
	mutex_unlock(&mca_sysfs_lock);

	return cls;
}

static struct mca_sysfs_class_node *mca_sysfs_find_class(const char *cls_name)
{
	struct mca_sysfs_class_node *cls;

	list_for_each_entry(cls, &mca_sysfs_class_list, node) {
		if (!strcmp(cls->name, cls_name))
			return cls;
	}

	pr_err("can not find class %s\n", cls_name);

	return NULL;
}

static struct device *mca_sysfs_find_dev(const char *cls_name,
					 const char *dev_name)
{
	struct mca_sysfs_class_node *cls;
	struct mca_sysfs_dev_node *dev_node;

	cls = mca_sysfs_find_class(cls_name);
	if (!cls)
		return NULL;

	list_for_each_entry(dev_node, &cls->dev_list_header, node) {
		if (!strcmp(dev_node->name, dev_name))
			return dev_node->dev;
	}

	return NULL;
}

/**
 * mca_sysfs_create_group() - create a device and give it a group of attributes
 * @cls_name: the class it belongs to, created if this is the first caller
 * @dev_name: what the device is called
 * @group:    the attributes
 *
 * Return: the device, which later callers reach by name, or NULL.
 */
struct device *mca_sysfs_create_group(const char *cls_name,
				      const char *dev_name,
				      const struct attribute_group *group)
{
	struct mca_sysfs_class_node *cls;
	struct mca_sysfs_dev_node *dev_node;
	struct device *dev;

	if (!cls_name || !dev_name || !group)
		return NULL;

	cls = mca_sysfs_get_or_create_class(cls_name);
	if (!cls)
		return NULL;

	dev = device_create(cls->mca_class, NULL, 0, NULL, "%s", dev_name);
	if (IS_ERR(dev)) {
		/* The vendor's message loses its last letter here. */
		pr_err("create device %s fai\n", dev_name);
		return NULL;
	}

	if (sysfs_create_group(&dev->kobj, group)) {
		pr_err("%s creat sys group fail\n", dev_name);
		goto err;
	}

	dev_node = kzalloc(sizeof(*dev_node), GFP_KERNEL);
	if (!dev_node)
		goto err;

	dev_node->name = dev_name;
	dev_node->dev = dev;

	mutex_lock(&mca_sysfs_lock);
	list_add(&dev_node->node, &cls->dev_list_header);
	mutex_unlock(&mca_sysfs_lock);

	return dev;

err:
	device_unregister(dev);

	return NULL;
}
EXPORT_SYMBOL(mca_sysfs_create_group);

void mca_sysfs_remove_group(const char *cls_name, struct device *dev,
			    const struct attribute_group *group)
{
	struct mca_sysfs_class_node *cls;
	struct mca_sysfs_dev_node *dev_node, *tmp;

	if (!cls_name || !dev || !group)
		return;

	sysfs_remove_group(&dev->kobj, group);

	mutex_lock(&mca_sysfs_lock);

	cls = mca_sysfs_find_class(cls_name);
	if (cls) {
		list_for_each_entry_safe(dev_node, tmp, &cls->dev_list_header,
					 node) {
			if (dev_node->dev != dev)
				continue;

			list_del(&dev_node->node);
			kfree(dev_node);
			break;
		}
	}

	mutex_unlock(&mca_sysfs_lock);

	device_unregister(dev);
}
EXPORT_SYMBOL(mca_sysfs_remove_group);

/**
 * mca_sysfs_create_link_group() - add a subdirectory to an existing device
 * @dev_name:   the device to add it to
 * @link_name:  what the subdirectory is called
 * @target_dev: the device whose attributes it holds
 * @group:      the attributes
 *
 * A charger with two of something -- two load switches, two charge pumps --
 * gives each one a directory of its own, so that userspace reads the same
 * attribute names under different names rather than the same names with
 * different suffixes.
 */
int mca_sysfs_create_link_group(const char *dev_name, const char *link_name,
				struct device *target_dev,
				const struct attribute_group *group)
{
	struct device *dev;
	int ret;

	if (!dev_name || !link_name || !target_dev || !group)
		return -EINVAL;

	mutex_lock(&mca_sysfs_lock);
	dev = mca_sysfs_find_dev(MCA_SYSFS_CLASS, dev_name);
	mutex_unlock(&mca_sysfs_lock);
	if (!dev)
		return -ENODEV;

	ret = sysfs_create_group(&target_dev->kobj, group);
	if (ret) {
		pr_err("%s creat sys group fail\n", link_name);
		return ret;
	}

	ret = sysfs_create_link(&dev->kobj, &target_dev->kobj, link_name);
	if (ret) {
		pr_err("creat %s/%s fail\n", dev_name, link_name);
		sysfs_remove_group(&target_dev->kobj, group);
		return ret;
	}

	pr_debug("creat %s/%s success\n", dev_name, link_name);

	return 0;
}
EXPORT_SYMBOL(mca_sysfs_create_link_group);

void mca_sysfs_remove_link_group(const char *dev_name, const char *link_name,
				 struct device *target_dev,
				 const struct attribute_group *group)
{
	struct device *dev;

	if (!dev_name || !link_name || !target_dev || !group)
		return;

	mutex_lock(&mca_sysfs_lock);
	dev = mca_sysfs_find_dev(MCA_SYSFS_CLASS, dev_name);
	mutex_unlock(&mca_sysfs_lock);
	if (dev)
		sysfs_remove_link(&dev->kobj, link_name);

	sysfs_remove_group(&target_dev->kobj, group);
}
EXPORT_SYMBOL(mca_sysfs_remove_link_group);

/**
 * mca_sysfs_create_files() - add attributes to a device another module made
 * @dev_name:  the device to add them to
 * @attr_info: the attributes
 * @size:      how many
 *
 * The device must already exist: a module that owns a handful of attributes
 * on the charger does not create the charger.
 */
int mca_sysfs_create_files(const char *dev_name,
			   struct mca_sysfs_attr_info *attr_info, int size)
{
	struct device *dev;
	int i, ret;

	if (!dev_name || !attr_info)
		return -EINVAL;

	mutex_lock(&mca_sysfs_lock);
	dev = mca_sysfs_find_dev(MCA_SYSFS_CLASS, dev_name);
	mutex_unlock(&mca_sysfs_lock);
	if (!dev)
		return -ENODEV;

	for (i = 0; i < size; i++) {
		ret = device_create_file(dev, &attr_info[i].attr);
		if (ret) {
			pr_err("%s file create fail\n",
			       attr_info[i].attr.attr.name);
			while (i--)
				device_remove_file(dev, &attr_info[i].attr);

			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL(mca_sysfs_create_files);

/**
 * mca_sysfs_init_attrs() - build an attribute array from a driver's table
 * @attrs:     where to put them, with room for @size plus a terminator
 * @attr_info: the table
 * @size:      how many entries it has
 */
void mca_sysfs_init_attrs(struct attribute **attrs,
			  struct mca_sysfs_attr_info *attr_info, int size)
{
	int i;

	if (!attrs || !attr_info || size < 1)
		return;

	for (i = 0; i < size; i++)
		attrs[i] = &attr_info[i].attr.attr;

	attrs[size] = NULL;
}
EXPORT_SYMBOL(mca_sysfs_init_attrs);

/**
 * mca_sysfs_lookup_attr() - find the table entry a handler was called for
 * @name:      the attribute's name, as sysfs passed it
 * @attr_info: the table
 * @size:      how many entries it has
 *
 * Return: the entry, whose @sysfs_attr_name says which attribute it is, or
 * NULL.
 */
struct mca_sysfs_attr_info *mca_sysfs_lookup_attr(const char *name,
						  struct mca_sysfs_attr_info *attr_info,
						  int size)
{
	int i;

	if (!name || !attr_info)
		return NULL;

	for (i = 0; i < size; i++) {
		if (!strcmp(attr_info[i].attr.attr.name, name))
			return &attr_info[i];
	}

	return NULL;
}
EXPORT_SYMBOL(mca_sysfs_lookup_attr);

static int mca_debugfs_template_show(struct seq_file *s, void *unused)
{
	struct mca_debugfs_attr_data *d = s->private;
	char *buf;
	int rc;

	if (!d || !d->attr_info->show) {
		pr_err("invalid show\n");
		return -EINVAL;
	}

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/*
	 * The handler returns how much it wrote, and what it wrote is not
	 * necessarily a string -- a register dump has embedded NULs -- so the
	 * length is honoured rather than the buffer being treated as text.
	 */
	rc = d->attr_info->show(d, buf);
	if (rc > 0)
		seq_write(s, buf, rc);

	kfree(buf);

	return rc < 0 ? rc : 0;
}

static int mca_debugfs_template_open(struct inode *inode, struct file *file)
{
	return single_open(file, mca_debugfs_template_show, inode->i_private);
}

static ssize_t mca_debugfs_template_write(struct file *file,
					  const char __user *buf, size_t count,
					  loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mca_debugfs_attr_data *d = s->private;
	char kbuf[MCA_DEBUGFS_WRITE_MAX];

	if (!d || !d->attr_info->store) {
		pr_err("invalid store\n");
		return -EINVAL;
	}

	if (count >= MCA_DEBUGFS_WRITE_MAX) {
		pr_err("input too long\n");
		return -EINVAL;
	}

	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';

	return d->attr_info->store(d, kbuf, count);
}

static const struct file_operations mca_debugfs_fops = {
	.owner		= THIS_MODULE,
	.open		= mca_debugfs_template_open,
	.read		= seq_read,
	.write		= mca_debugfs_template_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/**
 * mca_debugfs_create_group() - give a driver a directory of debugfs files
 * @name:      what the directory is called
 * @attr_info: the files
 * @size:      how many
 * @data:      passed back to every handler
 */
int mca_debugfs_create_group(const char *name,
			     struct mca_debugfs_attr_info *attr_info, int size,
			     void *data)
{
	struct mca_debugfs_attr_data *d;
	struct dentry *dir, *looked_up;
	int i;

	if (!name || !attr_info)
		return -EINVAL;

	if (!mca_debugfs_root) {
		pr_err("root directory is null\n");
		return -ENODEV;
	}

	/*
	 * A driver that registers twice -- two instances of the same chip,
	 * say -- would otherwise get two directories of the same name, and
	 * only the first would be reachable.
	 *
	 * A lookup returns a reference of its own.  The files created below
	 * pin the directory, so it is dropped again before returning.
	 */
	dir = debugfs_lookup(name, mca_debugfs_root);
	looked_up = dir;
	if (!dir)
		dir = debugfs_create_dir(name, mca_debugfs_root);

	for (i = 0; i < size; i++) {
		d = kzalloc(sizeof(*d), GFP_KERNEL);
		if (!d) {
			if (looked_up)
				dput(dir);
			return -ENOMEM;
		}

		d->attr_info = &attr_info[i];
		d->private = data;

		debugfs_create_file(attr_info[i].name, attr_info[i].mode, dir,
				    d, &mca_debugfs_fops);
	}

	if (looked_up)
		dput(dir);

	pr_debug("group %s create succ\n", name);

	return 0;
}
EXPORT_SYMBOL(mca_debugfs_create_group);

static int __init mca_sysfs_init(void)
{
	struct mca_sysfs_class_node *cls;
	struct mca_sysfs_dev_node *dev_node;
	struct device *dev;
	int i;

	mca_debugfs_root = debugfs_create_dir(MCA_DEBUGFS_ROOT, NULL);
	if (IS_ERR(mca_debugfs_root)) {
		pr_err("Failed to create charger debugfs root directory: %d\n",
		       (int)PTR_ERR(mca_debugfs_root));
		mca_debugfs_root = NULL;
	}

	cls = mca_sysfs_get_or_create_class(MCA_SYSFS_CLASS);
	if (!cls)
		return 0;

	for (i = 0; i < ARRAY_SIZE(g_mca_sysfs_dev_name_list); i++) {
		dev_node = kzalloc(sizeof(*dev_node), GFP_KERNEL);
		if (!dev_node)
			return 0;

		dev = device_create(cls->mca_class, NULL, 0, NULL, "%s",
				    g_mca_sysfs_dev_name_list[i]);
		if (IS_ERR(dev)) {
			pr_err("create device %s fail\n",
			       g_mca_sysfs_dev_name_list[i]);
			kfree(dev_node);
			continue;
		}

		dev_node->name = g_mca_sysfs_dev_name_list[i];
		dev_node->dev = dev;

		mutex_lock(&mca_sysfs_lock);
		list_add(&dev_node->node, &cls->dev_list_header);
		mutex_unlock(&mca_sysfs_lock);
	}

	return 0;
}

static void __exit mca_sysfs_exit(void)
{
	debugfs_remove_recursive(mca_debugfs_root);
}

module_init(mca_sysfs_init);
module_exit(mca_sysfs_exit);

MODULE_DESCRIPTION("sysfs for the Xiaomi power drivers");
MODULE_LICENSE("GPL");
