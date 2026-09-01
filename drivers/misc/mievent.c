// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Event reporting used by the Xiaomi drivers.  See include/miev/mievent.h for
 * what this does and does not do.
 */

#define pr_fmt(fmt) "mievent: " fmt

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <miev/mievent.h>

/* An event carries a bounded number of pairs, in a bounded amount of space. */
#define MIEVENT_MAX_PARAS	64
#define MIEVENT_MAX_SIZE	2048
#define MIEVENT_MAX_PAIR	256

/**
 * cdev_tevent_alloc() - begin building an event
 * @eventid: which event this is
 *
 * Return: the event, or NULL if it could not be allocated.
 */
struct misight_mievent *cdev_tevent_alloc(unsigned int eventid)
{
	struct misight_mievent *event;

	event = kzalloc(sizeof(*event), GFP_KERNEL);
	if (!event)
		return NULL;

	event->buf_ptr = kcalloc(MIEVENT_MAX_PARAS, sizeof(*event->buf_ptr),
				 GFP_KERNEL);
	if (!event->buf_ptr) {
		kfree(event);
		return NULL;
	}

	event->eventid = eventid;
	event->time = ktime_get_real_seconds();

	return event;
}
EXPORT_SYMBOL_GPL(cdev_tevent_alloc);

static int cdev_tevent_add(struct misight_mievent *event, const char *key,
			   const char *value)
{
	char *pair;
	int len;

	if (!event || !key || !value)
		return -EINVAL;

	if (event->para_cnt >= MIEVENT_MAX_PARAS)
		return -ENOSPC;

	pair = kasprintf(GFP_KERNEL, "\"%s\":\"%s\"", key, value);
	if (!pair)
		return -ENOMEM;

	len = strlen(pair);
	if (len > MIEVENT_MAX_PAIR ||
	    event->used_size + len > MIEVENT_MAX_SIZE) {
		kfree(pair);
		return -ENOSPC;
	}

	event->buf_ptr[event->para_cnt++] = pair;
	event->used_size += len;

	return 0;
}

/**
 * cdev_tevent_add_int() - add an integer value to an event
 * @event: the event
 * @key:   name of the value
 * @value: the value
 */
int cdev_tevent_add_int(struct misight_mievent *event, const char *key,
			long value)
{
	char buf[32];

	scnprintf(buf, sizeof(buf), "%ld", value);

	return cdev_tevent_add(event, key, buf);
}
EXPORT_SYMBOL_GPL(cdev_tevent_add_int);

/**
 * cdev_tevent_add_str() - add a string value to an event
 * @event: the event
 * @key:   name of the value
 * @value: the value
 */
int cdev_tevent_add_str(struct misight_mievent *event, const char *key,
			const char *value)
{
	return cdev_tevent_add(event, key, value);
}
EXPORT_SYMBOL_GPL(cdev_tevent_add_str);

/**
 * cdev_tevent_write() - finish an event
 * @event: the event
 *
 * There is nowhere to send it, so the event is only checked and accounted for.
 * The caller still owns it and must destroy it.
 */
int cdev_tevent_write(struct misight_mievent *event)
{
	if (!event)
		return -EINVAL;

	pr_debug("event %u: %u values, %u bytes\n",
		 event->eventid, event->para_cnt, event->used_size);

	return 0;
}
EXPORT_SYMBOL_GPL(cdev_tevent_write);

/**
 * cdev_tevent_destroy() - release an event
 * @event: the event, which may be NULL
 */
void cdev_tevent_destroy(struct misight_mievent *event)
{
	unsigned int i;

	if (!event)
		return;

	for (i = 0; i < event->para_cnt; i++)
		kfree(event->buf_ptr[i]);

	kfree(event->buf_ptr);
	kfree(event);
}
EXPORT_SYMBOL_GPL(cdev_tevent_destroy);

MODULE_DESCRIPTION("Xiaomi driver event reporting");
MODULE_LICENSE("GPL");
