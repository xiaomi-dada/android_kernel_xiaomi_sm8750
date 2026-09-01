/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Event reporting used by the Xiaomi drivers.
 *
 * A driver builds an event out of key/value pairs and writes it.  Xiaomi's
 * module carries the events on to userspace: it queues each one in a kfifo
 * and publishes /dev/miev for libmisight to read them out of, which is how
 * MIUI's telemetry service collects them.  Nothing on this system reads that
 * device -- the service is not part of the build -- so the events have no
 * destination, and the queue, the character device and the ioctl interface
 * that go with it are not carried here.  An event is validated, kept while it
 * is being built, and dropped on write.  Xiaomi's own published header stubs
 * these functions out in the header itself, so a device built from that
 * source reports nothing either.
 */

#ifndef _EVENT_MIEVENT_H_
#define _EVENT_MIEVENT_H_

#include <linux/types.h>

enum DATA_TYPE { INT_T = 0, STR_T };

/**
 * struct misight_mievent - an event being built
 * @eventid:	which event this is
 * @para_cnt:	key/value pairs added so far
 * @used_size:	bytes the pairs occupy
 * @time:	when the event was allocated, in seconds
 * @buf_ptr:	the pairs, as formatted strings
 */
struct misight_mievent {
	unsigned int eventid;
	unsigned int para_cnt;
	unsigned int used_size;
	long long time;
	char **buf_ptr;
};

struct mievent_payload {
	int type;
	char *key;
	char *value;
};

struct misight_mievent *cdev_tevent_alloc(unsigned int eventid);
int cdev_tevent_add_int(struct misight_mievent *event, const char *key,
			long value);
int cdev_tevent_add_str(struct misight_mievent *event, const char *key,
			const char *value);
int cdev_tevent_write(struct misight_mievent *event);
void cdev_tevent_destroy(struct misight_mievent *event);

#endif /* _EVENT_MIEVENT_H_ */
