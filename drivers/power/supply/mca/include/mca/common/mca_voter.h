/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2017, 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2025 The LineageOS Project
 */

#ifndef __MCA_COMMON_H
#define __MCA_COMMON_H

#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

struct mca_votable;

/**
 * enum mca_votable_type - how an election combines its clients' votes
 * @MCA_VOTE_MIN:    effective result is the smallest value among enabled
 *                   clients
 * @MCA_VOTE_MAX:    effective result is the largest value among enabled
 *                   clients
 * @MCA_VOTE_OR:  effective result is set when any client is enabled
 * @MCA_VOTE_AND: effective result is set only when every client that has
 *                   voted is enabled
 *
 * For the two set types the value is always the same as the enable state, so
 * a client never abstains and the effective client id is always valid.
 */
enum mca_votable_type {
	MCA_VOTE_MIN,
	MCA_VOTE_MAX,
	MCA_VOTE_OR,
	MCA_VOTE_AND,
	NUM_VOTABLE_TYPES,
};

bool mca_is_client_vote_enabled(struct mca_votable *votable, const char *client_str);
bool mca_is_client_vote_enabled_locked(struct mca_votable *votable,
				       const char *client_str);
bool mca_is_override_vote_enabled(struct mca_votable *votable);
bool mca_is_override_vote_enabled_locked(struct mca_votable *votable);
int mca_get_client_vote(struct mca_votable *votable, const char *client_str);
int mca_get_client_vote_locked(struct mca_votable *votable, const char *client_str);
int mca_get_effective_result(struct mca_votable *votable);
int mca_get_effective_result_locked(struct mca_votable *votable);
const char *mca_get_effective_client(struct mca_votable *votable);
const char *mca_get_effective_client_locked(struct mca_votable *votable);
int mca_vote(struct mca_votable *votable, const char *client_str, bool state,
	     int val);
int mca_vote_override(struct mca_votable *votable, const char *override_client,
		      bool state, int val);
int mca_rerun_election(struct mca_votable *votable);
struct mca_votable *mca_find_votable(const char *name);
struct mca_votable *mca_create_votable(const char *name, int votable_type,
				   int (*callback)(struct mca_votable *votable,
						   void *data,
						   int effective_result,
						   const char *effective_client),
				   int default_val, void *data);
void mca_destroy_votable(struct mca_votable *votable);
void mca_lock_votable(struct mca_votable *votable);
void mca_unlock_votable(struct mca_votable *votable);

#endif /* __MCA_COMMON_H */
