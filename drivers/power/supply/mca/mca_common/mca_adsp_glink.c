// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Talking to the ADSP about charging.  See
 * include/mca/common/mca_adsp_glink.h.
 */

#define MCA_LOG_TAG "mca_adsp_glink"

#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <mca/common/mca_adsp_glink.h>
#include <mca/common/mca_log.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rculist.h>
#include <linux/slab.h>
#include <linux/soc/qcom/qti_pmic_glink.h>
#include <linux/string.h>
#include <linux/workqueue.h>

/* Who the ADSP thinks it is talking to. */
#define MCA_GLINK_MSG_OWNER		0x800A
#define MCA_GLINK_QBG_MSG_OWNER		0x8009

/* Every exchange is a request that expects a response. */
#define MCA_GLINK_MSG_TYPE_REQ_RESP	1

#define MCA_GLINK_OPCODE_READ		1
#define MCA_GLINK_OPCODE_WRITE		2

/*
 * How long to wait for the ADSP.  Long enough that a busy ADSP still answers,
 * short enough that a charging decision is not held up behind a dead one.
 */
#define MCA_GLINK_TIMEOUT_MS		1000

/**
 * struct mca_glink_req_msg - a request for one property
 * @hdr:         who it is for and what kind of message it is
 * @property_id: which property
 * @seq_num:     matched against the response
 * @data:        the value, when writing
 */
struct mca_glink_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			seq_num;
	u8			data[MCA_GLINK_DATA_MAX];
};

/**
 * struct mca_glink_resp_msg - the answer
 * @hdr:         who it is from
 * @property_id: which property it answers about
 * @seq_num:     which request it answers
 * @ret_code:    what the ADSP made of the request
 * @data:        the value
 */
struct mca_glink_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			seq_num;
	u32			ret_code;
	u8			data[MCA_GLINK_DATA_MAX];
};

/**
 * struct mca_glink_nty_msg - something the ADSP volunteered
 * @hdr:          who it is from
 * @notification: what happened
 * @data:         what goes with it
 */
struct mca_glink_nty_msg {
	struct pmic_glink_hdr	hdr;
	u32			notification;
	u8			data[MCA_GLINK_DATA_MAX];
};

/**
 * struct mca_adsp_glink_ops_list_node - one registered listener
 * @node: links it into the listener list
 * @data: handed back to its callbacks
 * @cb:   what it wants to hear about
 */
struct mca_adsp_glink_ops_list_node {
	struct list_head		node;
	void				*data;
	struct mca_adsp_call_back	*cb;
};

/**
 * struct mca_adsp_glink_data - the two channels to the ADSP
 * @dev:             this device
 * @client:          the charging channel
 * @qbg_client:      the gauge channel
 * @rw_lock:         one exchange at a time, so a response matches the request
 * @ack:             completed when the response for @pending_prop arrives
 * @state:           whether the ADSP is up
 * @adsp_sync_work:  tells the listeners to send their state again
 * @glink_down_work: tells the listeners what they told the ADSP is gone
 * @pending_prop:    the property being waited on, -1 when idle
 * @ret_code:        what the ADSP made of the last request
 * @seq_num:         incremented per request
 * @data:            where the last response's value was copied
 */
struct mca_adsp_glink_data {
	struct device			*dev;
	struct pmic_glink_client	*client;
	struct pmic_glink_client	*qbg_client;
	struct mutex			rw_lock;
	struct completion		ack;
	atomic_t			state;
	struct work_struct		adsp_sync_work;
	struct work_struct		glink_down_work;
	u32				pending_prop;
	int				ret_code;
	u32				seq_num;
	u8				data[MCA_GLINK_DATA_MAX];
};

static struct mca_adsp_glink_data *mca_glink_data;

static LIST_HEAD(mca_adsp_glink_ops_list);
static LIST_HEAD(mca_adsp_glink_qbg_ops_list);
static DEFINE_MUTEX(mca_adsp_glink_ops_lock);

static int mca_adsp_glink_register(struct list_head *list,
				   struct mca_adsp_call_back *cb, void *data)
{
	struct mca_adsp_glink_ops_list_node *node;

	if (!cb)
		return -EINVAL;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	node->cb = cb;
	node->data = data;

	/*
	 * Publish with the rcu form: the notification path walks this list
	 * from the glink callback, which cannot take the lock.  Nothing is
	 * ever removed from the list, so a reader only has to be kept from
	 * seeing a node before the fields above are visible.
	 */
	mutex_lock(&mca_adsp_glink_ops_lock);
	list_add_tail_rcu(&node->node, list);
	mutex_unlock(&mca_adsp_glink_ops_lock);

	return 0;
}

int mca_adsp_glink_resister_ops(struct mca_adsp_call_back *cb, void *data)
{
	return mca_adsp_glink_register(&mca_adsp_glink_ops_list, cb, data);
}
EXPORT_SYMBOL(mca_adsp_glink_resister_ops);

int mca_adsp_glink_qbg_resister_ops(struct mca_adsp_call_back *cb, void *data)
{
	return mca_adsp_glink_register(&mca_adsp_glink_qbg_ops_list, cb, data);
}
EXPORT_SYMBOL(mca_adsp_glink_qbg_resister_ops);

/*
 * One exchange at a time.  The response carries the sequence number of the
 * request it answers, so a late answer to a request that already timed out is
 * dropped rather than being handed to whoever asked next.
 */
static __always_inline int mca_adsp_glink_write(struct pmic_glink_client *client, u32 owner,
				u32 opcode, u32 property_id, void *data,
				int len)
{
	struct mca_glink_req_msg msg = {};
	int ret;

	if (!mca_glink_data || !client)
		return -ENODEV;

	if (!data || len < 0 || len > MCA_GLINK_DATA_MAX)
		return -EINVAL;

	mutex_lock(&mca_glink_data->rw_lock);

	msg.hdr.owner = owner;
	msg.hdr.type = MCA_GLINK_MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = opcode;
	msg.property_id = property_id;
	msg.seq_num = ++mca_glink_data->seq_num;

	if (opcode == MCA_GLINK_OPCODE_WRITE)
		memcpy(msg.data, data, len);

	if (!atomic_read(&mca_glink_data->state)) {
		mca_log_err("glink state is down\n");
		ret = -ENOTCONN;
		goto out;
	}

	reinit_completion(&mca_glink_data->ack);
	mca_glink_data->pending_prop = property_id;

	ret = pmic_glink_write(client, &msg, sizeof(msg));
	if (ret)
		goto out;

	if (!wait_for_completion_timeout(&mca_glink_data->ack,
					 msecs_to_jiffies(MCA_GLINK_TIMEOUT_MS))) {
		mca_log_err("timed out sending message, prop_id: %d\n",
			    property_id);
		ret = -ETIMEDOUT;
		goto out;
	}

	if (mca_glink_data->ret_code) {
		ret = -EINVAL;
		goto out;
	}

	if (opcode == MCA_GLINK_OPCODE_READ)
		memcpy(data, mca_glink_data->data, len);

out:
	mca_glink_data->pending_prop = -1;
	mutex_unlock(&mca_glink_data->rw_lock);

	return ret;
}

int mca_adsp_glink_read_prop(u32 property_id, void *data, int len)
{
	return mca_adsp_glink_write(mca_glink_data ? mca_glink_data->client :
				    NULL, MCA_GLINK_MSG_OWNER,
				    MCA_GLINK_OPCODE_READ, property_id, data,
				    len);
}
EXPORT_SYMBOL(mca_adsp_glink_read_prop);

int mca_adsp_glink_write_prop(u32 property_id, void *data, int len)
{
	return mca_adsp_glink_write(mca_glink_data ? mca_glink_data->client :
				    NULL, MCA_GLINK_MSG_OWNER,
				    MCA_GLINK_OPCODE_WRITE, property_id, data,
				    len);
}
EXPORT_SYMBOL(mca_adsp_glink_write_prop);

int mca_adsp_glink_qbg_read_prop(u32 property_id, void *data, int len)
{
	return mca_adsp_glink_write(mca_glink_data ? mca_glink_data->qbg_client :
				    NULL, MCA_GLINK_QBG_MSG_OWNER,
				    MCA_GLINK_OPCODE_READ, property_id, data,
				    len);
}
EXPORT_SYMBOL(mca_adsp_glink_qbg_read_prop);

int mca_adsp_glink_qbg_write_prop(u32 property_id, void *data, int len)
{
	return mca_adsp_glink_write(mca_glink_data ? mca_glink_data->qbg_client :
				    NULL, MCA_GLINK_QBG_MSG_OWNER,
				    MCA_GLINK_OPCODE_WRITE, property_id, data,
				    len);
}
EXPORT_SYMBOL(mca_adsp_glink_qbg_write_prop);

static void mca_adsp_glink_handle_message(struct mca_glink_resp_msg *msg,
					  size_t len)
{
	if (len < sizeof(*msg))
		return;

	if (msg->seq_num != mca_glink_data->seq_num) {
		mca_log_err("invalid seq_num: %d != %d\n", msg->seq_num,
			    mca_glink_data->seq_num);
		return;
	}

	if (msg->property_id != mca_glink_data->pending_prop)
		return;

	if (msg->ret_code)
		mca_log_err("glink retcode error, property_id: %d, retcode: %d, len: %lu %lu\n",
			    msg->property_id, msg->ret_code, len,
			    sizeof(*msg));

	mca_glink_data->ret_code = msg->ret_code;
	memcpy(mca_glink_data->data, msg->data, MCA_GLINK_DATA_MAX);

	complete(&mca_glink_data->ack);
}

static void mca_adsp_glink_handle_notification(struct list_head *list,
					       struct mca_glink_nty_msg *msg,
					       size_t len)
{
	struct mca_adsp_glink_ops_list_node *node;

	if (len < sizeof(*msg))
		return;

	mca_log_info("mca_adsp receive notification %d, owner %d\n",
		     msg->notification, msg->hdr.owner);

	/*
	 * pmic_glink calls its clients under a spinlock with interrupts off,
	 * so nothing here may sleep.  The registered notify callbacks are
	 * written for that -- they allocate with GFP_ATOMIC -- and the list
	 * is only ever appended to, so walking it needs no lock of its own.
	 */
	rcu_read_lock();
	list_for_each_entry_rcu(node, list, node) {
		if (node->cb->notify_cb)
			node->cb->notify_cb(msg->notification, msg->data,
					    MCA_GLINK_DATA_MAX, node->data);
	}
	rcu_read_unlock();
}

static int mca_adsp_glink_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct list_head *list = priv;

	if (len < sizeof(*hdr))
		return -EINVAL;

	if (hdr->type == MCA_GLINK_MSG_TYPE_REQ_RESP)
		mca_adsp_glink_handle_message(data, len);
	else
		mca_adsp_glink_handle_notification(list, data, len);

	return 0;
}

/*
 * The ADSP restarting loses everything it was told, so the listeners are
 * asked to send it again once it is back.  Both run from a worker: a listener
 * that answers by asking the ADSP something would otherwise deadlock against
 * the callback it is being called from.
 */
static void mca_adsp_glink_sync_work(struct work_struct *work)
{
	struct mca_adsp_glink_ops_list_node *node;

	mutex_lock(&mca_adsp_glink_ops_lock);
	list_for_each_entry(node, &mca_adsp_glink_ops_list, node) {
		if (node->cb->sync_data_cb)
			node->cb->sync_data_cb(node->data);
	}
	list_for_each_entry(node, &mca_adsp_glink_qbg_ops_list, node) {
		if (node->cb->sync_data_cb)
			node->cb->sync_data_cb(node->data);
	}
	mutex_unlock(&mca_adsp_glink_ops_lock);
}

static void mca_adsp_glink_down_work(struct work_struct *work)
{
	struct mca_adsp_glink_ops_list_node *node;

	mutex_lock(&mca_adsp_glink_ops_lock);
	list_for_each_entry(node, &mca_adsp_glink_ops_list, node) {
		if (node->cb->glink_down_cb)
			node->cb->glink_down_cb(node->data);
	}
	list_for_each_entry(node, &mca_adsp_glink_qbg_ops_list, node) {
		if (node->cb->glink_down_cb)
			node->cb->glink_down_cb(node->data);
	}
	mutex_unlock(&mca_adsp_glink_ops_lock);
}

static void mca_adsp_glink_state_cb(void *priv, enum pmic_glink_state state)
{
	atomic_set(&mca_glink_data->state, state == PMIC_GLINK_STATE_UP);

	if (state == PMIC_GLINK_STATE_UP)
		schedule_work(&mca_glink_data->adsp_sync_work);
	else
		schedule_work(&mca_glink_data->glink_down_work);
}

static int mca_adsp_glink_probe(struct platform_device *pdev)
{
	struct pmic_glink_client_data client_data = {};

	mca_glink_data = devm_kzalloc(&pdev->dev, sizeof(*mca_glink_data),
				      GFP_KERNEL);
	if (!mca_glink_data)
		return -ENOMEM;

	mca_glink_data->dev = &pdev->dev;
	mca_glink_data->pending_prop = -1;
	mutex_init(&mca_glink_data->rw_lock);
	init_completion(&mca_glink_data->ack);
	atomic_set(&mca_glink_data->state, 1);
	INIT_WORK(&mca_glink_data->adsp_sync_work, mca_adsp_glink_sync_work);
	INIT_WORK(&mca_glink_data->glink_down_work, mca_adsp_glink_down_work);

	client_data.name = "mca_adap_glink";
	client_data.id = MCA_GLINK_MSG_OWNER;
	client_data.priv = &mca_adsp_glink_ops_list;
	client_data.msg_cb = mca_adsp_glink_callback;
	client_data.state_cb = mca_adsp_glink_state_cb;

	mca_glink_data->client = pmic_glink_register_client(&pdev->dev,
							    &client_data);
	if (IS_ERR(mca_glink_data->client)) {
		int ret = PTR_ERR(mca_glink_data->client);

		if (ret != -EPROBE_DEFER)
			mca_log_err("Error in registering with pmic_glink %d\n",
				    ret);

		return ret;
	}

	client_data.name = "mca_adap_qbg_glink";
	client_data.id = MCA_GLINK_QBG_MSG_OWNER;
	client_data.priv = &mca_adsp_glink_qbg_ops_list;

	mca_glink_data->qbg_client = pmic_glink_register_client(&pdev->dev,
								&client_data);
	if (IS_ERR(mca_glink_data->qbg_client)) {
		int ret = PTR_ERR(mca_glink_data->qbg_client);

		if (ret != -EPROBE_DEFER)
			mca_log_err("Error in registering with qbg_glink %d\n",
				    ret);

		pmic_glink_unregister_client(mca_glink_data->client);

		return ret;
	}

	mca_log_info("probe ok\n");

	return 0;
}

static int mca_adsp_glink_remove(struct platform_device *pdev)
{
	pmic_glink_unregister_client(mca_glink_data->qbg_client);
	pmic_glink_unregister_client(mca_glink_data->client);
	cancel_work_sync(&mca_glink_data->adsp_sync_work);
	cancel_work_sync(&mca_glink_data->glink_down_work);
	mca_glink_data = NULL;

	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "mca,adsp_glink" },
	{ }
};
MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver mca_adsp_glink_driver = {
	.driver = {
		.name		= "mca_adsp_glink",
		.of_match_table	= match_table,
	},
	.probe		= mca_adsp_glink_probe,
	.remove		= mca_adsp_glink_remove,
};
module_platform_driver(mca_adsp_glink_driver);

MODULE_DESCRIPTION("MCA ADSP glink");
MODULE_LICENSE("GPL");
